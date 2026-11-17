"""Station plant model: conveyor + pallets, two pneumatic cylinders, camera, robot, MES.

Each tick the plant reads PLC outputs and writes PLC inputs, like a rest-of-station
simulator in a virtual-commissioning setup.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .ladder import Controller
from .vision import PartSpec, inspect, render

STOP_X = 1.00        # m, stop gate position
EXIT_X = 1.60        # m, part leaves the station
DIVERT_X = 1.08      # m, reject diverter just past the stop
SPAWN_GAP = 0.45     # m, min spacing before the next pallet enters
MAX_SPEED = 0.50     # m/s at 100 % speed reference


@dataclass
class Cylinder:
    name: str
    pos: float = 0.0                 # 0 = retracted, 1 = extended
    stroke_s: float = 0.35
    stuck: bool = False
    sensor_short: bool = False       # both sensors on

    def tick(self, plc: Controller, dt: float) -> None:
        ext, ret = plc.bit(f"O_{self.name}_ExtSol"), plc.bit(f"O_{self.name}_RetSol")
        if not self.stuck:
            if ext and not ret:
                self.pos = min(1.0, self.pos + dt / self.stroke_s)
            elif ret and not ext:
                self.pos = max(0.0, self.pos - dt / self.stroke_s)
        plc.set(f"I_{self.name}_Ext", 1 if self.pos >= 0.98 or self.sensor_short else 0)
        plc.set(f"I_{self.name}_Ret", 1 if self.pos <= 0.02 or self.sensor_short else 0)


@dataclass
class Part:
    spec: PartSpec
    x: float = 0.0
    serial: int = 0
    route: str | None = None         # set when the pallet passes the reject diverter


@dataclass
class Station:
    plc: Controller
    seed: int = 7
    parts_queue: list[PartSpec] = field(default_factory=list)
    robot_s_per_fastener: float = 0.7
    vision_ms: int = 150
    mes_online: bool = True
    conv_fault: bool = False

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.clamp = Cylinder("Clamp")
        self.gate = Cylinder("Gate", pos=1.0)
        self.parts: list[Part] = []
        self.done: list[Part] = []
        self.rejected: list[Part] = []
        self.mes_records: list[dict] = []
        self.t = 0.0
        self.serial = 0
        # camera / robot / MES internal state
        self._vis_t = None
        self._vis_prev_trig = 0
        self._rob_t = None
        self._rob_prev_start = 0
        self._mes_t = None
        self.robot_started = 0
        self.vision_log: list = []
        # healthy safety circuit, reset, start
        for tag in ("I_EStop_A", "I_EStop_B", "I_Door_A", "I_Door_B", "I_StopPB"):
            self.plc.set(tag, 1)

    # ---- helpers for scenarios -----------------------------------------
    def press(self, tag: str, ms: int = 50) -> None:
        self.plc.set(tag, 1)
        self.run_ms(ms)
        self.plc.set(tag, 0)
        self.run_ms(20)

    def press_nc(self, tag: str, ms: int = 50) -> None:     # NC button: 0 = pressed
        self.plc.set(tag, 0)
        self.run_ms(ms)
        self.plc.set(tag, 1)
        self.run_ms(20)

    def start(self) -> None:
        self.press("I_SafetyResetPB")
        self.press("I_StartPB")

    def run_ms(self, ms: int) -> None:
        for _ in range(int(ms // self.plc.scan_ms)):
            self.step()

    def run_until(self, cond, timeout_s: float) -> bool:
        end = self.t + timeout_s
        while self.t < end:
            if cond():
                return True
            self.step()
        return cond()

    # ---- one 10 ms tick: inputs -> scan -> plant response ----------------
    def step(self) -> None:
        dt = self.plc.scan_ms / 1000
        self.plc.scan()
        self._conveyor(dt)
        self.clamp.tick(self.plc, dt)
        self.gate.tick(self.plc, dt)
        self._camera()
        self._robot(dt)
        self._mes()
        self.t += dt

    def _conveyor(self, dt: float) -> None:
        p = self.plc
        p.set("I_Conv_Fault", 1 if self.conv_fault else 0)
        moving = p.bit("O_Conv_Run") and p.bit("O_Conv_SafeEnable") and not self.conv_fault
        v = MAX_SPEED * p.get("O_Conv_SpeedRef") / 100 if moving else 0.0
        if self.parts_queue and (not self.parts or min(q.x for q in self.parts) > SPAWN_GAP):
            self.serial += 1
            self.parts.append(Part(self.parts_queue.pop(0), 0.0, self.serial))
        for q in sorted(self.parts, key=lambda q: -q.x):
            nx = q.x + v * dt
            ahead = [o.x for o in self.parts if o.x > q.x]
            if ahead:
                nx = min(nx, min(ahead) - 0.30)                  # pallets queue, no overlap
            if self.gate.pos > 0.5 and q.x <= STOP_X:
                nx = min(nx, STOP_X)                              # gate blocks
            if self.clamp.pos > 0.05 and abs(q.x - STOP_X) < 0.01:
                nx = q.x                                          # clamped part can't move
            q.x = max(q.x, nx)
        for q in self.parts:
            if q.route is None and q.x >= DIVERT_X:
                q.route = "reject" if p.bit("O_Divert_Reject") else "good"
        for q in [q for q in self.parts if q.x >= EXIT_X]:
            self.parts.remove(q)
            (self.rejected if q.route == "reject" else self.done).append(q)
        p.set("I_PE_AtStop", 1 if any(STOP_X - 0.004 <= q.x <= STOP_X + 0.02 for q in self.parts) else 0)
        p.set("I_PE_Entry", 1 if any(q.x < 0.10 for q in self.parts) else 0)

    def _part_at_stop(self):
        for q in self.parts:
            if abs(q.x - STOP_X) < 0.01:
                return q
        return None

    def _camera(self) -> None:
        p = self.plc
        trig = p.bit("O_Vis_Trigger")
        if trig and not self._vis_prev_trig and self._vis_t is None:
            self._vis_t = 0
            p.set("I_Vis_Busy", 1)
        if self._vis_t is not None:
            self._vis_t += p.scan_ms
            if self._vis_t >= self.vision_ms:
                part = self._part_at_stop()
                res = inspect(render(part.spec, self.rng)) if part else None
                self.vision_log.append((part.spec if part else None, res))
                p.set("I_Vis_Pass", 1 if res and res.passed else 0)
                p.set("Vis_OffX", round(res.off_x_mm * 10) if res else 0)   # 0.1 mm units
                p.set("Vis_OffY", round(res.off_y_mm * 10) if res else 0)
                p.set("I_Vis_Busy", 0)
                p.set("I_Vis_Done", 1)
                self._vis_t = None
        if not trig and p.bit("I_Vis_Done"):
            p.set("I_Vis_Done", 0)
        self._vis_prev_trig = trig
        p.set("I_Vis_Ready", 0 if (self._vis_t is not None or p.bit("I_Vis_Done")) else 1)

    def _robot(self, dt: float) -> None:
        p = self.plc
        enabled = p.bit("O_Rob_Enable")
        start = p.bit("O_Rob_CycleStart")
        if not enabled and self._rob_t is not None:            # protective stop aborts the job
            self._rob_t = None
            p.set("I_Rob_Busy", 0)
        if enabled and start and not self._rob_prev_start and self._rob_t is None and not p.bit("I_Rob_Done"):
            self._rob_t = 0.0
            self.robot_started += 1
            p.set("I_Rob_Busy", 1)
        if self._rob_t is not None:
            self._rob_t += dt
            if self._rob_t >= 0.4 + 6 * self.robot_s_per_fastener:
                part = self._part_at_stop()
                p.set("I_Rob_OK", 0 if (part is None or part.spec.robot_nok) else 1)
                p.set("I_Rob_Busy", 0)
                p.set("I_Rob_Done", 1)
                self._rob_t = None
        if p.bit("I_Rob_Done") and p.get("Seq.Step") != 40:
            p.set("I_Rob_Done", 0)
        self._rob_prev_start = start
        p.set("I_Rob_Ready", 1 if enabled and self._rob_t is None and not p.bit("I_Rob_Done") else 0)

    def _mes(self) -> None:
        p = self.plc
        if p.bit("MES.Req") and self.mes_online and not p.bit("MES.Ack"):
            self._mes_t = (self._mes_t or 0) + p.scan_ms
            if self._mes_t >= 50:
                self.mes_records.append({"serial": int(p.get("MES.Serial")),
                                         "result": int(p.get("MES.Result")),
                                         "cycle_ms": int(p.get("MES.CycleMs"))})
                p.set("MES.Ack", 1)
                self._mes_t = None
        if not p.bit("MES.Req") and p.bit("MES.Ack"):
            p.set("MES.Ack", 0)
