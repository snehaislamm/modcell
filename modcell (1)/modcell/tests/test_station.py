"""Station-level tests: normal production, rejects, guidance, safety, device faults, MES.

Each test drives the real ladder (plc/) through the plant model with 10 ms scans.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from modcell.ladder import Controller  # noqa: E402
from modcell.plant import Station  # noqa: E402
from modcell.vision import PartSpec, inspect, render  # noqa: E402


def make(parts, **kw):
    plc = Controller(ROOT / "plc")
    st = Station(plc, parts_queue=list(parts), **kw)
    return plc, st


def produce(plc, st, n, timeout=200):
    return st.run_until(lambda: plc.get("Cnt.Total") >= n, timeout)


# ---------------------------------------------------------------- production
def test_ten_good_parts_cycle_time_and_mes():
    plc, st = make([PartSpec() for _ in range(10)])
    st.start()
    assert produce(plc, st, 10)
    st.run_ms(3000)
    assert plc.get("Cnt.Good") == 10 and plc.get("Cnt.Reject") == 0
    assert [r["serial"] for r in st.mes_records] == list(range(1, 11))
    assert all(r["result"] == 1 for r in st.mes_records)
    assert all(r["cycle_ms"] < 7000 for r in st.mes_records)      # in-station cycle
    assert len(st.done) == 10 and not st.rejected
    assert st.robot_started == 10


def test_vision_reject_skips_robot_and_diverts():
    plc, st = make([PartSpec(missing_hole=3)])
    st.start()
    assert produce(plc, st, 1)
    st.run_ms(3000)
    assert st.mes_records[0]["result"] == 2
    assert st.robot_started == 0
    assert len(st.rejected) == 1 and plc.get("Cnt.Reject") == 1


def test_robot_nok_is_rejected():
    plc, st = make([PartSpec(robot_nok=True)])
    st.start()
    assert produce(plc, st, 1)
    st.run_ms(3000)
    assert st.mes_records[0]["result"] == 3 and len(st.rejected) == 1


def test_vision_offset_guides_robot():
    plc, st = make([PartSpec(dx_mm=2.5, dy_mm=-1.5)])
    st.start()
    assert st.run_until(lambda: plc.get("Seq.Step") == 40, 30)
    assert abs(plc.get("Rob_OffX") / 10 - 2.5) <= 0.5
    assert abs(plc.get("Rob_OffY") / 10 + 1.5) <= 0.5


def test_offset_outside_robot_window_rejects():
    plc, st = make([PartSpec(dx_mm=6.0)])
    st.start()
    assert produce(plc, st, 1)
    assert st.mes_records[0]["result"] == 2


def test_cycle_stop_finishes_current_part():
    plc, st = make([PartSpec() for _ in range(3)])
    st.start()
    assert st.run_until(lambda: plc.get("Seq.Step") == 40, 30)
    st.press_nc("I_StopPB")
    assert plc.bit("Mode.Auto")                     # still running mid-part
    st.run_ms(15000)
    assert plc.get("Cnt.Total") == 1                # finished that part, then stopped
    assert not plc.bit("Mode.Auto") and plc.get("HMI.WaitCode") == 2


# ---------------------------------------------------------------- safety
def test_estop_mid_fasten_stops_outputs_in_one_scan_and_needs_reset():
    plc, st = make([PartSpec()])
    st.start()
    assert st.run_until(lambda: plc.bit("I_Rob_Busy"), 30)
    plc.set("I_EStop_A", 0)
    plc.set("I_EStop_B", 0)
    st.step()                                       # one 10 ms scan
    for out in ("O_Rob_Enable", "O_Conv_SafeEnable", "O_Valve_Dump", "O_Rob_CycleStart",
                "O_Clamp_ExtSol", "O_Clamp_RetSol"):
        assert not plc.bit(out), out
    assert plc.get("HMI.WaitCode") == 99 and plc.bit("O_Light_Red")
    plc.set("I_EStop_A", 1)
    plc.set("I_EStop_B", 1)
    st.run_ms(2000)
    assert not plc.bit("Saf.OK") and not plc.bit("Mode.Auto")   # no automatic restart
    st.press("I_StartPB")
    assert not plc.bit("Mode.Auto")                 # start refused until safety reset
    st.start()                                      # reset + start
    assert produce(plc, st, 1, 30)
    assert st.mes_records[0]["result"] == 1         # job re-run and completed


def test_estop_channel_discrepancy_latches_and_blocks_reset():
    plc, st = make([])
    st.start()
    plc.set("I_EStop_B", 0)                         # single-channel fault (welded contact on A)
    st.run_ms(400)
    assert not plc.bit("Saf.DiscFault")
    st.run_ms(200)
    assert plc.bit("Saf.DiscFault") and not plc.bit("Saf.OK")
    st.press("I_SafetyResetPB")
    assert not plc.bit("Saf.OK")                    # reset refused while channels disagree
    plc.set("I_EStop_B", 1)
    st.run_ms(100)
    assert plc.bit("Saf.DiscFault")                 # still latched after channels agree
    st.press("I_SafetyResetPB")
    assert plc.bit("Saf.OK") and not plc.bit("Saf.DiscFault")


def test_guard_door_open_stops_cell():
    plc, st = make([PartSpec()])
    st.start()
    assert st.run_until(lambda: plc.get("Seq.Step") == 20, 30)
    plc.set("I_Door_A", 0)
    plc.set("I_Door_B", 0)
    st.step()
    assert not plc.bit("O_Rob_Enable") and not plc.bit("Mode.Auto")


# ---------------------------------------------------------------- device faults
def test_stuck_clamp_alarm_first_out_and_recovery():
    plc, st = make([PartSpec(), PartSpec()])
    st.clamp.stuck = True
    st.start()
    assert st.run_until(lambda: plc.bit("Clamp.Fault"), 30)
    st.step()                                       # auto drops on the next scan
    assert plc.get("Alm.FirstOut") == 1 and plc.get("HMI.WaitCode") == 90
    assert not plc.bit("Mode.Auto")
    st.clamp.stuck = False
    st.press("I_FaultResetPB")
    assert not plc.bit("Alm.Any")
    st.press("I_StartPB")
    assert produce(plc, st, 2, 40)


def test_shorted_cylinder_sensor_faults_in_100ms():
    plc, st = make([])
    st.start()
    st.gate.sensor_short = True
    st.run_ms(150)
    assert plc.bit("Gate.Fault") and plc.get("Alm.FirstOut") == 2


def test_mes_offline_times_out():
    plc, st = make([PartSpec()], mes_online=False)
    st.start()
    assert st.run_until(lambda: plc.bit("Alm.MESTmo"), 30)
    assert plc.get("Alm.FirstOut") == 5 and plc.get("Seq.Step") == 70


def test_vfd_fault_stops_conveyor():
    plc, st = make([PartSpec()])
    st.start()
    st.run_ms(300)
    st.conv_fault = True
    st.run_ms(30)
    assert not plc.bit("O_Conv_Run") and plc.get("Alm.FirstOut") == 6


# ---------------------------------------------------------------- vision job
def test_vision_job_accuracy_over_300_random_parts():
    rng = np.random.default_rng(1)
    errs, correct = [], 0
    for i in range(300):
        spec = PartSpec(dx_mm=rng.uniform(-3.5, 3.5), dy_mm=rng.uniform(-3.5, 3.5),
                        missing_hole=int(rng.integers(0, 6)) if i % 4 == 0 else None)
        res = inspect(render(spec, rng))
        correct += res.passed == (spec.missing_hole is None)
        errs.append(max(abs(res.off_x_mm - spec.dx_mm), abs(res.off_y_mm - spec.dy_mm)))
    assert correct == 300
    assert max(errs) <= 0.5


@pytest.mark.parametrize("seed", range(3))
def test_mixed_shift_of_40_parts(seed):
    rng = np.random.default_rng(seed)
    parts = [PartSpec(missing_hole=2) if rng.random() < 0.1 else
             PartSpec(robot_nok=True) if rng.random() < 0.05 else PartSpec(dx_mm=rng.uniform(-2, 2))
             for _ in range(40)]
    plc, st = make(parts, seed=seed)
    st.start()
    assert produce(plc, st, 40, 600)
    st.run_ms(3000)
    expected_bad = sum(p.missing_hole is not None or p.robot_nok for p in parts)
    assert plc.get("Cnt.Reject") == expected_bad == len(st.rejected)
    assert not plc.bit("Alm.Any")
