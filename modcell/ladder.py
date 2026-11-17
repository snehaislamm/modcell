"""Scan-cycle engine for Rockwell-style neutral-text ladder logic.

Rungs use the same text you can paste into a Studio 5000 / RSLogix 5000 rung:
    XIC(Start)[XIC(Run) ,XIC(Jog) ]XIO(Stop)OTE(Run);

Supported instructions
  bit:      XIC XIO OTE OTL OTU ONS
  timer:    TON(timer,preset_ms,0)    members .EN .TT .DN .ACC .PRE
  counter:  CTU(counter,preset,0)     members .CU .DN .ACC .PRE
  reset:    RES(timer_or_counter)
  compare:  EQU NEQ GRT GEQ LES LEQ (a,b)
  math/move: MOV(src,dst) ADD(a,b,dst) SUB(a,b,dst)
  program:  JSR(routine)

Templates (AOI-style reuse): a routine file may use {p} as a tag prefix.
Each instance in program.json substitutes its own prefix, so one tested
routine (e.g. Cylinder) drives every cylinder on the station.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

TOKEN = re.compile(r"\s*(?:(?P<instr>[A-Z]{2,4})\((?P<args>[^()]*)\)|(?P<sym>[\[\],;]))")

OUTPUTS = {"OTE", "OTL", "OTU", "TON", "CTU", "RES", "MOV", "ADD", "SUB", "JSR"}


@dataclass
class Instr:
    op: str
    args: list[str]


@dataclass
class Branch:
    legs: list[list]  # each leg is a list of Instr | Branch


@dataclass
class Rung:
    routine: str
    number: int
    comment: str
    text: str
    elements: list = field(default_factory=list)


def parse_rung(text: str) -> list:
    """Parse one rung's neutral text into nested series/branch elements."""
    pos = 0
    tokens = []
    while pos < len(text):
        m = TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise SyntaxError(f"bad ladder text near: {text[pos:pos+30]!r}")
        pos = m.end()
        if m.group("instr"):
            args = [a.strip() for a in m.group("args").split(",")] if m.group("args").strip() else []
            tokens.append(Instr(m.group("instr"), args))
        else:
            tokens.append(m.group("sym"))

    def series(i: int, stop: set[str]):
        out = []
        while i < len(tokens):
            t = tokens[i]
            if isinstance(t, str) and t in stop:
                return out, i
            if t == "[":
                legs = []
                i += 1
                while True:
                    leg, i = series(i, {",", "]"})
                    legs.append(leg)
                    if tokens[i] == "]":
                        i += 1
                        break
                    i += 1  # skip ','
                out.append(Branch(legs))
                continue
            if t == ";":
                return out, i
            out.append(t)
            i += 1
        return out, i

    elements, _ = series(0, {";"})
    return elements


def load_routine(path: Path, prefix: str | None = None, name: str | None = None) -> list[Rung]:
    """Read a .rll file: '// comment' lines attach to the next rung; rungs end with ';'."""
    rungs, comment, buf = [], [], []
    rname = name or path.stem
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("//"):
            comment.append(line[2:].strip())
            continue
        buf.append(line)
        if line.endswith(";"):
            text = " ".join(buf)
            if prefix is not None:
                text = text.replace("{p}", prefix)
            c = " ".join(comment)
            if prefix is not None:
                c = c.replace("{p}", prefix)
            rungs.append(Rung(rname, len(rungs), c, text, parse_rung(text)))
            comment, buf = [], []
    return rungs


class Controller:
    """A minimal PLC: tag database + routines executed once per scan, top to bottom."""

    def __init__(self, plc_dir: Path, scan_ms: int = 10):
        self.scan_ms = scan_ms
        self.tags: dict[str, float] = {}
        self.routines: dict[str, list[Rung]] = {}
        manifest = json.loads((plc_dir / "program.json").read_text())
        for r in manifest["routines"]:
            self.routines[r] = load_routine(plc_dir / "routines" / f"{r}.rll")
        for inst in manifest.get("instances", []):
            name = f"{inst['template']}_{inst['prefix']}"
            self.routines[name] = load_routine(
                plc_dir / "templates" / f"{inst['template']}.rll", inst["prefix"], name)
        for k, v in manifest.get("init", {}).items():
            self.tags[k] = v
        self.main = manifest["main"]
        self.scans = 0

    # ---- tag access -------------------------------------------------------
    def get(self, ref: str) -> float:
        try:
            return float(ref)
        except ValueError:
            return self.tags.get(ref, 0)

    def set(self, tag: str, value) -> None:
        self.tags[tag] = value

    def bit(self, tag: str) -> bool:
        return bool(self.tags.get(tag, 0))

    @property
    def rung_count(self) -> int:
        return sum(len(r) for r in self.routines.values())

    # ---- execution --------------------------------------------------------
    def scan(self) -> None:
        self._run(self.main)
        self.scans += 1

    def _run(self, routine: str) -> None:
        for rung in self.routines[routine]:
            self._series(rung.elements, True)

    def _series(self, elements: list, rin: bool) -> bool:
        cond = rin
        for el in elements:
            if isinstance(el, Branch):
                cond = any([self._series(leg, cond) for leg in el.legs])  # evaluate every leg
            else:
                cond = self._exec(el, cond)
        return cond

    def _exec(self, ins: Instr, rin: bool) -> bool:
        op, a, t = ins.op, ins.args, self.tags
        if op == "XIC":
            return rin and self.bit(a[0])
        if op == "XIO":
            return rin and not self.bit(a[0])
        if op == "OTE":
            t[a[0]] = 1 if rin else 0
            return rin
        if op == "OTL":
            if rin:
                t[a[0]] = 1
            return rin
        if op == "OTU":
            if rin:
                t[a[0]] = 0
            return rin
        if op == "ONS":
            out = rin and not self.bit(a[0])
            t[a[0]] = 1 if rin else 0
            return out
        if op == "TON":
            tm, pre = a[0], self.get(a[1])
            t[f"{tm}.PRE"] = pre
            if rin:
                acc = min(self.get(f"{tm}.ACC") + self.scan_ms, pre)
                t[f"{tm}.ACC"] = acc
                t[f"{tm}.EN"] = 1
                t[f"{tm}.DN"] = 1 if acc >= pre else 0
                t[f"{tm}.TT"] = 0 if acc >= pre else 1
            else:
                t[f"{tm}.ACC"] = 0
                t[f"{tm}.EN"] = t[f"{tm}.TT"] = t[f"{tm}.DN"] = 0
            return rin
        if op == "CTU":
            c, pre = a[0], self.get(a[1])
            t[f"{c}.PRE"] = pre
            if rin and not self.bit(f"{c}.CU"):
                t[f"{c}.ACC"] = self.get(f"{c}.ACC") + 1
            t[f"{c}.CU"] = 1 if rin else 0
            t[f"{c}.DN"] = 1 if self.get(f"{c}.ACC") >= pre else 0
            return rin
        if op == "RES":
            if rin:
                for m in ("ACC", "DN", "TT", "EN", "CU"):
                    t[f"{a[0]}.{m}"] = 0
            return rin
        if op in ("EQU", "NEQ", "GRT", "GEQ", "LES", "LEQ"):
            x, y = self.get(a[0]), self.get(a[1])
            res = {"EQU": x == y, "NEQ": x != y, "GRT": x > y,
                   "GEQ": x >= y, "LES": x < y, "LEQ": x <= y}[op]
            return rin and res
        if op == "MOV":
            if rin:
                t[a[1]] = self.get(a[0])
            return rin
        if op == "ADD":
            if rin:
                t[a[2]] = self.get(a[0]) + self.get(a[1])
            return rin
        if op == "SUB":
            if rin:
                t[a[2]] = self.get(a[0]) - self.get(a[1])
            return rin
        if op == "JSR":
            if rin:
                self._run(a[0])
            return rin
        raise ValueError(f"unsupported instruction {op}")
