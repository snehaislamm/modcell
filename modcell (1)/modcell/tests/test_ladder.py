"""Engine and code-quality tests: instruction semantics, templates, documentation rules."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from modcell.ladder import Controller, load_routine, parse_rung  # noqa: E402

TAG = re.compile(r"\(([^()]*)\)")


class Bare(Controller):
    """Controller with a single in-memory routine."""

    def __init__(self, rungs: list[str]):
        self.scan_ms = 10
        self.tags = {}
        self.main = "M"
        self.scans = 0
        from modcell.ladder import Rung
        self.routines = {"M": [Rung("M", i, "", r, parse_rung(r)) for i, r in enumerate(rungs)]}


def test_seal_in_start_stop():
    c = Bare(["[XIC(Start) ,XIC(Motor) ]XIC(Stop)OTE(Motor);"])
    c.set("Stop", 1)
    c.scan(); assert not c.bit("Motor")
    c.set("Start", 1); c.scan(); c.set("Start", 0); c.scan()
    assert c.bit("Motor")                       # sealed in
    c.set("Stop", 0); c.scan()
    assert not c.bit("Motor")


def test_ton_timing_and_reset():
    c = Bare(["XIC(In)TON(T,250,0);"])
    c.set("In", 1)
    for _ in range(24):
        c.scan()
    assert c.bit("T.TT") and not c.bit("T.DN")
    c.scan()
    assert c.bit("T.DN") and c.get("T.ACC") == 250
    c.set("In", 0); c.scan()
    assert c.get("T.ACC") == 0 and not c.bit("T.DN")


def test_ons_and_ctu_count_edges_only():
    c = Bare(["XIC(Pb)ONS(Pb.ONS)OTE(Pulse);", "XIC(Pb)CTU(C,3,0);"])
    pulses = 0
    for level in [1, 1, 1, 0, 1, 0, 1, 1]:
        c.set("Pb", level); c.scan(); pulses += c.bit("Pulse")
    assert pulses == 3 and c.get("C.ACC") == 3 and c.bit("C.DN")


def test_nested_branch_and_compare():
    c = Bare(["[GEQ(X,10)XIC(A) ,[EQU(X,3) ,XIC(B) ] ]OTE(Q);"])
    c.set("X", 3); c.scan(); assert c.bit("Q")
    c.set("X", 12); c.scan(); assert not c.bit("Q")
    c.set("A", 1); c.scan(); assert c.bit("Q")


def test_template_instances_are_independent():
    plc = Controller(ROOT / "plc")
    assert "Cylinder_Clamp" in plc.routines and "Cylinder_Gate" in plc.routines
    txt = " ".join(r.text for r in plc.routines["Cylinder_Gate"])
    assert "{p}" not in txt and "Clamp" not in txt


def test_every_rung_is_commented():
    plc = Controller(ROOT / "plc")
    missing = [(r.routine, r.number) for rs in plc.routines.values() for r in rs if not r.comment]
    assert missing == []


def test_io_naming_convention():
    """Physical I/O must be I_* (inputs) or O_* (outputs); internal tags use Area.Name."""
    plc = Controller(ROOT / "plc")
    for rs in plc.routines.values():
        for r in rs:
            for args in TAG.findall(r.text):
                for a in (x.strip() for x in args.split(",")):
                    if not a or re.fullmatch(r"-?\d+(\.\d+)?", a) or a in plc.routines:
                        continue
                    assert re.fullmatch(r"(I|O)_[A-Za-z]+(_[A-Za-z]+)*|[A-Z][A-Za-z]*(\.[A-Za-z]+)+|(Vis|Rob)_[A-Za-z]+", a), a


def test_no_duplicate_ote_destructive_writes():
    """A coil may be driven by OTE in only one rung (classic double-coil bug)."""
    plc = Controller(ROOT / "plc")
    seen = {}
    for rs in plc.routines.values():
        for r in rs:
            for m in re.finditer(r"OTE\(([^)]*)\)", r.text):
                assert m.group(1) not in seen, f"{m.group(1)} in {seen.get(m.group(1))} and {r.routine}:{r.number}"
                seen[m.group(1)] = f"{r.routine}:{r.number}"
