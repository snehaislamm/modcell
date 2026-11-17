"""Overview screen in ISA-101 'high-performance HMI' style.

Grey everywhere; colour only for abnormal states. Every screen answers
'what is the station waiting on?' so an operator or tech can troubleshoot
without opening the PLC program.
"""
from __future__ import annotations

import html
import json
from pathlib import Path

WAIT = json.loads((Path(__file__).resolve().parents[1] / "hmi" / "wait_codes.json").read_text())
ALARMS = {1: "Clamp cylinder fault (motion timeout / sensor)", 2: "Stop gate cylinder fault",
          3: "Vision timeout - no result in 1 s", 4: "Robot timeout - cycle > 8 s",
          5: "MES timeout - no ack in 2 s", 6: "Conveyor VFD fault"}
STEPS = [(0, "Home"), (10, "Wait part"), (20, "Clamp"), (30, "Inspect"), (40, "Fasten"),
         (50, "Unclamp"), (60, "Release"), (70, "Report")]


def render(plc, title: str = "MODCELL ST10 - Module Fastening") -> str:
    g = plc.get
    step = int(g("Seq.Step"))
    wait = WAIT.get(str(int(g("HMI.WaitCode"))), {"text": "-", "check": ""})
    first = int(g("Alm.FirstOut"))
    banner = ""
    if not plc.bit("Saf.OK"):
        banner = '<div class="ab hi">SAFETY CIRCUIT OPEN - check E-stops / guard door, then press SAFETY RESET</div>'
    elif first:
        banner = f'<div class="ab hi">FIRST-OUT ALARM {first}: {html.escape(ALARMS[first])}</div>'
    chips = "".join(
        f'<div class="st{" on" if s == step else ""}">{s:02d}<br>{n}</div>' for s, n in STEPS)

    def io(label, tag):
        return f'<tr><td>{label}</td><td class="{"v1" if plc.bit(tag) else "v0"}">{"ON" if plc.bit(tag) else "off"}</td></tr>'

    total, good, rej = int(g("Cnt.Total")), int(g("Cnt.Good")), int(g("Cnt.Reject"))
    fpy = f"{100 * good / total:.1f}%" if total else "-"
    return f"""<!DOCTYPE html><html><head><meta charset="utf-8"><title>{title}</title><style>
body{{background:#d4d4d4;font:14px Arial,sans-serif;color:#222;margin:0;padding:16px}}
h1{{font-size:18px;margin:0 0 8px}} .ab{{padding:10px;margin-bottom:10px;font-weight:bold}}
.hi{{background:#e8a33d;color:#000;border-left:8px solid #b00}}
.row{{display:flex;gap:12px}} .card{{background:#e6e6e6;border:1px solid #aaa;padding:10px;flex:1}}
.st{{display:inline-block;width:74px;text-align:center;padding:6px 0;margin:2px;background:#ccc;border:1px solid #999;font-size:12px}}
.on{{background:#fff;border:2px solid #333;font-weight:bold}} .big{{font-size:22px;font-weight:bold}}
table{{border-collapse:collapse;width:100%}} td{{padding:3px 6px;border-bottom:1px solid #ccc}}
.v1{{font-weight:bold}} .v0{{color:#777}}
</style></head><body>
<h1>{title}</h1>{banner}
<div class="card"><b>Sequence</b><br>{chips}</div><br>
<div class="row">
 <div class="card"><b>Waiting on</b><div class="big">{html.escape(wait['text'])}</div>
  <div>Check: {html.escape(wait['check'])}</div></div>
 <div class="card"><b>Shift</b><table><tr><td>Total</td><td>{total}</td></tr>
  <tr><td>Good</td><td>{good}</td></tr><tr><td>Reject</td><td>{rej}</td></tr>
  <tr><td>First-pass yield</td><td>{fpy}</td></tr>
  <tr><td>Last cycle</td><td>{g('Seq.LastCycleMs') / 1000:.2f} s</td></tr></table></div>
 <div class="card"><b>Key I/O</b><table>{io('Part at stop', 'I_PE_AtStop')}{io('Clamp extended', 'I_Clamp_Ext')}
  {io('Gate up', 'I_Gate_Ext')}{io('Robot busy', 'I_Rob_Busy')}{io('Vision pass', 'I_Vis_Pass')}
  {io('Conveyor run', 'O_Conv_Run')}{io('Safety OK', 'Saf.OK')}</table></div>
</div></body></html>"""
