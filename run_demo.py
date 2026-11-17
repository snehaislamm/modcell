"""Run a 100-part shift with defects and one E-stop, print KPIs, write HMI snapshots + MES log."""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from modcell import hmi  # noqa: E402
from modcell.ladder import Controller  # noqa: E402
from modcell.plant import Station  # noqa: E402
from modcell.vision import PartSpec  # noqa: E402

rng = np.random.default_rng(42)
parts = []
for _ in range(100):
    r = rng.random()
    parts.append(PartSpec(missing_hole=int(rng.integers(0, 6))) if r < 0.06 else
                 PartSpec(robot_nok=True) if r < 0.09 else
                 PartSpec(dx_mm=rng.uniform(-3, 3), dy_mm=rng.uniform(-3, 3)))

plc = Controller(ROOT / "plc")
st = Station(plc, parts_queue=list(parts))
st.start()
wall = time.perf_counter()

st.run_until(lambda: plc.get("Cnt.Total") >= 40 and plc.get("Seq.Step") == 40, 600)
(ROOT / "hmi" / "overview_running.html").write_text(hmi.render(plc))
plc.set("I_EStop_A", 0); plc.set("I_EStop_B", 0); st.step()
(ROOT / "hmi" / "overview_estop.html").write_text(hmi.render(plc))
plc.set("I_EStop_A", 1); plc.set("I_EStop_B", 1); st.run_ms(1000)
st.start()
st.run_until(lambda: plc.get("Cnt.Total") >= 100, 1200)
st.run_ms(3000)
wall = time.perf_counter() - wall

good = [r["cycle_ms"] for r in st.mes_records if r["result"] == 1]
bad = sum(1 for p in parts if p.missing_hole is not None or p.robot_nok)
kpi = {
    "parts": int(plc.get("Cnt.Total")), "good": int(plc.get("Cnt.Good")), "reject": int(plc.get("Cnt.Reject")),
    "expected_reject": bad, "sorted_correctly": len(st.rejected) == bad,
    "sim_time_s": round(st.t, 1), "parts_per_hour": round(3600 * plc.get("Cnt.Total") / st.t),
    "mean_good_cycle_s": round(sum(good) / len(good) / 1000, 2),
    "rungs": plc.rung_count, "routines": len(plc.routines), "scans": plc.scans,
    "mean_scan_us": round(1e6 * wall / plc.scans, 1),
}
(ROOT / "mes_log.json").write_text(json.dumps(st.mes_records, indent=1))
print(json.dumps(kpi, indent=1))
