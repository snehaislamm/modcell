# modcell: controls package for one "unboxed" module-assembly station

A fenced robot cell that receives a module bracket on a pallet conveyor, clamps it, inspects it with a
camera, hands the measured X/Y offset to a fastening robot, unclamps, diverts rejects and reports every
part to the MES. Inspired by assembling vehicle modules in parallel rather than on one long line.

**What's real here:** the ladder logic is written in Rockwell neutral text (the format you can paste
into a Studio 5000 rung). It runs on a small scan-cycle engine (`modcell/ladder.py`) against a plant
model (conveyor, cylinders, camera, robot, MES) at a 10 ms scan. **What isn't:** no vendor PLC, robot or
camera hardware; the panel and safety documents are design studies.

## Run it
```
pip install numpy pytest
python -m pytest -q          # 25 tests
python run_demo.py           # 100-part shift with defects + one E-stop -> KPIs, hmi/*.html, mes_log.json
```

## Results (from run_demo.py and tests)
| Metric | Value |
|---|---|
| Ladder | 97 rungs, 12 routines, every rung commented (enforced by a test) |
| Reuse | one `Cylinder` template instanced for clamp and stop gate |
| Station cycle (good part) | 6.08 s in-station, ~520 parts/h incl. indexing |
| 100-part shift, 10 defects | 10/10 rejects diverted, 90 good, 100 MES records |
| Vision job | 300/300 random parts classified correctly, offset error <= 0.5 mm |
| E-stop | robot enable, drive STO, dump valve off within 1 scan (10 ms); no auto restart |
| Device faults | stuck/shorted cylinder, vision, robot, MES, VFD -> latched alarm + first-out code |

## Layout
```
plc/program.json           controller manifest: scan order, template instances, initial values
plc/routines/*.rll         MainRoutine, Safety, Modes, Sequence, Conveyor, Vision, Robot, MES, Alarms, Indicators
plc/templates/Cylinder.rll AOI-style reusable device logic ({p} = instance prefix)
modcell/ladder.py          scan engine: XIC XIO OTE OTL OTU ONS TON CTU RES MOV ADD SUB EQU..LEQ JSR, branches
modcell/plant.py           conveyor/pallets, cylinders, camera, robot, MES simulator
modcell/vision.py          smart-camera job: locate plate, count holes, pass/fail + 2D offset
modcell/hmi.py             ISA-101 style overview screen; hmi/wait_codes.json = "waiting on" text
docs/io_list.csv           I/O list
docs/safety.md             RIA R15.06 / ISO 13849 safeguarding concept + test traceability
docs/panel_layout.md       NFPA 79 / UL 508A panel layout, wire colours, BOM
```

## Sequence
| Step | Name | Moves on when |
|---|---|---|
| 0 | Home | clamp retracted, gate up, auto on |
| 10 | Wait for part | at-stop eye on for 200 ms |
| 20 | Clamp | clamp extended sensor |
| 30 | Inspect | camera done -> pass: copy offsets to robot / fail: result 2 |
| 40 | Fasten | robot done -> OK: result 1 / NOK: result 3 |
| 50 | Unclamp | clamp retracted -> set divert if bad, drop gate |
| 60 | Release | at-stop eye clear for 300 ms -> gate up |
| 70 | Report | MES ack -> count, back to 10 |

## Coding rules (checked in tests/test_ladder.py)
- Physical I/O named `I_*` / `O_*`; internal tags `Area.Name`.
- No coil driven by `OTE` in more than one rung.
- Every rung has a comment.
- Safety logic in the PLC only monitors; the hardwired safety relay does the stopping (docs/safety.md).

## Next step
Port the routines to a vendor runtime (CODESYS Control Win or Studio 5000 emulator) and drive it from
`plant.py` over Modbus TCP / OPC UA, so the same tests run against a real PLC (virtual commissioning).
