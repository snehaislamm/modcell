# Safeguarding concept (design study)

Scope: one robot fastening cell, fenced, one interlocked guard door, one E-stop at the operator panel.
References: ANSI/RIA R15.06-2012 (adopts ISO 10218-1/-2), ISO 13849-1 (PL), NFPA 79 §9.2 (stop categories), OSHA 1910.147 (lockout for maintenance, not covered by this logic).

## Risk assessment (abridged)
| Hazard | Task | S / F / P | PLr | Measure |
|---|---|---|---|---|
| Robot impact / crush | Operator enters cell to clear a jam | S2 F2 P2 | **d** | Coded dual-channel door interlock -> safety relay -> robot auto-stop + drive STO |
| Clamp pinch | Hand near fixture | S1 F2 P1 | b | Safety dump valve exhausts air on stop; guard keeps hands out in auto |
| Conveyor entanglement | Clearing pallet at stop | S1 F2 P2 | c | VFD STO through safety relay |
| Any of the above | Emergency | — | **d** | E-stop, 2 NC channels, Stop Category 0 (STO / dump), manual reset |

## Architecture
- **Category 3 / PL d**: two channels per device, cross-monitored by a safety relay (e.g. PNOZ-style), single fault does not cause loss of the safety function, discrepancy detected.
- The safety relay's output contacts sit **in series** with `O_Rob_Enable`, `O_Conv_SafeEnable` and `O_Valve_Dump`. The standard PLC can only drop them, never force them on. This is the "control reliable" principle: the PLC is not the safety system.
- On a GuardLogix / safety PLC this logic would live in the Safety Task with SIL-rated instructions (ESTOP, DCS, Manual Reset); `plc/routines/Safety.rll` mirrors the same behaviour for sequencing and diagnostics.

## Behaviours verified in tests/test_station.py
| Requirement | Test |
|---|---|
| E-stop drops robot enable, drive STO, dump valve and solenoids within one 10 ms scan | `test_estop_mid_fasten_stops_outputs_in_one_scan_and_needs_reset` |
| No automatic restart after E-stop release; START refused until SAFETY RESET | same |
| Channel discrepancy > 500 ms latches a fault; reset refused until channels agree and reset is pressed again | `test_estop_channel_discrepancy_latches_and_blocks_reset` |
| Guard door open in auto stops the cell | `test_guard_door_open_stops_cell` |
| Interrupted robot job re-runs from the start after recovery | E-stop test (part completes as good) |
