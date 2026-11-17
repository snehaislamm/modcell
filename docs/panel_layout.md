# Control panel design study (NFPA 79 / UL 508A / NEC Art. 409)

24 VDC control, 480 VAC 3-phase feed for the conveyor VFD. Enclosure 36"x30"x10", NEMA 12.

## Layout (backplate, top to bottom)
```
+--------------------------------------------------------------+
| [MAIN DISC.]  [Branch fuses]  [VFD 1 hp]   |  wire duct       |
|----------------- 480 VAC zone ------------ | ---------------- |
| [24VDC PSU 10A] [Circuit protection 24V x4] [Safety relay]    |
|------------------- 24 VDC zone ------------------------------ |
| [PLC CPU][DI 16][DI 16][DO 16]  [Ethernet switch, managed]    |
|---------------------------------------------------------------|
| Terminal strips X1 (field DI)  X2 (field DO)  X3 (safety)      |
+--------------------------------------------------------------+
Door: HMI 10", E-stop, Start/Stop, Fault Reset, Safety Reset, main disconnect handle
```
- Power and control segregated by duct; 480 V and 24 V never share a duct.
- Wire colours per NFPA 79 §13.2.4: black = AC power, red = AC control, blue = DC control, orange = externally powered interlock, white/grey = grounded neutral, green/yellow = equipment ground.
- Every field device lands on a numbered terminal; wire numbers = terminal numbers (traceable on the schematic).
- SCCR: panel rating = lowest component SCCR after applying UL 508A Supplement SB; check against available fault current at the feed.
- 20 % spare I/O and 25 % spare duct fill for later changes.

## BOM (main items)
| Qty | Item | Why |
|---|---|---|
| 1 | Fused main disconnect, door-interlocked | NFPA 79 §5.3 supply disconnect |
| 1 | 24 VDC 10 A PSU + 4-ch electronic circuit protection | separate trip per load group |
| 1 | Dual-channel safety relay, 3 NO safety outputs | E-stop + door to PL d |
| 1 | PLC CPU + 2x DI16 + 1x DO16 (sourcing) | 15 DI / 13 DO used + spare |
| 1 | Managed Ethernet switch | PLC, HMI, camera, robot, VFD, MES on EtherNet/IP |
| 1 | 1 hp VFD with STO input | conveyor |
