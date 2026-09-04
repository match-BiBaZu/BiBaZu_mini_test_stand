# BiBaZu Mini Test Stand

The pneumatic and stepper-control path now runs on the Beckhoff C9020 through
TwinCAT. The GUI is an ADS client; it does not use the Arduino and does not
open the EtherCAT adapter. TwinCAT is the sole EtherCAT master.

The Colibri USB/RS485 axis and TAL221/QuantumX force system are unchanged. The
old Arduino sketch remains under `BiBaZu_mini_test_stand/` as a behavior
reference only.

## Existing I/O mapping

| PLC variable | Terminal link |
| --- | --- |
| `MAIN.bReadLimitSwitch` | EL1014 terminal 12, channel 1 input |
| `MAIN.iReadNozzlePressure` | EL3164 terminal 15, channel 1 value |
| `MAIN.iReadNozzleFlow` | EL3164 terminal 15, channel 2 value |
| `MAIN.iSetRegulatorPressure` | EL4102 terminal 14, channel 1 output |
| `MAIN.bSetNozzleOn1` through `bSetNozzleOn4` | EL2004 terminal 16, channels 1 through 4 |
| `MAIN.bSetNozzleOn5` through `bSetNozzleOn6` | EL2004 terminal 17, channels 1 through 2 |
| `MAIN.Axis1` | NC axis `Achse 1`, EL7062 channel 1 |

`MAIN.Axis1.PlcToNc` and `MAIN.Axis1.NcToPlc` are linked to `Achse 1`.
The PLC task is 5 ms, matching the former Arduino control interval. Verify
C9020 real-time load before activating this rate.

## PLC/ADS contract

`MAIN.TcPOU` owns all physical I/O. PyADS must write only the public command
mailbox, never physical outputs or `Axis1` directly. The client writes a
payload, `nCmdType`, and finally `udCmdSeq`; it waits for the matching
`udCmdAckSeq` before submitting another transaction. A skipped sequence is a
PLC fault that inhibits outputs.

| Command | Action |
| ---: | --- |
| 1 | Set pressure target |
| 2 | Pulse selected nozzles |
| 3 | Start automatic pressure sweep |
| 4 | Stop all outputs |
| 5 | Set motor power state |
| 6 / 7 | Relative / absolute motor move |
| 8 / 9 | Home / set motor zero |
| 10 / 11 | Motor stop / reset |
| 12 | Apply pulse duration and flow threshold |

The watchdog requires a changing `udCmdHeartbeat` every two seconds. Before
the first heartbeat, and after a timeout, the PLC closes all valves, commands
zero volts from the EL4102, and removes motor power. Connecting the GUI alone
does not apply pressure.

The PLC calculates flow peak, sample count, duration, and volume at its 5 ms
rate. GUI ADS telemetry is best-effort: its pulse-start notification and
force/pressure trace times are host receipt times, not PLC-edge timestamps.
Use the PLC flow summary as authoritative and validate force timing/jitter if
you need time-critical measurements.

## VPPM and analog inputs

The regulator is `VPPM-8L-1-G14-0L6H-V1N-S1`: 0-6 bar with a 0-10 V command.

| Festo cable | Function | Connection |
| --- | --- | --- |
| Yellow, pin 4 | `W+`, 0-10 V command | EL4102 terminal 1, channel 1 output |
| Green, pin 3 | `W-`, signal reference | EL4102 terminal 3 or 7, channel 1 GND |
| Brown, pin 2 | +24 V supply | 24 V supply |
| Blue, pin 7 | 0 V supply | Supply 0 V |
| Pink, pin 6 | Optional actual value | EL3164 channel 3 only if intentionally wired and mapped |

EL4102 terminal 5 is channel 2 output, not channel 1 ground. Its direct
0-10 V process image is `0..32767`; for a direct 0-6 bar VPPM command:

```text
raw = round(target_bar / 6.0 * 32767)
```

The old Arduino PWM-to-10 V calibration is disabled. Enable the PLC
gain/offset correction only after a new direct-EL4102 pressure calibration.

The default EL3164 scaling preserves the old 1-5 V assumptions:

```text
voltage = raw * 10 / 32767
pressure_bar = clamp((voltage - 1) * 2.5, 0, 10)
flow_nl_min = clamp((voltage - 1) * 50, 0, 200)
```

Verify the voltage range printed on each sensor before energizing the system.
VPPM feedback is deliberately reported as unavailable until the pink wire is
mapped to an actual analog-input channel.

## GUI setup

This repository uses `requirements.txt`, not a `pyproject.toml`. Install its
dependencies into the existing virtual environment and run the GUI:

```powershell
uv pip install --python .\.venv\Scripts\python.exe -r .\requirements.txt
.\.venv\Scripts\python.exe .\test_run_gui.py
```

In **Connection settings**, enter the live C9020 AMS Net ID, reachable C9020
IP address, and PLC ADS port (normally `851`). The stored Net ID
`5.75.145.248.1.1` is only a project value; verify it online. Configure ADS
routes in both directions and ensure `TcAdsDll.dll` is available on the GUI PC.

## Six-nozzle masks and motion

Bits 0 through 5 select nozzles 1 through 6. For example, mask `33` selects
nozzles 1 and 6. The GUI and CSV output contain six nozzle columns.

**Play Imperial March** uses the currently selected nozzle mask and repeats a
short pneumatic melody until the same button is pressed again. Each note waits
for the PLC pulse and flow capture to finish, so the GUI stays responsive and
the PLC never receives overlapping pulses. Stopping sends the fail-safe
`STOP` command, which closes the valves, resets pneumatic output pressure, and
removes motor power.
The empirical note widths can be tuned in `NOZZLE_MARCH_PULSE_MS` in
`test_run_gui.py`; all values must remain within `10..500 ms` and on the PLC's
5 ms task grid.

The EL7062 is driven through NC axis `Achse 1`, not an Arduino STEP/DIR loop.
`bConfigMotionEnabled` and `bConfigHomingEnabled` default to `FALSE` on
purpose. Before enabling either, configure the EL7062 motor current and
scaling, NC soft limits, home behavior, and EL1014 pressed-state polarity. The
PLC requires a referenced axis for travel moves and rejects a negative jog
when the home switch is already active. The EL1014 input is not a substitute
for an independent safety circuit or E-stop.

DI1/home is `0 mm`. The physical command convention is Home/Jog left =
negative and Jog right = positive. The current axis feedback has the opposite
coordinate sign: extending right into the working area reports negative
positions. The PLC placeholder position range is therefore `-2000..0 mm`;
replace `-2000 mm` with the measured physical travel limit during commissioning.

The mechanical center is stored as an absolute Stepper setting (currently
`-53 mm`) and is not edited in the normal motion row. `Center offset` is the
operator input used by **Move center**: `0 mm` moves to the stored center,
negative offsets move left, and positive offsets move right. For example,
`-3 mm` moves to `-50 mm`, three millimetres left of center.

### EL7062 channel 1 CoE settings

For the two-phase NEMA 17 on EL7062 channel 1, set these direct CoE entries
only after confirming the motor data sheet states a 1.5 A phase current:

| CoE entry | Value | Purpose |
| --- | ---: | --- |
| `0x8010:64` Commutation type | `16` | Stepper with internal counter; no encoder |
| `0x8008:12` Encoder type | `0` | Disabled when no external encoder is installed |
| `0x8011:12` Rated current | `1500` mA | Motor data-sheet current |
| `0x8011:33` Motor fullsteps per revolution | `200` | 1.8 degree motor |
| `0x8011:34` Configured motor current | `1500` mA | Active current limit |
| `0x8010:72` Stand still torque limitation | Commission after a holding-torque test | Optional current reduction at standstill |

`1.9971692 mm/rev` is a feed constant, not a separate `0x8011` motor field.
The checked-in axis currently maps the standard FB/DRV position PDOs, not the
DMC PDOs. Do not use the DMC `feed / 2^32` formula unless the PDO mode is also
changed to DMC. With the standard 20-bit single-turn feedback, the provisional
factor would be `1.9971692 / 2^20`, approximately
`1.90464897155762e-6 mm/increment`. Confirm the configured single-turn bits and
measure one physical motor revolution before activating that value; the
checked-in `1e-5` axis value is not yet a commissioned physical scale.

## Commissioning checklist

1. Build the TwinCAT PLC, regenerate symbols, activate/download it, and verify
   `MAIN.bStatusReady` through ADS port 851.
2. Confirm that all six valves are false and EL4102 raw output is zero at PLC
   start and before the first GUI heartbeat.
3. Verify watchdog/disconnect behavior before applying air pressure.
4. Meter EL4102 values 0, about 16384, and 32767 for 0, about 5, and 10 V.
5. Check EL3164 raw/scaled values against known sensor voltages.
6. At low pressure, test every valve and a multi-nozzle mask.
7. Verify 10, 50, and 500 ms pulses plus the PLC flow summary.
8. At low speed, press Home and verify that the axis jogs left (negative
   machine direction), DI1 stops it, and the stopped position becomes 0 mm.
   Verify that extending to the right afterwards produces negative positions. Repeat while
   pressing Stop motor before DI1; the axis must stop without setting a new
   zero.
9. Run and abort a short sweep; confirm every output becomes safe.
10. Recheck Colibri and QuantumX force data.

## Unchanged Colibri and force paths

- Colibri BAC remains USB/RS485 at 9600 baud, slave `0xFF`.
- QuantumX remains MX440B `192.168.10.20`, TAL221 A channel 3 and TAL221 B
  channel 4, through the local bridge `127.0.0.1:5500`.
- Force calibration, timestamps, and total-force calculation remain in
  `force_sources.py` and the existing calibration files.

Different network ports are fine when Windows routes the C9020 and MX440B
networks independently. The QuantumX bridge is loopback-only, so the GUI and
bridge must run on the same host unless that setup is deliberately changed.
See `AGENTS_MX440B_KD24s.md` for force-system details.
