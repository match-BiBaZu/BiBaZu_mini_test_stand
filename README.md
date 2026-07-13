# Pneumatic Test Stand Control and Data Acquisition

This repository contains the PC application and Arduino firmware for a configurable pneumatic test stand. The system controls pressure, pulses up to four valves/nozzles, records pressure, flow, and force, and positions a test part with two motion axes.

The documentation is intentionally organized around generic component roles so that sensors, valves, and mechanics can be replaced. The settings for the currently installed Colibri axis are retained in full because its wiring and BAC protocol are installation-specific.

## Repository contents

| Path | Purpose |
| --- | --- |
| `test_run_gui.py` | Tkinter GUI, serial communication, test sequencing, motion control, logging, and CSV export |
| `BiBaZu_mini_test_stand/BiBaZu_mini_test_stand.ino` | Arduino Mega 2560 firmware for the pneumatic system and stepper axis |
| `user_presets.json` | Local geometric offsets used for part positioning |
| `requirements.txt` | Python dependency list |

## Connected test stand components

The software and firmware currently expect the following components. Where the exact model is not encoded in this repository, the component is described by its electrical and functional interface rather than by an assumed part number.

| Component | Role | Connection and installed settings |
| --- | --- | --- |
| Host PC | Runs the GUI and saves logs/CSV files | Three independent serial connections are supported |
| Arduino Mega 2560 | Real-time pneumatic control, sensor acquisition, valve control, and stepper control | USB serial, `230400` baud |
| Pneumatic pressure regulator | Sets the test pressure | VEAB feedback connection is noted in the firmware; command is provided through a calibrated PWM-to-0-10 V converter; range `0-6 bar` |
| PWM-to-0-10 V converter | Converts the Arduino command into the regulator input voltage | Arduino D3, 2 kHz PWM; measured calibration curve is stored in the firmware |
| Pressure sensor before the valve | Measures supply pressure at the valve/nozzle | Arduino A0; current conversion assumes `1-5 V` represents `0-10 bar` |
| Regulator feedback signal | Measures actual regulator pressure | Arduino A2; current conversion assumes `0-5 V` represents `0-6 bar`; the VEAB black output wire is noted in the firmware |
| Flow sensor | Measures the pulse flow and integrated volume | Arduino A1; unidirectional `1-5 V`, configured for `0-200 l/min` |
| Four valves/nozzles | Produce selectable pneumatic pulses | Arduino D4-D7, active HIGH; each nozzle can be enabled separately |
| Stepper axis and driver | Generic Y-axis positioning | STEP D8, DIR D9, ENABLE D10, negative limit switch D11 |
| Negative limit switch | References the stepper axis | Arduino D11 with `INPUT_PULLUP`, active LOW |
| Colibri linear axis | Second positioning axis, used as the Z axis by the part-position calculation | MPT7512-AK-S-1 with GSM17 / Gunda Colibri Kompakt 17 and integrated BAC controller |
| DEDITEC USB-RS485 adapter | Connects the PC to the Colibri BAC interface | FTDI VID:PID `0403:6001`, serial `FT30KWBI`; BAC at `9600` baud |
| Serial force sensor/amplifier | Measures force during each impulse | Separate serial port, default `38400` baud; configured range `2 N`; accepts the supported binary frame or ASCII values |

The exact valve, pressure-sensor, flow-sensor, stepper-driver, and force-amplifier model numbers should be added here when they are confirmed from the installed hardware labels.

## Electrical connections

### Arduino Mega 2560 pin map

| Arduino pin | Signal | Direction / behavior |
| --- | --- | --- |
| A0 | Pressure before valve | Analog input; `pressure_bar = (voltage - 1.0) * 2.5` |
| A1 | Flow | Analog input; `1-5 V` maps to `0-200 l/min` |
| A2 | Regulator feedback | Analog input; `0-5 V` maps to `0-6 bar` |
| D3 / OC3C | Regulator command | 2 kHz PWM to the 0-10 V converter |
| D4 | Valve/nozzle 1 | Digital output, active HIGH |
| D5 | Valve/nozzle 2 | Digital output, active HIGH |
| D6 | Valve/nozzle 3 | Digital output, active HIGH |
| D7 | Valve/nozzle 4 | Digital output, active HIGH |
| D8 | Stepper STEP | Active LOW, 5 microsecond pulse |
| D9 | Stepper DIR | HIGH is positive; LOW is negative |
| D10 | Stepper ENABLE | HIGH enables the driver |
| D11 | Stepper negative limit | `INPUT_PULLUP`, active LOW |

The Arduino and external devices must share the required signal reference. Check the permissible voltage levels before connecting a replacement sensor or driver; the Arduino analog inputs must not be exposed directly to a 10 V signal.

### Serial connections

The GUI uses separate ports for each device:

| Device | Typical Linux port | Serial settings | Protocol |
| --- | --- | --- | --- |
| Arduino/test stand | `/dev/ttyACM0` | `230400` baud | ASCII commands and semicolon-separated samples |
| Colibri axis | `/dev/ttyUSB0` | `9600`, 8 data bits, no parity, 1 stop bit | Binary BAC protocol |
| Force sensor/amplifier | Device-dependent | Default `38400`, 8 data bits, no parity, 1 stop bit | Supported binary frames or ASCII values |

The GUI scans all serial ports and prefers labels containing `Arduino`/`ttyACM` for the controller, `DEDITEC`/`FTDI`/`RS485`/`ttyUSB` for the Colibri, and `GSV`/`ME`/`USB serial` for the force input. Always verify the selected ports before connecting.

The stable Linux path for the installed Colibri adapter is:

```text
/dev/serial/by-id/usb-FTDI_DEDITEC_USB-RS485-Stick_FT30KWBI-if00-port0
```

To inspect serial devices on Linux:

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id
ls -l /dev/ttyUSB* /dev/ttyACM*
```

If access is denied, add the user to `dialout` and then sign out and back in:

```bash
sudo usermod -a -G dialout "$USER"
```

## Installation and startup

Python 3 with Tkinter and `pyserial` is required.

```bash
python -m pip install -r requirements.txt
python test_run_gui.py
```

Upload `BiBaZu_mini_test_stand/BiBaZu_mini_test_stand.ino` to an Arduino Mega 2560. The firmware deliberately fails compilation for a different Arduino target because its 2 kHz regulator output uses Mega 2560 Timer 3 and pin D3/OC3C.

Only one program may own a serial port at a time. Close the Arduino IDE Serial Monitor, BAC-CFG, or other terminal programs before connecting from the GUI.

## Default application and test settings

### GUI defaults

| Setting | Default | Valid/implemented range |
| --- | ---: | ---: |
| Manual target pressure | `0.50 bar` | `0-6 bar` |
| Automated test start pressure | `0.50 bar` | `0-6 bar` |
| Automated test end pressure | `0.80 bar` | `0-6 bar` |
| Pressure step | `0.05 bar` | Firmware resolution is `0.05 bar` |
| Repeats per pressure | `10` | `1-100` |
| Manual increment start | `0.50 bar` | `0-6 bar` |
| Manual pressure increment | `0.05 bar` | `0-6 bar` |
| Flow-detection threshold | `2.0 l/min` | `0-200 l/min` |
| Selected nozzles | All four | Any non-empty subset |
| Live stream | Enabled | On/off |
| Stepper jog distance | `10.0 mm` | `0.01-2000 mm` |
| Stepper speed | `5.0 mm/s` | GUI `0.01-50 mm/s`; firmware command is capped at 5000 steps/s |
| Colibri relative distance | `1.0 mm` | `0.005-75 mm` |
| Colibri absolute target | `0.0 mm` | GUI `-75 to +75 mm` |
| Force-sensor baud rate | `38400` | GUI permits `1200-921600` |
| CSV format | German format enabled | Semicolon and decimal comma when enabled; comma and decimal point otherwise |

### Firmware and acquisition defaults

| Setting | Value |
| --- | ---: |
| Main sample/control interval | `5 ms` |
| Valve pulse duration | `100 ms` (`20` control ticks) |
| Flow capture minimum | `8 samples` / `40 ms` |
| End-of-flow quiet period | `3 samples` / `15 ms` below the threshold after a peak |
| GUI capture window after valve close | `500 ms` |
| Pressure samples skipped after opening | `2` / `10 ms` |
| Regulator maximum pressure | `6.0 bar` |
| Flow range | `0-200 l/min` |
| Stepper scale | `0.009985846 mm/step` |
| Stepper maximum command rate | `5000 steps/s`, approximately `49.93 mm/s` |
| Force range used for binary decoding | `2.0 N` with scale factor `1.05` |

### Installed PWM-to-0-10 V converter calibration

This measured curve is used by the Arduino firmware. It accounts for the converter's jump from 0 V to approximately 1.04 V at its smallest non-zero PWM input.

| PWM duty (%) | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Output (V) | 0.000 | 1.040 | 1.190 | 1.304 | 1.407 | 1.508 | 1.609 | 1.710 | 1.804 | 1.911 | 2.020 | 3.030 | 4.030 | 5.030 | 6.030 | 7.020 | 7.990 | 9.000 | 9.860 | 10.000 |

## Operating the test stand

1. Verify pneumatic, electrical, sensor, and mechanical connections.
2. Start the GUI and select the Arduino port.
3. Connect the Arduino. The GUI applies the pressure and flow-threshold settings after connecting.
4. Connect and software-zero the force sensor if force data is required.
5. Connect and reference the required motion axes.
6. Select one or more nozzles.
7. Use a manual pulse at low pressure to validate the setup, or configure the pressure range and repeat count and start an automated test.
8. Stop the test immediately if pressure, flow, force, or motion is unexpected.
9. Save the summarized impulse CSV and the automatically generated `_raw.csv` sample file.

The GUI also supports manual pressure application, pressure-increment pulses, stepper and Colibri jog/absolute motion, part-based axis targets, continuous live data, and a detailed debug log.

## Arduino command interface

The GUI sends these ASCII command families to the Arduino:

| Command | Purpose |
| --- | --- |
| `START:<start>:<end>:<repeats>:<mask>` | Run an automated pressure sweep |
| `STOP` | Stop the test and close the valves |
| `STREAM_ON` / `STREAM_OFF` | Enable or disable continuous samples |
| `SET_PRESSURE:<bar>` | Set the regulator target |
| `SET_FLOW_THRESHOLD:<l/min>` | Set pulse-flow detection threshold |
| `PULSE:<mask>` | Trigger a manual pulse |
| `MOTOR_ENABLE:<0|1>` | Disable or enable the stepper driver |
| `MOTOR_SPEED:<steps/s>` | Set stepper speed |
| `MOTOR_MOVE:<steps>` | Relative stepper move |
| `MOTOR_ABS:<steps>` | Absolute stepper move |
| `MOTOR_HOME` | Reference toward the negative limit switch |
| `MOTOR_ZERO` | Set the current stepper position to zero |
| `MOTOR_POS` | Request the stepper position |
| `MOTOR_STOP` | Stop stepper motion |

The four low bits of the valve mask correspond to nozzles 1-4.

## Part-position CSV and geometric presets

Part-position input files must use English CSV formatting. Uncheck **German CSV format** before loading them. Required columns are:

```text
Pose,Hole,Y-offset,Z-offset,Y-CapOffset,Z-CapOffset
```

The optional cap offsets replace the normal Y/Z offsets when **Use cap offsets** is enabled.

Axis targets are calculated as:

```text
stepper target = nozzle offset + selected Y offset
Colibri target = test stand height - holder height + selected Z offset
```

The current local values in `user_presets.json` are:

| Preset | Current value |
| --- | ---: |
| Nozzle offset | `125.41 mm` |
| Test stand height | `150.00 mm` |
| Holder height | `43.53 mm` |

These values describe the current fixture geometry, not universal test-stand constants. The GUI can save changed values back to `user_presets.json` when it closes.

## CSV output

**Save CSV** writes two files:

- A summarized impulse file containing timing, selected pose/hole, read-back axis offsets, target and measured pressure, average force, maximum flow, integrated volume, nozzle selection, and sample counts.
- A `_raw.csv` file containing time-series pressure, regulator command, valve state, flow, and force.

With **German CSV format** enabled, output uses semicolons and decimal commas. With it disabled, output uses commas and decimal points.

## Debug logging

Use **Start debug log** before reproducing a problem. The log records:

- GUI actions and settings
- Arduino TX/RX
- force-sensor connection and zeroing events
- Colibri BAC TX/RX as hexadecimal frames
- Colibri position, status, and error snapshots

Recommended procedure:

1. Start the debug log and select a file.
2. Connect the required devices.
3. Reproduce the problem.
4. Stop the debug log.
5. Review the recorded commands, samples, and status transitions.

## Installed Colibri axis configuration

This section documents the current MPT7512/Colibri installation. Keep these settings unless the physical axis, controller configuration, or adapter is changed.

### Axis and controller

- Linear stage: `MPT7512-AK-S-1`
- Travel: `75 mm`
- Lead screw pitch: `2 mm/revolution`
- Motor/controller: GSM17 / Gunda Colibri Kompakt 17 with integrated BAC positioning controller
- Motor resolution from the product data: `400 steps/revolution`
- Resulting scale: `200 steps/mm` or `0.005 mm/step`

Relevant source documents used during commissioning were:

- `Colibri17_datenblatt.pdf`
- `Handbuch BAC_De V1.1 22.06.17.pdf`
- `Kurzanleitung Schnelleinstieg BAC_De 22.06.17.pdf`
- `Motorpositioniertisch MPT7512-AK-S-1 - Precision made in Germany.pdf`
- `Programming Quick Quide/Readme.txt`
- `Programming Quick Quide/Backup_Program.PRG`

These documents are commissioning references and are not currently stored in this repository.

### Supply and 15-pin connector

According to the Colibri data sheet, power is supplied through the 15-pin high-density D-sub connector.

| Pin | Function |
| --- | --- |
| 1 | Motor supply, +24 to +48 V DC |
| 2 | Control supply, +24 to +36 V DC |
| 3 | GND / 0 V |
| 4 | `Ready` / RDY output |
| 5 | `Motor stopped` / MOST output |
| 6 | `Start` or clock input |
| 7 | E5, direction, reference-point, or analog input |

Supply precautions from the data sheet/manual:

- Fuse the control supply separately; the documentation gives `0.5 A slow-blow` as an example.
- Fuse the motor supply separately; the documentation gives `2 A slow-blow` as an example.
- Each Colibri motor supply must have its own fuse.
- Measure pin assignments and voltages before commissioning.

BAC-CFG displayed a motor voltage of 24 V during testing. Therefore 24 V is the conservative starting voltage for this installed setup.

### Interface and adapter

Colibri variants can use different electrical interfaces. The documentation mentions TTL/USB for configuration and RS485-BAC. Do not interchange RS232, TTL, and RS485; incorrect voltage levels can damage the controller.

The adapter detected successfully during commissioning was:

```text
/dev/ttyUSB0
description: DEDITEC USB-RS485-Stick
hardware ID: USB VID:PID=0403:6001 SER=FT30KWBI
```

On Linux, the FTDI adapter is handled by `ftdi_sio`. On Windows, BAC-CFG can be used for commissioning. The BAC-CFG installer described in the quick-start guide also installs the driver for the original USB-TTL adapter.

BAC-CFG commissioning sequence:

1. Connect the motor/controller through the appropriate USB adapter.
2. Start BAC Configurator.
3. Select **Connect**.
4. If it cannot connect, select the correct COM port under **Options > Interface**.
5. For manual movement, open **Diagnostics** and enable PC operation.
6. Start a reference run.
7. Use absolute positioning; the original setup used negative positions during commissioning.

### Serial and motion constants

```text
Baud rate:          9600
Data bits:          8
Parity:             none
Stop bits:          1
Slave address:      255 / 0xFF
Protocol:           binary BAC
Scale:              0.005 mm/step
Travel:             75 mm
Reference current:  20%
```

The tested baud rates `4800`, `19200`, `38400`, `57600`, and `115200` produced no response. The installed controller responded at `9600` baud.

### BAC protocol

BAC is a binary protocol, not an ASCII protocol.

```text
Start block: 0x04
End block:   0x05
SHIFT:       0x06
```

The unescaped payload is:

```text
address, data length, data..., checksum
```

The checksum is the 8-bit sum of address, data length, and data. Bytes `0x04`, `0x05`, and `0x06` are escaped with SHIFT inside a frame.

Request/command telegrams used by the GUI:

| Hex | Name | Purpose |
| --- | --- | --- |
| `0x01` | `TG_REQ_STATUS` | Request status |
| `0x02` | `TG_REQ_ERROR` | Request the last error |
| `0x03` | `TG_REQ_POSITION` | Request current position |
| `0x06` | `TG_REQ_PARAM` | Read a parameter |
| `0x15` | `TG_MOVE_REL` | Relative move |
| `0x16` | `TG_MOTOR` | Motor command |
| `0x1A` | `TG_MOVE_ABS` | Absolute move |
| `0x1F` | `TG_SET_PARAM` | Write a parameter |

Response telegrams:

| Hex | Name | Purpose |
| --- | --- | --- |
| `0x80` | `TG_STATUS` | Status response |
| `0x81` | `TG_ERROR` | Error response |
| `0x83` | `TG_PARAM` | Parameter response |
| `0x84` | `TG_POSITION` | Position response |
| `0x85` | `TG_MOVING` | Positioning-command response |

Motor command values:

| Value | Name | Purpose |
| ---: | --- | --- |
| 0 | `ESTOP` | Immediate stop without ramp |
| 1 | `STOP` | Stop with ramp |
| 2 | `REF` | Reference run |
| 5 | `REMOTE` | Enable remote/PC operation |
| 8 | Set reference point | Set the current position as zero/reference |
| 10 | `EXITREM` | Leave remote operation |
| 11 | `DISABLE` | Disable the output stage |
| 12 | `ENABLE` | Enable the output stage |

### Status and error bits

Status byte:

| Bit | Meaning |
| ---: | --- |
| 0 | Motor running |
| 1 | Software limit switch |
| 3 | RDY / ready |
| 4 | Reference point valid |
| 5 | Remote operation |
| 6 | Output stage enabled |
| 7 | Manufacturer-parameter password active |

Error byte:

| Bit | Meaning |
| ---: | --- |
| 0 | General error |
| 1 | Watchdog |
| 2 | Burn-out / supply error |
| 3 | EEPROM |
| 4 | Motor-voltage error (`ERR_UMOT`) |
| 5 | Overtemperature |
| 6 | Print-mark error |
| 7 | Bootloader |

An `error_byte` of `0x11` was observed during testing. It combines bit 0 (general error) and bit 4 (`ERR_UMOT`). If this recurs, check the motor supply, fuse, power-supply current limit, terminals, and cable. A nominal 24 V reading at rest does not exclude a voltage drop under load.

### GUI behavior and referencing

The GUI can:

- Connect the Colibri through its own serial port
- Read status, position, and error state
- Enable or disable the output stage
- Start a negative reference run
- Set the current position as zero
- Jog by a relative distance in millimeters
- Move to an absolute position in millimeters
- Stop the Colibri

Before movement, the GUI automatically sends `REMOTE` and `ENABLE`. Before a negative reference run, it temporarily sets reference type parameter `4:1` to value `2` (negative rotational monitoring) and reference-current parameter `5:2` to `20%`. The axis should move toward its negative end, detect the stop through rotational monitoring, back away by its configured release steps, and establish the reference point.

If a fixture touches before the stage reaches its own mechanical stop, treat the fixture contact as the practical zero:

1. Approach it with small negative jogs.
2. Select **Set zero here**.
3. Optionally jog positive by `0.1 mm` or another safe clearance.
4. Use positive working positions relative to that zero.

Example working status/position exchange:

```text
COLIBRI TX 04 ff 02 01 02 05
COLIBRI RX ff 05 80 6d 70 11 72
COLIBRI TX 04 ff 02 03 06 0a 05
COLIBRI RX ff 06 84 70 1a 00 00 13
```

The position is a signed little-endian 32-bit integer. `70 1a 00 00` is 6768 steps, or `33.84 mm` at `0.005 mm/step`.

### Known Colibri observations

- Communication, status, and position queries work at `9600` baud.
- `DISABLE` was tested and did not cause movement.
- The Colibri axis moves from the GUI.
- A commanded `8.5 mm` jog correctly transmitted 1700 steps, but the axis stopped early with `error_byte=0x11`.
- The GUI waits for a jog or absolute move to reach its target, stop early, time out, or report an error byte.
- Continue using the debug log to investigate any recurrence of `ERR_UMOT`.

## Safety

- Verify all supply voltages, grounds, fuses, and signal levels before energizing the stand.
- Start pneumatic tests at low pressure and with a safe exhaust path.
- Trigger valves only when the nozzles and test part are safely contained.
- Keep clear of both axes and make sure the full commanded path is mechanically free.
- Reference an axis only when its stop/reference mechanism is safe to contact.
- Keep **Stop**, **Stop motor**, and **Stop Colibri** accessible.
- Do not rely on the GUI as the only emergency-stop or overpressure protection.
- Do not interchange RS232, TTL, and RS485 connections.
- Investigate any Colibri motor-voltage error before continuing motion tests.
