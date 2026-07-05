# Linearpositioniertisch / Colibri Kompakt 17

Arbeitsnotizen zur zweiten Achse am Mini-Teststand. Dieses README fasst zusammen,
was aus den lokalen Unterlagen, den Tests am angeschlossenen Motor und der
aktuellen GUI-Integration bekannt ist.

## Hardware

Verwendete Achse:

- Lineartisch: `MPT7512-AK-S-1`
- Hub: 75 mm
- Spindelsteigung: 2 mm/Umdrehung
- Motor/Steuerung: GSM17 / Gunda Colibri Kompakt 17 mit integrierter BAC-Positioniersteuerung
- Motoraufloesung laut Produktblatt: 400 Schritte/Umdrehung
- Daraus folgt: 200 Schritte/mm bzw. 0.005 mm/Schritt

Relevante lokale Unterlagen:

- `Colibri17_datenblatt.pdf`
- `Handbuch BAC_De V1.1 22.06.17.pdf`
- `Kurzanleitung Schnelleinstieg BAC_De 22.06.17.pdf`
- `Motorpositioniertisch MPT7512-AK-S-1 - Precision made in Germany.pdf`
- `Programming Quick Quide/Readme.txt`
- `Programming Quick Quide/Backup_Program.PRG`

## Versorgung und Stecker

Laut Colibri-Datenblatt erfolgt die Leistungsversorgung ueber den
15-poligen HD-SubD-Stecker.

Wichtige Pins am 15-poligen HD-SubD:

| Pin | Funktion |
| --- | --- |
| 1 | Motorversorgung +24 V DC bis +48 V DC |
| 2 | Steuerspannung +24 V DC bis +36 V DC |
| 3 | GND / 0 V |
| 4 | Ausgang `Bereit` / RDY |
| 5 | Ausgang `Motor steht` / MOST |
| 6 | Eingang `Start` oder `Takt` |
| 7 | Eingang E5, Richtung, Referenzpunkt oder Analogwert |

Absicherung laut Datenblatt/Handbuch:

- Steuerspannung: einzeln absichern, Beispiel 0.5 A traege
- Motorversorgung: einzeln absichern, Beispiel 2 A traege
- Die Motorspannung muss fuer jeden Colibri einzeln abgesichert werden.
- Vor Inbetriebnahme die Pinbelegung und Spannungen messen.

Beim Test wurde im BAC-CFG-Screenshot eine Motorspannung von 24 V angezeigt.
Fuer erste Tests ist 24 V daher die konservative Wahl.

## Schnittstelle / Adapter

Der Colibri besitzt je nach Variante unterschiedliche Schnittstellen. Die
Unterlagen nennen u.a. TTL/USB zur Konfiguration und RS485-BAC.

Wichtig: RS232, TTL und RS485 duerfen nicht verwechselt werden. Falsche Pegel
koennen die Steuerung beschaedigen.

Am aktuellen Teststand wurde erfolgreich gefunden:

```text
/dev/ttyUSB0
desc: DEDITEC USB-RS485-Stick - DEDITEC USB-RS485-Stick
hwid: USB VID:PID=0403:6001 SER=FT30KWBI
```

Stabiler Linux-Pfad:

```text
/dev/serial/by-id/usb-FTDI_DEDITEC_USB-RS485-Stick_FT30KWBI-if00-port0
```

Der Arduino/Teststand wurde parallel als `/dev/ttyACM0` erkannt.

## Treiber und Software

### Linux

Fuer die Python-GUI wird `pyserial` benoetigt:

```bash
python3 -m pip install pyserial
```

Im Projekt liegt auch `BiBaZu_mini_test_stand/requirements.txt`; dort steht
aktuell ebenfalls `pyserial`.

Der gefundene DEDITEC/FTDI-Adapter wird unter Linux vom Kernel-Treiber
`ftdi_sio` als `/dev/ttyUSB0` bereitgestellt. Falls der Port nicht sichtbar ist:

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Je nach Benutzerrechten muss der Benutzer Mitglied der Gruppe `dialout` sein:

```bash
groups
sudo usermod -a -G dialout $USER
```

Danach einmal ab- und wieder anmelden.

### Windows / BAC-CFG

Im Ordner liegt `setup_baccfg_9_4.exe`. Die BAC-Kurzanleitung beschreibt die
Inbetriebnahme mit BAC-CFG. Laut Handbuch installiert der BAC-CFG-Installer auch
Treiber fuer den originalen USB-TTL-Umsetzer.

Quick-Guide-Ablauf aus `Programming Quick Quide/Readme.txt`:

1. Motor ueber USB-Dongle mit PC verbinden.
2. BAC Konfigurator starten.
3. `Verbinden` klicken.
4. Wenn keine Verbindung moeglich ist: `Optionen` -> `Schnittstelle` und COM-Port anpassen.
5. Fuer manuelles Verfahren: `Diagnose`, `PC-Betrieb` aktivieren.
6. Referenzfahrt starten.
7. Absolutfahrt verwenden; im vorhandenen Setup wurden negative Positionen verwendet.

## Funktionierende serielle Parameter

Getestet und funktionierend:

```text
Baudrate: 9600
Datenbits: 8
Paritaet: none
Stopbits: 1
Slave-Adresse: 255 / 0xFF
Protokoll: binaeres BAC-Protokoll
```

Andere getestete Baudraten ohne Antwort:

- 4800
- 19200
- 38400
- 57600
- 115200

## BAC-Protokoll

Das BAC-Protokoll ist binaer, nicht ASCII.

Telegrammrahmen:

```text
Startblock: 0x04
Endblock:   0x05
SHIFT:      0x06
```

Aufbau der Rohdaten im Telegramm:

```text
Adresse, Datenlaenge, Daten..., Pruefsumme
```

Die Pruefsumme ist die 8-Bit-Summe ueber Adresse, Datenlaenge und Daten.
Bytes `0x04`, `0x05` und `0x06` werden im Telegramm mit SHIFT escaped.

Wichtige Telegrammtypen aus der GUI:

| Hex | Name | Funktion |
| --- | --- | --- |
| 0x01 | TG_REQ_STATUS | Status anfordern |
| 0x02 | TG_REQ_ERROR | letzten Fehler anfordern |
| 0x03 | TG_REQ_POSITION | aktuelle Position anfordern |
| 0x15 | TG_MOVE_REL | Relativfahrt |
| 0x16 | TG_MOTOR | Motorkommando |
| 0x1A | TG_MOVE_ABS | Absolutfahrt |

Antworttypen:

| Hex | Name | Funktion |
| --- | --- | --- |
| 0x80 | TG_STATUS | Statusantwort |
| 0x81 | TG_ERROR | Fehlerantwort |
| 0x84 | TG_POSITION | Positionsantwort |
| 0x85 | TG_MOVING | Antwort auf Positionierbefehl |

Motorkommandos:

| Wert | Name | Funktion |
| --- | --- | --- |
| 0 | ESTOP | Not-Halt ohne Rampe |
| 1 | STOP | Halt mit Rampe |
| 2 | REF | Referenzfahrt |
| 5 | REMOTE | Remote-/PC-Betrieb setzen |
| 8 | Position ist Referenzpunkt | aktuelle Position als Null/Referenzpunkt setzen |
| 10 | EXITREM | Remote-Betrieb beenden |
| 11 | DISABLE | Endstufe ausschalten |
| 12 | ENABLE | Endstufe einschalten |

## Status- und Fehlerbits

Statusbyte laut BAC-Handbuch:

| Bit | Bedeutung |
| --- | --- |
| 0 | Motor laeuft |
| 1 | Software-Endschalter |
| 3 | RDY / bereit |
| 4 | Referenzpunkt in Ordnung |
| 5 | Remote-Betrieb |
| 6 | Endstufe EIN |
| 7 | Passwort fuer Herstellerparameter |

Fehlerbyte laut BAC-Handbuch:

| Bit | Bedeutung |
| --- | --- |
| 0 | allgemeiner Fehler |
| 1 | Watchdog |
| 2 | Burn-Out / Spannungsversorgung |
| 3 | EEPROM |
| 4 | Motorspannungsfehler (`ERR_UMOT`) |
| 5 | Uebertemperatur |
| 6 | Druckmarkenfehler |
| 7 | Bootloader |

Beim Test wurde einmal `error_byte=0x11` gesehen. Das bedeutet:

- Bit 0: allgemeiner Fehler
- Bit 4: Motorspannungsfehler (`ERR_UMOT`)

Wenn dieser Zustand wieder auftaucht: Motorversorgung und Sicherung pruefen.

## GUI-Integration

Die Integration liegt in:

```text
../BiBaZu_mini_test_stand/test_run_gui.py
```

Wichtige Konstanten:

```python
COLIBRI_BAUD_RATE = 9600
COLIBRI_SLAVE_ADDRESS = 0xFF
COLIBRI_MM_PER_STEP = 0.005
COLIBRI_STEPS_PER_MM = 200.0
COLIBRI_TRAVEL_MM = 75.0
COLIBRI_REFERENCE_CURRENT_PERCENT = 20
```

Die GUI hat zwei getrennte serielle Verbindungen:

- Arduino/Teststand: normalerweise `/dev/ttyACM0`, 230400 baud, ASCII-Kommandos
- Colibri-Achse: normalerweise `/dev/ttyUSB0`, 9600 baud, BAC-Binaerprotokoll

Colibri-Funktionen in der GUI:

- eigenen Colibri-Port verbinden
- Status/Position lesen
- Endstufe ein/aus
- negative Referenzfahrt starten (`Reference -`)
- aktuelle Position als Nullpunkt setzen (`Set zero here`)
- Relativ-Jog in mm
- Absolutposition in mm anfahren
- Stop Colibri

Die GUI setzt vor Bewegungen automatisch `REMOTE` und `ENABLE`.
Der Referenzbutton setzt vor dem Start die Referenzart temporaer auf
`Drehueberwachung negativ` und den Referenzstrom auf 20 %. Die Achse soll damit
kontrolliert in die negative Endlage fahren, den Anschlag ueber die
Drehueberwachung erkennen, ein paar Freifahrschritte zurueckfahren und diesen
Punkt als Null/Referenz setzen.

Falls eine Halterung oder ein Anbauteil den mechanischen Anschlag bereits kurz
vor dem eigentlichen Achsanschlag beruehrt, ist der praktisch nutzbare Nullpunkt
dieser Kontaktpunkt. Empfohlener Ablauf:

1. Mit kleinen negativen Jogs vorsichtig bis an diesen Kontaktpunkt fahren.
2. `Set zero here` druecken.
3. Optional mit `Jog +` um 0.1 mm oder einen passenden Sicherheitsabstand
   freifahren.
4. Danach alle Arbeitspositionen positiv relativ zu diesem Nullpunkt anfahren.

## Debug-Log

In der GUI gibt es den Button `Start debug log`.

Empfohlener Ablauf beim Debuggen:

1. GUI starten.
2. `Start debug log` klicken und Datei auswaehlen.
3. Colibri verbinden.
4. Problem reproduzieren.
5. `Stop debug log` klicken.
6. Die Logdatei auswerten.

Der Log enthaelt:

- GUI-Aktionen
- Arduino-TX/RX
- Colibri-TX/RX als BAC-Hextelegramme
- Colibri-Snapshots mit Status, Position und Fehlerbits

Beispiel fuer funktionierende Status-/Positionsabfragen:

```text
COLIBRI TX 04 ff 02 01 02 05
COLIBRI RX ff 05 80 6d 70 11 72
COLIBRI TX 04 ff 02 03 06 0a 05
COLIBRI RX ff 06 84 70 1a 00 00 13
```

Die zweite Antwort enthaelt die Position als little-endian 32-Bit Integer.
`70 1a 00 00` entspricht 6768 Schritten, also 33.84 mm.

## Bekannte Beobachtungen

- Kommunikation funktioniert bei 9600 baud.
- Status- und Positionsabfrage funktionieren.
- `DISABLE` wurde getestet und funktioniert, ohne Bewegung auszuloesen.
- Der Servo/Colibri bewegt sich aus der GUI.
- Es gab Hinweise auf `ERR_UMOT` / Motorspannungsfehler. Das sollte mit dem
  Debug-Log weiter eingegrenzt werden.
- Bei einem Jog von 8.5 mm sendete die GUI korrekt 1700 Schritte. Die Achse
  stoppte jedoch deutlich vor dem Ziel und meldete `error_byte=0x11`, also
  allgemeinen Fehler plus Motorspannungsfehler. Die Motorspannung kann im
  Ruhezustand trotzdem ca. 24 V anzeigen; entscheidend ist der Einbruch unter
  Last. Netzteil, Sicherung, Klemmen, Kabel und Strombegrenzung pruefen.
- Die GUI wartet bei Jog/Absolutfahrt inzwischen bis Ziel erreicht, vorzeitig
  gestoppt, Timeout oder Fehlerbyte erkannt wird.

## Vorsicht beim Testen

- Bewegungsbefehle nur ausloesen, wenn die Achse mechanisch frei fahren kann.
- Referenzfahrt nur starten, wenn der Anschlag/Referenzmechanismus sicher ist.
- `Stop Colibri` in der GUI bereithalten.
- Versorgung und GND vor Tests messen.
- RS232, TTL und RS485 nicht mischen.
