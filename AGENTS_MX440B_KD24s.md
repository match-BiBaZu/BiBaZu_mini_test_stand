# AGENTS.md — HBM QuantumX MX440B mit zwei KD24s-Kraftsensoren

## Ziel

Richte einen HBM QuantumX MX440B unter Windows so ein, dass zwei ME-Meßsysteme KD24s 2 N auf getrennten Kanälen gemessen, separat kalibriert und anschließend ohne catman/MX Assistant über Ethernet ausgelesen werden können.

Das Endergebnis soll sein:

1. Beide Sensoren werden auf getrennten MX440B-Kanälen erfasst.
2. Jeder Kanal liefert einen korrekt skalierten Kraftwert in Newton.
3. Die Summe `F_total = F_1 + F_2` ist unabhängig von der Lastposition auf der Plattform.
4. Eine eigene Anwendung liest die Daten über die HBM Common API aus.
5. Optional stellt ein kleiner Windows-Dienst die Messwerte per TCP, UDP oder ROS 2 für einen Ubuntu-Rechner bereit.
6. Alle Einstellungen, Tests und offenen Punkte werden reproduzierbar dokumentiert.

---

## Hardware

### Messverstärker

- HBM QuantumX MX440B
- Vier getrennte Sensorkanäle
- Verbindung zum PC über Ethernet
- Konfiguration zunächst unter Windows mit MX Assistant oder catman

### Sensor 1

- Typ: ME-Meßsysteme KD24s 2 N
- Seriennummer: 25200802
- Nennlast: 2 N
- Kennwert: 0.48455 mV/V bei 2 N
- Nullsignal laut Prüfprotokoll: -0.00122 mV/V
- Eingangswiderstand: 705.4 Ohm

### Sensor 2

- Typ: ME-Meßsysteme KD24s 2 N
- Seriennummer: 26202259
- Nennlast: 2 N
- Kennwert: 0.93138 mV/V bei 2 N
- Nullsignal laut Prüfprotokoll: -0.00998 mV/V
- Eingangswiderstand: 389.6 Ohm

Die Sensoren dürfen nicht elektrisch parallel geschaltet werden. Ihre Kennwerte unterscheiden sich ungefähr um den Faktor 1.92. Jeder Sensor benötigt daher einen eigenen Messkanal und eine eigene Skalierung.

---

## Sensorbelegung

Für beide KD24s gilt:

| Ader | Funktion |
|---|---|
| Rot | Speisung positiv, Us+ |
| Schwarz | Speisung negativ, Us- |
| Grün | Messsignal positiv, Ud+ |
| Weiß | Messsignal negativ, Ud- |
| Transparent | Schirm |

Die Sensoren sind als DMS-Vollbrücke anzuschließen.

Vor dem Einschalten mit einem Multimeter prüfen:

- Kein Kurzschluss zwischen Speisung+ und Speisung-
- Kein Kurzschluss zwischen Signal+ und Signal-
- Schirm nicht mit einer Signalleitung verbunden
- Die vorgeschriebenen Sense- und TEDS-/Erkennungsbrücken im SUB-HD-15-Stecker sind korrekt gesetzt
- Die Pinbelegung wird anhand des offiziellen MX440B-Handbuchs und der eingeprägten Pinnummern verifiziert
- Nicht allein nach der optischen Orientierung des Steckers verdrahten, da Kontakt- und Lötseite spiegelverkehrt erscheinen können

---

## Sicherheits- und Messgrenzen

- Kein einzelner KD24s darf dauerhaft über 2 N belastet werden.
- Auch bei einer Plattform mit theoretisch 4 N Gesamtbereich kann eine Randlast nahezu vollständig auf nur einem Sensor liegen.
- Vor jeder Belastung beide Kanäle auf Übersteuerung prüfen.
- Zunächst mit kleinen Prüfkräften beginnen.
- Eine mechanische Überlastung lässt sich nicht durch den Messbereich des Verstärkers verhindern.

---

## Phase 1 — Windows-PC vorbereiten

Installiere ausschließlich offizielle HBK-Komponenten:

1. QuantumX / SomatXR System Package
2. MX Assistant
3. HBM Device Manager
4. HBM Common API
5. Optional catman Easy/AP für Vergleichsmessungen

Offizielle Downloadseite:

https://www.hbkworld.com/en/products/support-resources/support-hbm/downloads/downloads-software/support-downloads-quantumx-somatxr

Hinweise:

- Das aktuelle System Package enthält MX Assistant, HBM Device Manager und Dokumentation.
- Die HBM Common API enthält Dokumentation und C#-Beispiele.
- Für neue Projekte die HBM Common API verwenden.
- Die ältere QuantumX API gilt als veraltet und wird nicht mehr offiziell unterstützt.

Während der Installation Versionen dokumentieren:

```text
Windows-Version:
MX Assistant-Version:
HBM Device Manager-Version:
HBM Common API-Version:
MX440B-Firmware:
MX440B-Seriennummer:
```

Keine Firmware aktualisieren, bevor:

1. die bestehende Firmwareversion dokumentiert wurde,
2. die Kommunikation funktioniert,
3. die passende Firmware ausdrücklich für B-Module freigegeben ist,
4. ein tatsächlicher Grund für das Update besteht.

---

## Phase 2 — Netzwerkverbindung herstellen

1. MX440B direkt oder über einen Switch mit dem Windows-PC verbinden.
2. HBM Device Manager starten.
3. Gerät suchen.
4. Aktuelle IP-Adresse, Subnetzmaske und Firmware dokumentieren.
5. Falls nötig, eine statische IP im gleichen Subnetz vergeben.
6. Erreichbarkeit mit `ping` prüfen.
7. Prüfen, ob MX Assistant das Gerät öffnen kann.

Beispiel für ein isoliertes Messnetz:

```text
MX440B:      192.168.10.20
Windows-PC:  192.168.10.10
Netzmaske:   255.255.255.0
Gateway:     leer
```

Keine IP-Adresse blind übernehmen. Zuerst vorhandene Netzwerkeinstellungen prüfen.

---

## Phase 3 — Kanäle konfigurieren

Sensor 1 an Kanal 1 und Sensor 2 an Kanal 2 anschließen.

Für jeden Kanal getrennt konfigurieren:

- Sensortyp: DMS-Vollbrücke / Full Bridge
- Anschlussart: 4-Leiter-Vollbrücke, sofern die Sense-Leitungen im Stecker lokal gebrückt sind
- Brückenspeisung: zunächst 5 V, sofern vom MX440B und Sensoranschluss unterstützt
- Messbereich: mindestens ±1 mV/V
- Abtastrate: zunächst 100 Hz
- Filter: zunächst niedriger Tiefpass, z. B. 10 bis 20 Hz
- Einheit: N
- Nullabgleich unbelastet
- Keine gemeinsame Skalierung beider Sensoren verwenden

### Sensor 1

```text
Nennkraft: 2 N
Kennwert: 0.48455 mV/V
```

### Sensor 2

```text
Nennkraft: 2 N
Kennwert: 0.93138 mV/V
```

Falls MX Assistant eine Zweipunktkalibrierung verlangt:

1. Unbelasteten Messwert aufnehmen.
2. Definierte Prüfkraft aufbringen.
3. Tatsächliche Kraft in Newton eingeben.
4. Skalierung speichern.
5. Last entfernen und Nullrückkehr prüfen.

Bei einem Prüfgewicht gilt:

```text
F = m * g
g = 9.80665 m/s²
```

Beispiele:

```text
100 g = 0.980665 N
200 g = 1.96133 N
```

Die Prüfkraft darf den Sensor nicht überlasten.

---

## Phase 4 — Einzelkanäle validieren

Jeden Sensor zunächst unabhängig testen.

Für jeden Kanal mindestens folgende Punkte aufnehmen:

- unbelastet
- ca. 0.5 N
- ca. 1.0 N
- ca. 1.5 N
- maximal etwa 1.9 N

Für jeden Punkt notieren:

```text
Sollkraft:
Istkraft:
Abweichung:
Rohsignal in mV/V:
Streuung über 10 s:
Nullwert nach Entlastung:
```

Akzeptanzkriterien zunächst:

- monotone Kennlinie
- kein Clipping oder Overrange
- plausible Empfindlichkeit
- Nullwert kehrt nach Entlastung reproduzierbar zurück
- keine sprunghaften Kabel- oder Kontaktfehler
- Vorzeichen beider Sensoren gleich

Stimmt das Vorzeichen nicht, nicht sofort umverdrahten. Zuerst prüfen, ob das Vorzeichen in der Software invertiert werden kann. Die dokumentierte Aderbelegung muss konsistent bleiben.

---

## Phase 5 — Plattformtest

Nach erfolgreicher Einzelprüfung beide Sensoren mechanisch unter der Plattform montieren.

Berechneten Kanal anlegen:

```text
F_total = F_1 + F_2
```

Zusätzlich optional:

```text
load_fraction_1 = F_1 / F_total
load_fraction_2 = F_2 / F_total
```

Nur berechnen, wenn `abs(F_total)` oberhalb einer kleinen Schwelle liegt, um Division durch Werte nahe null zu vermeiden.

Testpositionen:

1. mittig
2. direkt über Sensor 1
3. direkt über Sensor 2
4. mehrere Zwischenpositionen

Für jede Position dieselbe bekannte Kraft verwenden.

Akzeptanzkriterium:

- `F_total` bleibt innerhalb der gewünschten Toleranz unabhängig von der Position.
- `F_1` und `F_2` dürfen sich positionsabhängig ändern.
- Kein einzelner Sensor überschreitet 2 N.

Falls `F_total` positionsabhängig ist:

1. mechanische Kraftnebenschlüsse prüfen,
2. Plattform auf Verkippung prüfen,
3. Sensorbefestigung auf Verspannung prüfen,
4. Kabelkräfte ausschließen,
5. beide Einzelkalibrierungen wiederholen.

---

## Phase 6 — HBM Common API testen

Zunächst auf dem Windows-PC ein minimales C#-Programm erstellen.

Ziele des ersten Programms:

1. QuantumX-Geräte im Netzwerk erkennen oder über bekannte IP verbinden.
2. MX440B identifizieren.
3. Kanalmetadaten auslesen.
4. Messwerte von Kanal 1 und Kanal 2 streamen.
5. Zeitstempel und Einheit ausgeben.
6. Verbindung sauber schließen.
7. Verständliche Fehlermeldungen liefern.

Die API-Aufrufe dürfen nicht aus dem Gedächtnis erfunden werden. Die lokal installierte HBM-Common-API-Dokumentation und die mitgelieferten C#-Beispiele sind die maßgebliche Quelle.

Suche im installierten SDK insbesondere nach Beispielen zu:

```text
QuantumX
device discovery
connect
measurement
streaming
channel
signal
subscribe
sample rate
Common API
```

Erstelle anschließend diese Projektstruktur:

```text
quantumx_bridge/
├── AGENTS.md
├── README.md
├── docs/
│   ├── hardware_configuration.md
│   ├── mx440b_settings.md
│   ├── calibration_results.csv
│   └── troubleshooting.md
├── src/
│   └── QuantumXBridge/
│       ├── QuantumXBridge.csproj
│       ├── Program.cs
│       ├── DeviceDiscovery.cs
│       ├── AcquisitionService.cs
│       ├── SensorConfig.cs
│       └── OutputServer.cs
├── config/
│   └── quantumx.yaml
└── tests/
```

Beispielkonfiguration:

```yaml
device:
  ip: "192.168.10.20"
  expected_model: "MX440B"

acquisition:
  sample_rate_hz: 100
  output_rate_hz: 100

channels:
  - name: force_1
    module_channel: 1
    serial_number: "25200802"
    nominal_force_n: 2.0
    characteristic_mv_v: 0.48455

  - name: force_2
    module_channel: 2
    serial_number: "26202259"
    nominal_force_n: 2.0
    characteristic_mv_v: 0.93138

derived:
  total_force: "force_1 + force_2"

network_output:
  mode: "tcp"
  bind_address: "0.0.0.0"
  port: 5500
```

---

## Phase 7 — Ethernet-Bridge für Ubuntu

Da die HBM Common API voraussichtlich auf Windows ausgeführt wird, soll ein kleiner Windows-Dienst Messwerte netzwerktransparent bereitstellen.

Priorität:

1. TCP mit newline-delimited JSON für robuste erste Tests
2. UDP nur bei notwendiger geringer Latenz
3. ROS 2 nur nach erfolgreichem Basistest

Empfohlenes Nachrichtenformat:

```json
{"timestamp_ns": 0, "force_1_n": 0.0, "force_2_n": 0.0, "force_total_n": 0.0, "status": "ok"}
```

Anforderungen:

- UTF-8
- eine JSON-Nachricht pro Zeile
- monotone Zeitstempel oder klar dokumentierte Gerätezeit
- automatische Wiederverbindung zum MX440B
- Client-Verbindungsabbrüche dürfen die Messung nicht beenden
- Statusmeldungen bei Overrange, Verbindungsverlust oder ungültigem Kanal
- keine stillschweigende Ausgabe von Nullwerten bei Kommunikationsfehlern

---

## Phase 8 — Ubuntu-Client

Unter Ubuntu einen Python-Client erstellen, der:

1. TCP-Verbindung zum Windows-PC aufbaut,
2. JSON-Zeilen liest,
3. Werte validiert,
4. Messausfälle erkennt,
5. optional CSV schreibt,
6. optional ROS-2-Nachrichten publiziert.

Empfohlene ROS-2-Topics:

```text
/force_platform/force_1
/force_platform/force_2
/force_platform/force_total
/force_platform/status
```

Empfohlene Nachrichtentypen:

```text
std_msgs/msg/Float64
diagnostic_msgs/msg/DiagnosticArray
```

Später kann ein eigener Nachrichtentyp mit gemeinsamem Zeitstempel verwendet werden.

---

## Arbeitsregeln für den Codex-Agenten

- Keine API-Klassen, Methoden oder Namespaces erfinden.
- Zuerst lokale SDK-Dokumentation und offizielle Beispiele untersuchen.
- Vor Änderungen vorhandene Konfiguration und Firmware dokumentieren.
- Kleine, überprüfbare Schritte durchführen.
- Nach jedem Schritt einen Test ausführen.
- Keine Firmware aktualisieren, solange es nicht erforderlich ist.
- Keine Sensorwerte stillschweigend skalieren.
- Rohwert, skalierten Einzelwert und Summenwert getrennt halten.
- Sensorseriennummern fest den Kanälen zuordnen.
- Fehlerzustände explizit melden.
- Geheimnisse, Lizenzschlüssel und lokale Zugangsdaten nicht ins Repository schreiben.
- Jede funktionierende Konfiguration in `docs/mx440b_settings.md` dokumentieren.
- Jede Messreihe mit Datum, Last, Position und verwendeter Konfiguration dokumentieren.
- Bei Unklarheiten Screenshots oder konkrete Fehlermeldungen anfordern, statt zu raten.

---

## Definition of Done

Das Projekt ist abgeschlossen, wenn:

- [ ] MX440B wird unter Windows zuverlässig erkannt.
- [ ] Kanal 1 misst Sensor 25200802 korrekt.
- [ ] Kanal 2 misst Sensor 26202259 korrekt.
- [ ] Beide Kanäle sind einzeln kalibriert.
- [ ] `F_total = F_1 + F_2` ist implementiert.
- [ ] Die Gesamtmessung ist über mehrere Lastpositionen validiert.
- [ ] Ein C#-Programm liest beide Kanäle über die HBM Common API aus.
- [ ] Das Programm läuft ohne MX Assistant oder catman im Vordergrund.
- [ ] Messwerte werden über Ethernet als dokumentiertes Format bereitgestellt.
- [ ] Ein Ubuntu-Client empfängt und protokolliert die Werte.
- [ ] Verbindungsverlust und Wiederverbindung wurden getestet.
- [ ] Alle Einstellungen und Abhängigkeiten sind dokumentiert.

---

## Beim gemeinsamen Einrichten zuerst erfassen

Sobald der Windows-PC verfügbar ist, bitte zunächst bereitstellen:

1. Foto des Typenschilds des MX440B
2. Screenshot aus HBM Device Manager mit IP und Firmware
3. Screenshot der Kanalkonfiguration in MX Assistant
4. Installierte Version der HBM Common API
5. Verzeichnisstruktur der mitgelieferten API-Beispiele
6. Fehlermeldungen vollständig als Text oder Screenshot
7. Gewünschte Ubuntu-Version und ROS-2-Version
8. Netzwerkaufbau zwischen Windows-PC, MX440B und Ubuntu-PC

Danach wird zuerst die Messung in MX Assistant validiert und erst anschließend die eigene API-Anwendung erstellt.
