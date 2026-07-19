# QuantumX bridge – erster API-Lesetest

Dieser erste Schritt liest die bereits im MX Assistant konfigurierten
Messsignale von MX440B-Kanal 1 und 2 gemeinsam über die HBM Common API aus.
Er verändert keine Geräte- oder Sensoreinstellungen.

Voraussetzungen:

- HBM Common API 6.3 unter dem Standardpfad installiert
- MX440B unter `192.168.10.20` erreichbar
- Kanal 1 und 2 im MX Assistant als Kraftsignale in `N` konfiguriert

Build und Test:

```powershell
dotnet build .\quantumx_bridge\src\QuantumXBridge\QuantumXBridge.csproj -c Release
.\quantumx_bridge\src\QuantumXBridge\bin\Release\net48\QuantumXBridge.exe 192.168.10.20 20 100
```

Argumente: Geräte-IP, Anzahl Messsätze und Abstand in Millisekunden. Diagnose
und Metadaten erscheinen auf stderr; stdout enthält ausschließlich NDJSON.

Der Snapshot-Test dient zunächst zur Prüfung von Verbindung, Firmware,
Kanalzuordnung, Einheit, Skalierung und Messwertstatus. Für die spätere
dynamische Messung wird nach erfolgreichem Test der Streamingpfad ergänzt.

## Verifizierter Hardwarestand

- Gerät: MX440B, Common-API-Modellkennung `MX440A`
- Seriennummer/UUID: `0009E5006EE2`
- IP-Adresse: `192.168.10.20`
- Firmware: `4.50.8.0`
- Common API: `6.3.0.145`
- Kanal 1: `AnalogIn_Connector1.Signal1`
- Kanal 2: `AnalogIn_Connector2.Signal1`

Die Common API benötigt laut ihrer lokalen XML-Dokumentation mindestens sechs
Sekunden zwischen `DaqEnvironment.GetInstance()` und dem ersten Netzwerkscan.
Der Reader wartet deshalb 6,5 Sekunden. Ohne diese Wartezeit wird das Gerät
nicht gefunden und die Streamingports fehlen in der `ConnectionInfo`.

Der Hardwaretest vom 19.07.2026 lieferte für beide Kanäle gültige Werte. Die
API-Zeitstempel beider Werte waren in jedem gemeinsamen Snapshot identisch.

## Einfache Kraftanzeige

Die Windows-GUI `QuantumXMonitor` zeigt Sensor 1, Sensor 2 und deren Summe in
Newton. Jeder angezeigte Wert ist ein gleitender Mittelwert aus den letzten 100
synchronen MX440B-Messwerten. Ungültige/Overrange-Werte werden nicht als Null
in das Fenster aufgenommen.

```powershell
dotnet build .\quantumx_bridge\src\QuantumXMonitor\QuantumXMonitor.csproj -c Release
.\quantumx_bridge\src\QuantumXMonitor\bin\x86\Release\net48\QuantumXMonitor.exe
```

Nach dem Start benötigt die Common API etwa 6,5 Sekunden Initialisierungszeit,
bevor die MX440B gefunden wird. MX Assistant muss währenddessen geschlossen
sein.

Falls der kontinuierliche HBK-Datenstrom wegen einer Firewallregel nicht
initialisiert werden kann, wechselt die GUI automatisch zur gepaarten
Einzelwertabfrage. Der 100-Werte-Mittelwert bleibt aktiv; die niedrigere
tatsächliche Abfragerate wird in der Statuszeile angezeigt.

Die Anzeige stellt dieselben gemittelten Werte zusätzlich als NDJSON auf
`127.0.0.1:5500` bereit. `test_run_gui.py` startet sie bei Bedarf mit
`--server-only`, übernimmt `force_total_n` als kompatiblen bisherigen
Kraftwert und protokolliert `force_1_n`, `force_2_n`, Zeitstempel, Sequenz und
Status zusätzlich. Impulsmittelwerte berücksichtigen jeden QuantumX-Messsatz
anhand von Zeitstempel und Sequenz nur einmal.
