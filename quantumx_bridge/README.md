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
