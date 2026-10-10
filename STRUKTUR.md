# Aufbau der Heizungsfirmware

`src/main.cpp` bindet die Bibliotheken, die öffentlichen Modulschnittstellen und die Firmwareabschnitte ein. Die Reihenfolge der Firmwareabschnitte erhält die Abhängigkeiten des bisherigen Programms.

## Eigenständige C++-Module

| Dateien | Aufgabe |
| --- | --- |
| `src/web/page.h`, `page.cpp` | HTML, CSS und JavaScript der Webseite |
| `src/diagnostics/cpu_load.h`, `cpu_load.cpp` | Initialisierung, Erfassung und Anzeige der groben CPU-Schätzung |
| `src/ntp_diagnostics/ntp_diagnostics.h`, `ntp_diagnostics.cpp` | Separate NTP-Abfragen und Anzeige der Zeitabweichungen |

Diese `.cpp`-Dateien kompiliert PlatformIO eigenständig. Die `.h`-Dateien beschreiben ihre öffentlichen Schnittstellen.

## Firmwareabschnitte

| Datei unter `src/firmware/` | Aufgabe |
| --- | --- |
| `shared_state.inc` | Gemeinsame Variablen, Gerätedefinitionen, Timer und Funktionsdeklarationen |
| `runtime_support.inc` | Gaszähler, Sensorvalidierung und Reparatur gespeicherter Einstellungen |
| `setup.inc` | Initialisierung, Webrouten und Start der Dienste |
| `loop.inc` | Hauptschleife, Auftragsübernahme und zyklische Verarbeitung |
| `web_status.inc` | Gemeinsamer Status-Snapshot für die Webseite |
| `web_handlers.inc` | WebSocket-Handler, Platzhalter und Anzeigeformatierung |
| `network.inc` | WLAN-Ereignisse, Verbindung und MQTT-Statusversand |
| `mqtt.inc` | MQTT-Empfang, Warteschlange und Befehlsverarbeitung |
| `clock.inc` | Netzwerkzeit, Sommerzeit und Zeitprogramme |
| `uptime.inc` | Bestehende Laufzeitberechnung für LCD/MQTT |
| `sensor_reading.inc` | Temperaturmessung und Boiler-Leseweg |
| `sensor_bus.inc` | Fühlertyp-Erkennung und OneWire-Leseweg |
| `heating.inc` | Heizkurve, Anforderungen, Brenner und Pumpen inklusive Fehlersperren |
| `mixer.inc` | I2C-Ausgänge, Mischer und Vorlaufsensor-Sperre |
| `lcd_menu.inc` | Tastatur, Eingaben und LCD-Menüs |
| `lcd_display.inc` | LCD-Hauptanzeige mit Änderungen einzelner Zeichen und X bei Sensorfehlern |
| `serial_console.inc` | Formatierung der seriellen Ausgabe |

Die `.inc`-Dateien sind bewusst eingebundene Implementierungsabschnitte, keine separat kompilierten Module. Die Bestandslogik teilt viele globale Variablen. Diese erste Aufteilung macht die Zuständigkeiten sichtbar, ohne zusätzlich die Datenhaltung oder Heizungsabläufe umzubauen. Sie darf nicht in `.cpp` umbenannt oder zusätzlich kompiliert werden, solange ihre Schnittstellen nicht getrennt sind.

## Konfiguration und Zugangsdaten

- `src/config.h`: Konfiguration, Pins und Konstanten.
- `src/secrets.h`: lokale WLAN- und MQTT-Verbindungseinstellungen; nicht veröffentlichen.
- `platformio.ini`: Buildumgebung und Bibliotheken.
- `lib/`: lokale LCD- und Tastaturbibliotheken mit den vorhandenen Hardwareanpassungen.

## Prüfen und bauen

Im Projektordner:

```sh
pio run -e nodemcuv2
```

Die Umstrukturierung wurde erfolgreich kompiliert. Ausgelagerte Firmwareabschnitte wurden vor dem Build durch Wiederzusammenfügen auf unveränderten Inhalt geprüft. Die Firmware wurde anschließend per OTA hochgeladen; der Anwender hat den Betrieb bestätigt. Ein mehrtägiger Stabilitätstest sowie der gezielte LCD-Menürückkehrtest stehen noch aus.
