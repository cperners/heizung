# Intelligente Heizungssteuerung – Waveshare ESP32-P4-ETH

> **Vorhandenes Board: Chiprevision 3.2.** Die lokale Umgebung ist auf ESP-IDF
> v5.5.5 umgestellt. Die Firmware enthält jetzt eine HTTP-Weboberfläche.
> Einstieg und Webzugang: [Weboberfläche](docs/WEB.md).



Modulares ESP-IDF-Projekt für Ethernet/IP101, fünf getrennte DS18B20, Brenner,
Heizkreis- und Boilerpumpe und einen verriegelten 3-Punkt-Mischer. Der lokale
Regler arbeitet ohne MQTT-Verbindung. MQTT liefert Messwerte, tatsächliche
Ausgangszustände, Fehler und validierte, persistierte Konfiguration.

**Stand 3. November 2026:** Ethernet, Webbedienung, MQTT-Eingänge, fünf Hauptfühler sowie Bild und Touchnavigation wurden vom Nutzer am Gerät bestätigt. Firmware-Build und lokale Regelungstests erfolgreich. Die reale Heizungsaktorik und der Dauerbetrieb sind noch nicht vollständig geprüft. Nicht als geprüfte oder zertifizierte Kesselsicherheitssteuerung einsetzen. Lastausgänge sind im
Auslieferungszustand gesperrt. Die Standardtemperaturen sind Beispiele und müssen
zur konkreten Anlage passen. Kesselinterne Sicherheitseinrichtungen,
Sicherheitstemperaturbegrenzer und erforderliche interne Umwälzung bleiben
unabhängig funktionsfähig.

## Einstieg

1. [Anschluss und Boardprüfung](docs/HARDWARE.md) lesen und mit der eigenen
   Boardrevision sowie den Relaisdaten vergleichen.
2. Die lokale ESP-IDF-**v5.5.5**-Umgebung ist unter `.local/` eingerichtet.
   Build im Projektordner: `scripts/idf.sh build`. Details: [Lokaler Build](docs/LOKALER_BUILD.md).
3. `idf.py set-target esp32p4`, anschließend `idf.py menuconfig` ausführen.
   Unter „Heizungssteuerung“ Brokeradresse, Zugangsdaten und Relaispolarität
   einstellen. Ausgänge zunächst gesperrt lassen.
4. `idf.py build`, danach `idf.py -p PORT flash monitor`. PORT durch den
   tatsächlich vorhandenen seriellen Anschluss ersetzen.
5. [Inbetriebnahme](docs/INBETRIEBNAHME.md) durchlaufen, Konfiguration anpassen.
   Erst danach Lastausgänge in `menuconfig` freigeben und erneut flashen.

Für Linux/macOS lassen sich die portablen Tests ohne ESP-IDF mit
`tests/run_host_tests.sh` ausführen. Die GitHub-Prüfung in
`.github/workflows/build.yml` baut zusätzlich die Firmware im offiziellen
ESP-IDF-v5.5.5-Container, sobald das Projekt in einem Repository ausgeführt wird.
Diese Prüfung wurde hier noch nicht ausgeführt.

## Architektur

| Modul | Aufgabe |
|---|---|
| `app_main` | Startfolge, 100-ms-Regler, 5-s-Telemetrie, Task-Watchdog |
| `sensors`, `ds18b20` | Acht getrennte Busse, ROM-/Scratchpad-CRC, Konversion, Gültigkeit |
| `outputs` | Einzige GPIO-Schreibstelle für Lasten, Freigabe, unabhängige 500-ms-Überwachung |
| `heating` | Heizkurve, Raumkorrektur, Zeitprogramm, Frostschutz, Brenner und Nachlauf |
| `mixer` | Richtungsverriegelung, Totzeit, Referenzfahrt, geschätzte Position |
| `boiler` | Warmwasser-Ladeanforderung mit Hysterese |
| `buffer` | Optionales Auswerten der drei Pufferfühler |
| `pv` | Validierte MQTT-Eingänge mit 30-s-Verfallszeit; kein Leistungsausgang |
| `ethernet` | IP101, P4-MAC/RMII, DHCP, Link-/IP-Status |
| `mqtt` | Last Will, Reconnect, QoS 1, fragmentierte Eingänge und separate Befehlsqueue |
| `ntp` | Drei NTP-Server, Wiener Zeitzone mit Sommer-/Winterzeit |
| `config`, `config_rules` | Standardwerte, strikte Grenzen, atomarer NVS-Commit |

Der Regler und seine Datentypen sind von Hardware und Transport getrennt. Weboberfläche und native Touch-Anzeige verwenden die gemeinsame Konfiguration. USB- und Web-Firmwareupdates sind vorhanden. Home-Assistant-Discovery und eine echte PV-Leistungsregelung sind nicht implementiert. Pufferfühler lassen sich bereits zuschalten; `pv_enabled:true`
wird bis zur Implementierung der Leistungselektronik absichtlich abgewiesen.
GPIO46 bleibt immer AUS.

Details: [Konfiguration und MQTT](docs/KONFIGURATION.md),
[Regelung und Fehlerreaktionen](docs/REGELUNG.md),
[Verifikationsstand](docs/VERIFIKATION.md).

Gasverbrauch per MQTT: [Einrichtung und Tagesbasis](docs/GASZAEHLER.md).

Externer MQTT-Fühler für den Kachelofen: [Einrichtung und Umschaltung](docs/KACHELOFEN.md).

[Einstellungssicherung und Kachelofen-Verlauf](docs/SICHERUNG_VERLAUF.md).

[USB- und Web-Firmwareupdates](docs/FIRMWARE_UPDATE.md).

Einstellbare Restwärmenutzung beim Wechsel auf Aus: [Verhalten und Prüfstand](docs/RESTWAERME.md).

[Touchdisplay: Anschluss, Bedienung und Prüfstand](docs/TOUCH_DISPLAY.md).

[Aktueller Projektstand als Markdown](docs/STATUS_2026-11-03.md).
