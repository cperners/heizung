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
| `runtime_diagnostics.inc` | Freier/minimaler Heap, größter freier Speicherblock und Schleifenabstände für Web/MQTT |
| `setup.inc` | Initialisierung, Webrouten und Start der Dienste |
| `loop.inc` | Hauptschleife, Auftragsübernahme und zyklische Verarbeitung |
| `web_status.inc` | Gemeinsamer Status-Snapshot für die Webseite |
| `web_page_response.inc` | Abschnittsweise HTML-Ausgabe mit festen Platzhalterwerten |
| `ota_diagnostics.inc` | Beobachtung von OTA-Start, Fortschritt, Fehlern und Abschluss |
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

## Mischer bei Betriebsart Aus

In `src/config.h` stehen `MISCHER_AUS_ZUFAHRT_MINUTEN` (2 Minuten) und `MISCHER_AUS_KESSEL_MAX_TEMP` (30 °C). In Aus bleiben beide Richtungen zunächst aus. Erst bei gültiger Kesseltemperatur bis zur Grenze startet die einmalige Schließfahrt. Bei wärmerem oder ungültigem Kesselsensor wird unterbrochen; nach erneuter Freigabe beginnt die volle Zeit neu. Nach Abschluss bleiben beide Richtungen aus. Wiedereinschalten bricht ab und gibt die vorhandene Initialisierung frei. Das gilt auch beim Start in Aus und hat dort Vorrang vor der Sensorfehler-Auffahrt. Der erste Gerätetest wurde vom Betreiber als passend gemeldet; eine vollständige Fehlerfallprüfung steht aus.

Der Kachelofen-MQTT-Timeout beträgt 16 Minuten und steht derzeit noch in `shared_state.inc`. Seine Auslagerung nach `config.h` ist vorgemerkt.

## Urlaubsbetrieb

Die Web-Betriebsart Urlaub verwendet durchgehend `tRoomUrlaub` (Anfangswert `URLAUB_DEFAULT_TEMP`, 15 °C) mit Raumregelung. Tag-/Nachtzeit und Kachelofen-Automatik bestimmen dann nicht den aktiven Sollwert bzw. die wirksame Regelungsart. Die normale Regelungsmodus-Auswahl bleibt erhalten und wird nach Verlassen von Urlaub wieder wirksam. Die Boileranforderung ist aus; vorhandene Sensorfehler-Nachläufe bleiben wirksam.

Die Urlaubstemperatur ist im Webformular für Wunschtemperaturen in 0,1-Grad-Schritten von 5 bis 40 °C einstellbar. Sie wird zusammen mit den übrigen Formulareinstellungen in der Hauptschleife gespeichert und erst nach erfolgreichem Commit aktiv. Modus und Temperatur bleiben nach Neustart erhalten. EEPROM-emulierter Speicher wurde am Ende um die Adressen 160 (Temperatur) und 168 (Aktivflag) auf 176 Byte erweitert; vorhandene Adressen bleiben unverändert.

`applyOperatingMode()` übernimmt Web-Betriebsarten sowie den neuen MQTT-Urlaubsbefehl mit Speicherung vor Laufzeitübernahme. Alte Betriebsartenbefehle bzw. LCD-Betriebsartänderungen beenden Urlaub. MQTT auf dem bestehenden `set`-Thema:

- `seturlaub`: Urlaub aktivieren, Antwort `h_urlaub`.
- `urlaubTemp:16.5`: Urlaubstemperatur speichern, Antwort `h_urlaubtemp`; aktiviert Urlaub nicht von selbst.
- Ungültige Urlaubstemperatur: `h_urlaubtemp_invalid`; Speicherfehler: `h_save_error`.

Die Antwort kommt im bestehenden JSON-Feld `answer` im nächsten Statusversand. Status enthält außerdem `urlaubAktiv`, `urlaubTemperatur` und Betriebsartkennung `U`. Der ausgewählte Regelungsmodus in `atRegelung` bleibt die normale Auswahl; während Urlaub ist die wirksame Regelung unabhängig davon Raumregelung.

WLAN-SSID und RSSI sind in einer gemeinsamen Kachel, beide werden über den Status-Snapshot aktualisiert. Der bisherige unbeschriftete WLAN-Schieberegler ist entfernt. Build, lokale Abläufe, MQTT-Grenzfälle und JavaScript/Platzhalter wurden geprüft. Ein Gerätetest für Urlaub einschließlich Neustart/Persistenz steht noch aus.

## Optimierungen nach dem Urlaubsbetrieb

- Alle sechs MQTT-Betriebsartenbefehle verwenden nun `applyOperatingMode()`. Bei fehlgeschlagenem Speichern bleiben die bisherigen Laufzeitwerte aktiv und die Antwort lautet `h_save_error`. Wiederholung eines bereits in diesem Lauf erfolgreich gespeicherten Modus vermeidet einen weiteren Flash-Schreibvorgang. Eine MQTT-Antwort wird erst aus dem Puffer entfernt, wenn der Statusversand vom MQTT-Client angenommen wurde; das ist keine Broker-Empfangsbestätigung.
- `web_page_response.inc` liefert die Hauptseite abschnittsweise mit festgehaltenen Platzhalterwerten aus. `web/template_stream.h` ersetzt ausschließlich bekannte Platzhalter; Prozentzeichen in CSS/JavaScript und Werten bleiben erhalten. Die vollständige HTML-Seite wird nicht mehr für jede Anfrage als großer String aufgebaut und kopiert.
- Webstatus und MQTT verwenden ihre JSON-Dokumente wieder, statt die Variant-Speicherpools bei jedem Zyklus neu anzulegen. Nach einem Allokationsfehler wird das betroffene Dokument zur Erholung zurückgesetzt. NTP-Text reserviert seinen begrenzten Arbeitsbereich vorab. Dies reduziert Speicherbewegungen, beweist aber noch keine Behebung der nach Tagen beobachteten Webverzögerung.
- `ota_diagnostics.inc` beobachtet ElegantOTA-Start, Fortschritt und Abschluss. Die Hauptschleife ergänzt Fehlercodes und erkennt ausbleibenden Fortschritt nach 30 Sekunden. Es werden keine laufenden Updates automatisch abgebrochen und keine Relais verändert. Ein bereits offener Vorgang wird als solcher gemeldet. Die sporadischen OTA-Abbrüche sind weiterhin nicht abschließend erklärt.

OTA-Diagnosen erscheinen in der Systemdiagnose, im MQTT-Objekt `ota` und über den lesenden Endpunkt `/ota-status`. `bytesWritten` zählt Firmwarebytes, `httpContentLength` die Multipart-HTTP-Länge einschließlich Verwaltungsdaten; beide Werte müssen daher nicht gleich sein. Die Beobachtung beginnt nach Installation dieser Version und wird beim Neustart zurückgesetzt. Ein erfolgreicher Abschluss kann durch den anschließenden Neustart nur kurz sichtbar sein.

Prüfungen: kompletter Firmware-Build; echte MQTT-Modusfunktionen mit erfolgreichen/fehlerhaften Speichertransaktionen und Wiederholungen; komplette HTML-Ausgabe bei Puffergrößen von 1 bis 1460 Bytes einschließlich Prozentzeichen/Leerwerten; OTA-Callbacks einschließlich fehlendem Fortschritt und erneutem Start; JSON-Speicherwiederverwendung über 10000 Zyklen; JavaScript-Syntax. Der Betrieb dieser Optimierungen am Gerät und der Langzeitvergleich stehen aus. Nach dem Update neue Speicher-Ausgangswerte festhalten, weil die dauerhaft behaltenen JSON-Pools den freien Heap gegenüber der Vorversion verändern können.
