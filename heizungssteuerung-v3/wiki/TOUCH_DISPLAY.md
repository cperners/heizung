# Waveshare 7-DSI-TOUCH-C – erste Firmware

Stand 3. November 2026: Native Oberfläche eingebaut und mit ESP-IDF 5.5.5 erfolgreich gebaut. Bild, Hintergrundlicht und Touch sind am realen Display noch nicht geprüft.

## Hardware

Waveshare 7-DSI-TOUCH-C, Artikel 33259, 1024 × 600. LCD EK79007, Touch GT911. Fotos zeigen das Flachbandkabel am Anschluss DISPLAY des ESP32-P4-ETH. Nutzer bestätigt Rot an VCC 5V und Schwarz an GND der USB-Stiftbuchse; die Datenkontakte bleiben frei. Kein Umstecken unter Spannung. Strombudget und tatsächliche Spannungen bleiben vor Ort zu prüfen.

Display nutzt zwei DSI-Lanes und LDO-Kanal 3 mit 2,5 V für den internen PHY. Touch und Hintergrundlicht liegen auf I2C SDA GPIO7/SCL GPIO8. GT911 wird an 0x5D oder 0x14 gesucht; Backlight an 0x45/Register 0x96. Die bisherigen Sensor-, Relais- und Ethernetpins bleiben unverändert.

## Oberfläche

- Übersicht mit Außen, Raum, Kessel, Vorlauf, Boiler und Kachelofen, jeweils zwei Nachkommastellen. Ungültige Werte erscheinen als Strich.
- Status von Brenner, Heizungs-/Boilerpumpe, Mischer und Restwärmenutzung. Angezeigte Ausgangspegel sind keine elektrische Rückmeldung angeschlossener Geräte.
- Sechs Betriebsarten über dieselbe validierte, persistente Schnittstelle wie im Web; entsprechend der vorhandenen Vereinbarung ohne Anmeldung. Speicherung erfolgt nur in der Displaytask, nie im Regler.
- Gemeinsamer Verlauf mit 1-/6-/24-Stunden-Ansicht und einzeln schaltbaren Kurven. Daten stammen aus den bestehenden RAM-Verläufen; Neustart löscht sie. Skala in Grad Celsius, intern auf Hundertstel abgebildet.
- Lesbare Einstellungsübersicht. Geschützte Einstellungen werden weiterhin nach Anmeldung auf der Webseite bearbeitet. Keine neue lokale Passwortverwaltung eingebaut.

Die native Darstellung folgt der freigegebenen Seitenstruktur; Pixelbild und Bedienbarkeit müssen am Display beurteilt werden. Weboberfläche und MQTT bleiben parallel verfügbar.

## Build und erster Start

Offizielle Quellen zuerst mit `bash scripts/setup-display-deps.sh` laden. Sie liegen in `.local/display-dependencies`; das Skript verwendet festgelegte Quellstände. Quelllizenzen bleiben dort erhalten. Die lokalen CMake-Adapter binden die unveränderten Treiber ein.

Die aktuelle Buildkonfiguration aktiviert `HEATING_TOUCH_DISPLAY` und PSRAM (200 MHz). Ohne gefundenes PSRAM bleibt die Anzeige aus; die Konfiguration erlaubt das Weiterstarten ohne PSRAM. Displayinitialisierung und LVGL laufen in einer eigenen Task niedriger Priorität. Die Regler-Task wird vorher gestartet. Ein gemeldeter Display-Initialisierungsfehler beendet nur die Displaytask; Fehlerfreiheit unter realer Renderinglast ist noch nicht nachgewiesen.

Für die erste Inbetriebnahme vollständigen USB-Flash verwenden, damit Bootloader und Anwendung zur neuen Konfiguration passen:

```sh
scripts/idf.sh -p /dev/cu.usbmodem5C940425091 flash monitor
```

Im Projektordner ausführen; bei geändertem USB-Port diesen anpassen. Einstellungen nicht löschen. Eine aktuelle Einstellungssicherung vorher aufbewahren. Die Partitionstabelle bleibt unverändert. Weitere Updates über USB oder Web bleiben vorgesehen; Web verwendet ausschließlich `build/heizungssteuerung.bin`.

Nach dem Start prüfen:

1. PSRAM wird im seriellen Log erkannt; keine Neustartschleife.
2. Bild ist richtig orientiert und alle vier Seiten sind sichtbar.
3. Touch trifft die gewählten Schaltflächen. Bei Abweichung nicht mehrfach blind Betriebsarten umstellen, sondern Ausrichtung korrigieren.
4. Temperaturen stimmen mit dem Web überein; Betriebsartwechsel erscheint auch dort und bleibt nach Neustart erhalten.
5. Ethernet, MQTT und alle Fühler bleiben bei laufender Anzeige stabil. Echte Laufzeit-/Watchdogprüfung steht aus.

Die Firmware wurde gebaut (etwa 1,42 MiB, 29 % Platz in 2-MiB-Partition frei); Regelungs-Hosttests bestanden. Keine behauptete Hardware- oder Touchprüfung. Im Log beschreibt „Touch bereit“ nur die erfolgreiche Treiberinitialisierung; die Bedienprüfung folgt am Gerät.

## Dokumentation

Diese Seite ist auch als Markdown für das Docmost-Wiki bereitgestellt. Der lokale Export aktualisiert den dortigen Artikel nicht automatisch.

## Korrektur der Einschaltfolge (3. November)

Erster realer Test: Touch meldet GT911-ID, Bildschirm bleibt schwarz. Der ursprüngliche Code setzte nur die Helligkeit. Die korrigierte Firmware sendet vor der DSI-Panelinitialisierung die Touch-C-Freigabefolge am I2C-Controller (Register 0x95: 0x11, dann 0x17; Register 0x96: 0, dann 255). Jeder Schreibfehler wird gemeldet und beendet die Displayinitialisierung. Das entspricht dem Touch-C-Zweig des offiziellen Waveshare-BSP; der reale Erfolg ist noch zu prüfen.

Quelle: https://github.com/waveshareteam/Waveshare-ESP32-components/blob/80402cdb12d5170f0a0b1d8bc58f64d0ce175d3e/bsp/esp32_p4_platform/esp32_p4_platform.c

## Touchdiagnose nach erstem Bildtest

Nutzer bestätigt Bild und Beleuchtung nach korrigierter Einschaltfolge, aber noch keine Reaktion auf Berührung. Diagnoseversion zeigt unten rechts in der Übersicht die Zahl der Touchpunkte, Fehler und letzte Koordinaten; im seriellen Log alle zehn Sekunden Abfragen/Punkte/Fehler/außerhalb sowie die konfigurierte GT911-Fläche beim Start. Koordinaten werden nur innerhalb 1024×600 an LVGL weitergegeben. Noch keine ungeprüfte Spiegelung oder Achsenvertauschung angewandt.

## Touch-Ausrichtung korrigiert

Beim gezielten Druck unten auf „Verlauf“ meldete der GT911 x=381, y=28. Das entspricht einer um 180 Grad gegen das sichtbare Bild gedrehten Touchfläche. Beide Koordinatenachsen werden nun gespiegelt, mit inklusiven Maxima 1023/599. Der genannte Punkt wird damit auf x=642, y=571 abgebildet. Der praktische Seitenwechsel ist nach Installation der Korrektur noch zu bestätigen.

## Am Gerät bestätigt (3. November 2026)

Nach Ergänzung der Einschaltfolge und Spiegelung beider Touchachsen bestätigt der Nutzer: Bild und Beleuchtung funktionieren, und die Seiten wechseln passend zur Berührung. Langzeitstabilität, alle Betriebsartwechsel und Verhalten bei Displayausfall sind damit noch nicht vollständig geprüft.

## Diagnose nach erfolgreicher Inbetriebnahme entfernt

Auf Nutzerwunsch sind Touchzähler, Koordinatenanzeige sowie regelmäßige Berührungs-/Diagnoseprotokolle wieder entfernt. Beide Achsenspiegelungen und die korrigierte Display-Einschaltfolge bleiben erhalten. Echte Initialisierungs- und Lesefehler werden weiterhin gemeldet; Lesefehler höchstens einmal in zehn Sekunden. Die vorstehenden Diagnoseabschnitte dokumentieren die Fehlersuche, nicht die aktuelle Anzeige.

## Automatische Dunkelschaltung nach einer Minute

Nach 60 Sekunden ohne gültige Berührung wird die Hintergrundbeleuchtung ausgeschaltet. Anzeige, Touchabfrage und Heizungsregelung laufen weiter. Der erste Tipp schaltet nur das Licht wieder ein; seine komplette Berührung wird bis zu einer frisch gemeldeten Fingerfreigabe vom Bedienweg ferngehalten. Erst eine weitere Berührung bedient Schaltflächen. Ein leerer Abfragezyklus oder Lesefehler zählt nicht als Loslassen. Bei fehlendem Touchcontroller wird nicht automatisch abgeschaltet.

Die Zusatzanzeige mit Touchzähler und x/y-Koordinaten ist entfernt. Tests prüfen Zeitgrenze, Aktivitätsverlängerung, fehlgeschlagene Lichtschaltung, unterdrückte Aufweckberührung, Loslassen und Überlauf des Millisekundenzählers. Abschaltung und Aufwecken sind nach Installation noch am Gerät zu bestätigen. Die Zeit beträgt fest 60 Sekunden.

## Erweiterte Einstellungen und Bildschirmruhe

Die neue Fassung ergänzt sämtliche Einstellwerte, eigenen Displaycode (anfangs 1234, am Display änderbar) und Abschaltung nach einer Minute mit reinem Aufwecktipp. Keine SD-Karte. Die frühere reine Einstellungsübersicht ist ersetzt. Bedienung und Prüfgrenzen: [TOUCH_EINSTELLUNGEN.md](TOUCH_EINSTELLUNGEN.md). Die neuen Menüs sind noch am Gerät zu testen.
