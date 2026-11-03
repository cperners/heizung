# MQTT-Basistopic auf der Webseite ändern

Nach Anmeldung unter **System → MQTT** das Feld **MQTT-Basistopic** ändern, zum Beispiel von `heizung/anlage` auf `haus/heizung`. Mit den übrigen MQTT-Einstellungen speichern und den Neustart abwarten. Die Änderung benötigt danach keinen neuen Firmware-Build.

Unter dem gewählten Pfad liegen weiterhin dieselben Untertopics:

- `status` einschließlich Last Will (online/offline)
- `aussen`, `raum`, `kessel`, `vorlauf`, `boiler`, Pufferwerte
- `brenner`, `heizungspumpe`, `boilerpumpe`, `mischer_auf`, `mischer_zu`
- `soll/raum`, `soll/vorlauf`, `soll/boiler`, `mode`
- `diagnostics`, `config`, `config/set`, `config/result`

Die öffentlich lesbare Topic-Übersicht zeigt den gespeicherten neuen Pfad. ioBroker, Node-RED oder andere Empfänger müssen passend angepasst werden. Retained-Nachrichten im alten Pfad werden nicht automatisch gelöscht und können dort weiter sichtbar bleiben.

Die externen Empfangstopics für Kachelofen und Gaszähler sowie die vorbereiteten PV-Eingänge `heizung/pv/...` bleiben unabhängig. Diese Empfangstopics dürfen nicht im eigenen Anlagenpfad liegen. Die MQTT-Verarbeitung verwirft eigene Anlagenausgaben als Fühlerdaten, um Rückkopplung zu verhindern.

Das Basistopic muss 1 bis 159 druckbare ASCII-Zeichen lang sein, ohne Leerzeichen, Wildcards, doppelte oder führende/abschließende Schrägstriche. `$` am Anfang und Überschneidungen mit dem PV-Namensraum sind ausgeschlossen.

## Vorhandene Einstellungen und Sicherungen

Alte gespeicherte Verbindungsdaten werden einschließlich Benutzer und Passwort übernommen; das Basistopic startet mit `heizung/anlage`. Das neue Verbindungsformat liegt in einem separaten NVS-Namensraum, die alte Fassung bleibt für Firmware-Rückfall erhalten. Eine ältere Firmware sieht spätere Änderungen im neuen Format nicht; beim Rückfall kann daher der frühere Verbindungsstand gelten.

Neue Einstellungssicherungen verwenden Version 2 und enthalten das Basistopic. Sicherungen der bisherigen Version 1 ohne dieses Feld werden weiterhin angenommen und stellen das frühere Standardtopic `heizung/anlage` her. Kennwörter bleiben wie bisher vom Export und Import ausgenommen.

## Verifikation

Lokal geprüft: Migration alter NVS-Daten, Topic-Prüfung, persistente Änderungen, fehlgeschlagene Speicherung, Untertopic-Zusammensetzung und Abgrenzung benachbarter Topic-Pfade sowie alte/neue Sicherungen. Der vollständige ESP-IDF-Build prüft die MQTT-Integration. Die Umstellung beim echten Broker einschließlich ioBroker-Anpassung ist noch zu prüfen.
