# Einstellungen am Touchdisplay

Stand: 3. November 2026. Firmware erfolgreich gebaut; die erweiterten Eingabemenüs und die Bildschirmruhe sind am Gerät noch zu testen. Bild und Touchnavigation der vorherigen Fassung sind bereits bestätigt.

## Entsperren und Zahlencode

Auf der Seite **Einstellungen** „Code eingeben“ wählen. Der anfängliche Displaycode ist **1234**, unabhängig von Benutzer und Passwort der Webseite. Ein neuer Code aus vier bis acht Ziffern lässt sich im Bereich **Display-Zahlencode** mit Wiederholung speichern. Danach erneut entsperren. Gespeicherte Codes haben nach Neustart Vorrang vor der menuconfig-Vorgabe. Der Code wird mit zufälligem Salt als PBKDF2-SHA256-Prüfwert in NVS gespeichert, nicht im Klartext exportiert.

„Sperren“ beendet die Freigabe. Nach einer Minute ohne Berührung wird sie ebenfalls beendet; offene Eingaben werden geschlossen und nicht gespeicherte Eingaben verworfen. Erfolgreich gespeicherte Einstellungen bleiben erhalten. Zwischen Anmeldeversuchen mindestens zwei Sekunden warten.

## Einstellungsbereiche

| Bereich | Inhalt |
|---|---|
| Raumtemperatur | normales Raumsoll, Urlaubssoll, Sollwert ohne Zeit |
| Heizkurve & Vorlauf | Bezugstemperatur, Steilheit, Raumkorrektur, Sommergrenze, Vorlaufgrenzen |
| Boiler | Soll, Hysterese, Kesselanhebung, Vorrang |
| Zeitprogramm | vier Startzeiten und vier Raumsollwerte, täglich gleich |
| Kessel & Brenner | Kesselgrenze/-reserve, Brennerhysterese, Mindestlauf- und Stillstandszeit, Pumpennachlauf |
| Mischer & Frostschutz | vollständige Laufzeit, Wechselpause, Regelband, Frostgrenzen, Messwertalter, optionale Pufferfühler |
| Restwärme | Aktivierung, Abschalttemperatur, maximale Laufzeit |
| MQTT | Broker, Basistopic, Benutzer, neues Passwort, Passwort löschen |
| Zeitserver | drei Server |
| Kachelofen | Aktivierung, Topic, beide Schwellen, maximales Empfangsalter |
| Gaszähler | Aktivierung, Topic, Aktualitätshinweis |
| Gas-Tagesbasis | stille Korrektur innerhalb der bisherigen Grenzen |
| Webzugang | Webbenutzer und neues Webkennwort mit Wiederholung |
| Display-Zahlencode | eigener Zahlencode mit Wiederholung |
| Sicherung & Firmware | Hinweis auf vorhandene Dateiübertragung über Webseite/USB |
| System & Fühler | Netzwerk-/Zeitstatus, Firmwarestand, MQTT-Topics und Einzeldiagnose der Fühler |

Die Betriebsart bleibt auf ihrer eigenen Seite ohne PIN umschaltbar. Es sind alle 41 benannten Web-Einstellfelder sowie Betriebsart und Zeitprogramm abgedeckt. Grenzwerte und Kombinationen prüft dieselbe bestehende Schnittstelle wie für Webänderungen.

## Eingaben und Speichern

Bereich in der Auswahl wählen, Feld antippen und über die Bildschirmtastatur bearbeiten. Zahlen mit Dezimalpunkt, Zeiten als HH:MM. Schalter wechseln zwischen Ein und Aus. „Übernehmen“ schließt die Eingabe; erst **Speichern** schreibt die Einstellungen. Während einer laufenden Übertragung wird kein weiterer Schreibauftrag angenommen.

Ein leeres neues MQTT-Passwort lässt das bestehende Passwort unverändert. Die eigene Löschoption löscht es. Bestehende Passwörter werden nicht ausgelesen oder angezeigt. Webzugang und Displaycode sind getrennt; der Displaycode gilt nicht als Anmeldung über das Netzwerk.

Änderungen an MQTT, Zeitservern, Kachelofen, Gaszähler und Gas-Tagesbasis starten das Gerät nach erfolgreichem Speichern neu. Der Button heißt dort **Speichern & Neustart**. Regelungs-, Zeitprogramm- und Restwärmewerte benötigen keinen Neustart. Nach Webzugangsänderung im Browser mit den neuen Daten anmelden.

## Bildschirmruhe

Nach 60 Sekunden ohne gültige Berührung wird nur die Hintergrundbeleuchtung ausgeschaltet. Regelung, Messung, MQTT und Verläufe laufen weiter. Der erste Tipp weckt ausschließlich auf; seine Berührung wird bis zum gemeldeten Loslassen von der Bedienung ferngehalten. Erst danach bedient ein weiterer Tipp die Oberfläche. Touchzähler und Koordinatenanzeige sind entfernt.

## Keine SD-Karte

Es wird keine SD-Karte initialisiert oder verwendet. Sicherungsdateien werden weiterhin im Browser heruntergeladen bzw. eingespielt. Firmwaredateien werden über Webseite oder USB installiert. Diese Dateiübertragungen sind keine lokalen Touch-Dateiauswahldialoge. Alle Einstellwerte sind am Touch bearbeitbar.

## Prüfung

Lokal bestanden: Firmware-Build, PIN-Anmeldung und Codewechsel, NVS-Fehler und Neustartpersistenz, Sperre veralteter Aufträge, Ablehnung externer Requests auch bei bekanntem internem Token, IPv4-/IPv6-Loopback-Erkennung, 60-Sekunden-Grenze, Aufwecken ohne Bedienaktion und Zählerüberlauf. Die neuen Tests wurden zusätzlich mit AddressSanitizer/UndefinedBehaviorSanitizer ausgeführt. Der Abgleich der Web-Eingabefelder meldet keine fehlenden Felder. Bestehende Einstellungs- und Regelungstests bestanden ebenfalls.

Die Touchbedienung nutzt die vorhandenen HTTP-Handler über die geräteinterne Loopback-Verbindung. Ein pro Start zufälliger interner Sitzungswert, die aktive lokale PIN-Freigabe und die Prüfung des lokalen Socket-Absenders sind gemeinsam erforderlich. Keine neue PIN-Anmeldung aus dem LAN. Web-Anmeldung und CSRF-Prüfung bleiben erhalten. Netzwerk-/Speicherarbeit läuft in einer separaten Task, nicht in der Regler-Task.

## Nächster Gerätetest

1. Vorher Einstellungen im Web sichern und die neue Firmware installieren.
2. Einstellungen mit dem anfänglichen Code entsperren, Werte mit dem Web vergleichen.
3. Ein Feld öffnen, ändern und zunächst abbrechen; danach das Speichern gezielt mit einem gewünschten Parameter prüfen.
4. Eine Minute ohne Berührung warten. Erster Tipp soll nur aufwecken; Einstellungen müssen wieder gesperrt sein.
5. Displaycode lokal ändern und nach Neustart erneut prüfen. Den neuen Code nicht in den Chat schreiben.

Gerätetest der neuen Funktionen steht aus. Die bereits erfolgreiche Bild-/Touch-Grundfunktion ersetzt diese Bedienprüfung nicht. Nach Änderungen die Markdown-Seiten auch im Docmost-Wiki aktualisieren.
