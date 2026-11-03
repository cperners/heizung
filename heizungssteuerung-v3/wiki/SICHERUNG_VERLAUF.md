<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/SICHERUNG_VERLAUF.md -->

# Einstellungssicherung und Kachelofen-Verlauf

## Sicherung

Nach Anmeldung unter „Einstellungen sichern & wiederherstellen“ auf „Einstellungen herunterladen“ klicken. Die JSON-Datei enthält die vollständige Regelung mit Betriebsart und Zeitprogramm, MQTT-Adresse, Benutzer und Basistopic, drei Zeitserver, Kachelofen- und Gaszähler-Einstellungen.

Nicht enthalten sind Display-Zahlencode, MQTT-Passwort, Webzugang, Gas-Tagesbasis, Gesamtzählerstand und Messverläufe. Bei Wiederherstellung bleiben die auf dem Zielgerät vorhandenen Zugangsdaten erhalten; ein neues Gerät muss diese separat bekommen. Die Datei enthält Netzwerkadressen und Topics und sollte entsprechend aufbewahrt werden.

Zum Zurückspielen Datei auswählen und bestätigen. Nur angemeldete Benutzer können exportieren oder importieren. Die Firmware prüft Schema, vollständige Felder, Datentypen und Grenzwerte aller Abschnitte vor der ersten Änderung. Unbekannte Felder, Duplikate, Kennwortfelder und inkompatible Versionen werden abgelehnt. Dateien müssen kleiner als 8192 Byte sein.

Eine akzeptierte Datei wird zuerst als einzelner Wiederherstellungsauftrag gespeichert. Beim folgenden Neustart wird dieser vor dem Start der Regelung auf alle Konfigurationsbereiche angewandt. Erst nach erfolgreicher Speicherung aller Bereiche wird der Auftrag gelöscht. Bei Stromausfall wird der Auftrag beim nächsten Start erneut angewandt. Bei einem anhaltenden Speicherfehler startet die Regelung nicht; das serielle Log zeigt den Fehler. Der Auftrag ist kein Zurücksetzen auf Werkseinstellungen und keine automatische Rücknahme nach einem fehlerhaften Heizungsparameter, der innerhalb der erlaubten Grenzen liegt.

## Gemeinsamer Temperaturverlauf

Die öffentliche Verlaufskachel zeigt Außen-, Raum-, Kessel-, Vorlauf- und Boilerfühler gemeinsam mit dem Kachelofen. Jede der sechs Kurven ist einzeln ein-/ausblendbar. Als Zeitraum stehen eine, sechs und 24 Stunden zur Verfügung. Die Y-Skala passt sich den sichtbaren Messwerten an; zum Vergleich nebeneinanderliegender Fühler kann der deutlich wärmere Kachelofen ausgeblendet werden. Es werden keine Kalibrierkorrekturen angewandt.

Die fünf lokalen Fühler werden im Minutenraster mit dem letzten Ergebnis des jeweiligen Intervalls im RAM gespeichert (maximal 1440 Punkte). Die Darstellung fasst für sechs bzw. 24 Stunden auf Zwei- bzw. Fünf-Minuten-Raster zusammen, jeweils letzter Wert. Ungültige Werte werden als Lücke geführt. Wie bei jeder Zusammenfassung können kurze Ausfälle innerhalb eines Intervalls verdeckt bleiben; für die aktuelle Gültigkeit ist die Diagnose maßgeblich.

Der Kachelofen bleibt im Fünf-Minuten-Raster mit dem letzten aktuellen gültigen MQTT-Wert (maximal 288 Punkte). Retained-Nachrichten und ungültige Werte werden nicht aufgenommen. Ohne neue Messung entstehen keine erfundenen Punkte. Lücken ab dem eingestellten maximalen Messwertalter werden nicht durch Verbindungslinien überbrückt.

Die Zeitachse zeigt Zeit vor jetzt und benötigt deshalb keine synchronisierte Uhr. Die Sensoraufzeichnung läuft auf dem ESP auch bei geschlossenem Browser. Verlaufabfragen erfolgen alle 30 Sekunden, die Zeitskala ist deshalb um bis zu 30 Sekunden verzögert. Neustart und Firmwareupdate löschen sämtliche Verläufe. Sie sind nicht Bestandteil der Einstellungssicherung; für dauerhafte Archivierung bleibt ioBroker geeignet.

## Prüfung

Lokal geprüft: Sicherungsexport ohne Kennwörter, Prüfung ohne Übernahme, ungültige Felder und Grenzwerte, Speicherausfall mit erhaltenem Wiederherstellungsauftrag, erneutes Anwenden und Löschen des Auftrags sowie begrenzter Verlauf ohne Retained-Daten. Firmware-Build und JavaScript-Syntaxprüfung erfolgreich. Der Nutzer hat Sicherung und Wiederherstellung am Gerät geprüft. Die vollständige Langzeitprüfung der Verläufe steht aus.

Das frühere Schema 2 ergänzt das MQTT-Basistopic; Schema 1 bleibt importierbar und verwendet den damaligen Standardpfad.

Schema 3 ergänzt Restwärmenutzung. Schema 1/2 ohne diesen Bereich erhält beim Import die neuen Startwerte aktiviert/30 °C/1800 Sekunden.
