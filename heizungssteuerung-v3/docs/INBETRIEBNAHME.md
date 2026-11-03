# Inbetriebnahme

1. Boardrevision, tatsächliche Anschlussnetze, Versorgung und Relaispolarität
   prüfen. Unabhängige Kesselsicherheitskette und Mischer-Endschalter prüfen.
   Bei Netzspannung und Eingriffen in den Kessel qualifiziertes Fachpersonal
   hinzuziehen. Zu Beginn keine Lasten an Relais anschließen.
2. Firmware mit gesperrter Ausgangsfreigabe bauen und flashen. Im Log müssen
   Lastausgänge als GESPERRT erscheinen. AUS-Pegel an allen sechs Ausgängen messen,
   einschließlich Boot, Reset, Flashen und Stromausfall. Aktiv-LOW gegebenenfalls
   neu konfigurieren, bevor Lasten angeschlossen werden.
3. Jeden der fünf Sensoren separat anschließen, Zuordnung durch Erwärmen prüfen.
   Gültige Werte und Fehlerzähler im Diagnose-Topic beobachten. Einen Sensor
   abziehen: nur dessen Messwert muss `null` werden; Kessel/Vorlauf-Fehler müssen
   Brenneranforderung sperren. Kein Sensor darf parasitär versorgt sein.
4. Ethernet DHCP und MQTT-Verbindung prüfen. Broker stromlos bzw. Kabel abziehen:
   Last Will/Verbindungsfehler und lokale Regelung prüfen. NTP nachweisen:
   Zeitfehlerbit muss verschwinden. Keine manuelle Systemzeitquelle vorgesehen.
5. Grenzwerte, Hydraulik, Mindestzeiten, Mischer-Laufzeit, Zeitprogramm und
   Warmwasservorrang anpassen. Für Sensorfehler und Übertemperatur jeden Wärmeweg
   an der konkreten Anlage bewerten. Besonders prüfen, ob Heizkreiszirkulation
   bei geschlossenem Mischer tatsächlich eine zulässige Wärmeabfuhr ermöglicht.
6. Erst mit Testlasten Ausgangsfreigabe aktivieren und erneut flashen. Brenner-
   Stillstandszeit nach Start beachten. Mischer zunächst referenzieren lassen;
   AUF/ZU dürfen niemals gleichzeitig Spannung erhalten. Richtungswechsel-Totzeit
   elektrisch messen, auch bei STOP dazwischen. Mechanische Laufzeit und Endpunkte
   mit der Schätzung vergleichen.
7. Mit simulierten Temperaturen die Grenzen testen: Brenner sofort nach Erkennen
   von Kessel-/Vorlaufübertemperatur AUS, unabhängig von Mindestlaufzeit.
   Sensorfehlerreaktionen, Warmwasser-Sollabschaltung, Pumpennachlauf und Frost
   prüfen. Sensorsampling dauert etwa 1 s, danach reagiert der 100-ms-Regler;
   dies ist kein hardwarebasierter Sofortschutz.
8. Regler-Timeout gezielt im Testaufbau verursachen: Ausgänge nach >500 ms ohne
   Regleranforderung AUS, Diagnose `output_timeout=true`, nur Neustart gibt den
   verriegelten Fehler frei. Task-Watchdog und unabhängige Sicherheitskette prüfen.
9. Gültige MQTT-Änderung schicken, `config/result` prüfen, neu starten und
   Persistenz vergleichen. Ungültige und retained Updates müssen abgewiesen
   werden. NVS bei Fehlern nicht blind löschen: Ursache prüfen. Ein bewusstes
   `idf.py erase-flash` löscht auch Konfiguration und setzt nach neuem Flashen
   Standardwerte; Lasten dafür trennen und anschließend neu in Betrieb nehmen.
10. Erst nach dokumentiert bestandenem Testaufbau echte Lasten anschließen,
    beaufsichtigten Betrieb und anschließend längeren Probebetrieb durchführen.

Keine hier aufgeführte Hardwareprüfung wurde vom Codeautor ausgeführt.
