# Prüfstand und offene Vorhaben

Stand 3. November 2026, aus Projektunterlagen und Nutzerrückmeldungen. Keine neue Hardwareprüfung für diesen Wiki-Export.

## Am Gerät vom Nutzer bestätigt

- Ethernet, DHCP, Weboberfläche, Anmeldung und MQTT-Verbindung.
- Kachelofen-Empfang mit Aktivstatus; Topic muss einschließlich führendem Schrägstrich exakt passen. Bei fünf Minuten Sendeabstand wurden 900 Sekunden maximales Alter eingestellt.
- Gas-Gesamtzählerstand über MQTT. Tagesverbrauch benötigt eine gültige Tagesbasis und aktuelle Messwerte; vollständiger Tageswechseltest bleibt offen.
- Alle fünf Hauptfühler liefern Werte; gemeinsame Anzeige zuletzt etwa 24,3 bis 25 °C. Eine zu strenge Prüfung interner DS18B20-Bytes wurde korrigiert.
- Betriebsarten ohne Anmeldung, gespeicherte Werte nach Kaltstart und richtige Vorauswahl.
- Heizkurvengrafik, Favicon, Einstellungssicherung und Wiederherstellung.
- USB-Flash und Web-Firmwareupload mit anschließendem Neustart.
- ESBE-86-Laufzeit auf der Webseite auf 240 Sekunden gestellt.

## Lokal geprüft, praktisch noch nicht vollständig nachgewiesen

Regelungslogik, Betriebsarten, Konfigurationsvalidierung, Sensor-Dekodierung, MQTT-Verarbeitung, Sicherungsprüfung und Updateabläufe sind lokal getestet. Das ersetzt keine Prüfung der realen Kessel-, Pumpen- und Mischerhydraulik. Physischer Stromausfall während OTA und tatsächlicher Bootloader-Rückfall wurden noch nicht bestätigt. Die bisherigen Ausgangsanzeigen sind keine elektrische Rückmeldung der angeschlossenen Geräte.

## Offen / noch nicht implementiert

1. Sommer-Pumpenschutz: Heizungs- und Boilerpumpe je einmal täglich 15 Sekunden. Uhrzeit und Zusammenspiel mit Boilerladung und Betriebsarten noch abschließend festlegen.
2. Mischerpflege: erwogen sind 20 Sekunden Auf und 20 Sekunden Zu mit Richtungswechselpause. Ablauf bei unbekannter Position/Referenzfahrt bewusst offengelassen.
3. Rücklauffühler: Einsatz für Temperaturdifferenz und Restwärmenutzung prüfen. Kein GPIO zugeteilt.
4. Endlagenrückmeldung am ESBE 86: potentialfreie Kontakte oder geeignetes Zubehör nicht bestätigt. Interne Endabschalter sind nicht automatisch nutzbare Meldekontakte. Kein Austausch auf Serie 90 geplant.
5. Waveshare 7-DSI-TOUCH-C: Treiber, PSRAM und native Oberfläche sind eingebaut; Bild, Beleuchtung und Touch-Seitenwechsel sind am Gerät bestätigt. Dauerlauf und weitere Bedienprüfungen stehen aus.
6. Externe Solid-State-Relais: genauer Typ und Treiberanpassung noch offen. Trägerplatine mit Federklemmen für ein angepasstes Gehäuse ist ein Vorhaben.

Die Firmware-Voreinstellung von 240 Sekunden ist in den aktuellen Builds enthalten. Die Geräteeinstellung hat der Nutzer direkt im Web vorgenommen.
