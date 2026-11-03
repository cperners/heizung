# Weboberfläche und Anmeldung

Die Oberfläche läuft direkt auf dem ESP unter seiner DHCP-Adresse über HTTP, Port 80. Die zuletzt berichtete Adresse war 192.168.0.50; der Router kann eine andere vergeben. Die Seite benötigt keine externen Schriftarten oder Internetdienste.

## Ohne Anmeldung

Temperaturen, Fühlerdiagnose, Heizkurve, Boilerwerte, Zeitprogramm, Ausgangsanforderungen und Verläufe sind sichtbar. Auch konfigurierte MQTT-Topics, drei Zeitserver und NTP-Status einschließlich Alter der letzten Synchronisation werden angezeigt. Die sechs Betriebsarten können ausdrücklich auch ohne Login umgeschaltet werden. Die aktuelle Betriebsart ist im Auswahlfeld vorselektiert.

## Nach Anmeldung

Regelung, MQTT-Verbindung und Basistopic, Kachelofen, Gaszähler, Zeitserver und Webzugang sind änderbar. Sicherung, Wiederherstellung, Tagesbasiskorrektur und Firmwareupload benötigen ebenfalls Anmeldung. Die Passwortfelder geben gespeicherte Kennwörter nicht zurück. Die Mindestlänge des Webkennworts beträgt vier Zeichen.

Für die erstmalige Einrichtung wird der Webzugang im Konfigurationsmenü gesetzt und anschließend gebaut/geflasht. Bereits gespeicherte Zugangsdaten haben Vorrang vor Build-Vorgaben. Weitere Änderungen gehen über den Bereich Webzugang.

HTTP überträgt die Verbindung unverschlüsselt. Das Gerät ist für das vertrauenswürdige lokale Netz vorgesehen. Die Anmeldung ersetzt keine Transportverschlüsselung.

## Speichern

Regelungswerte werden validiert und ohne Neustart gespeichert. Änderungen an MQTT, Zeitservern sowie Kachelofen-/Gas-Einrichtung kündigen einen Neustart an. Ein leeres MQTT-Passwortfeld lässt das bestehende Passwort unverändert; Löschen erfolgt über die eigene Option.

## Zeitserver

Vorgaben: 192.168.0.1, 0.at.pool.ntp.org und 1.at.pool.ntp.org. Zeitzone Wien mit automatischer Sommerzeit. Der Router kann nur antworten, wenn er tatsächlich NTP anbietet. Die Anzeige nennt die konfigurierten Server, nicht zwingend den Server der letzten Antwort.

## Mischer

Unter **Mischer & Frostschutz → Vollständige Laufzeit** sind für den vorhandenen ESBE 86 **240 Sekunden** vereinbart und laut Nutzer am Gerät eingestellt. Das Ändern dieses Wertes auf der Webseite benötigt keinen Firmwareupload.

## Temperaturverlauf und Diagnose

Fünf Hauptfühler und Kachelofen haben einzeln wählbare Kurven mit 1-/6-/24-Stunden-Ansicht. Temperaturen erscheinen mit zwei Nachkommastellen; zusätzliche Dezimalstellen erhöhen nicht die Messgenauigkeit. Verläufe werden im RAM geführt und gehen beim Neustart verloren.

Die Kachel Fühlerdiagnose zeigt GPIO, Zustand, Versuche und Erfolge. Aufgeklappt erscheinen ROM-Adresse und Rohdaten. Ein fehlender Messwert allein beweist keinen GPIO-Schaden. Jeder lokale Fühler braucht seine eigene Datenleitung und einen Pull-up nach 3,3 V; siehe [Anschlüsse](HARDWARE.md).

## Korrektur der Einstellungsanzeige

Heizkurven- und Regelungswerte werden beim Laden passend zur Schrittweite des Eingabefeldes formatiert. Eine Steilheit von 0,1 erscheint ohne lange binäre Gleitkomma-Nachkommastellen. Die Regelungsberechnung bleibt unverändert. Diese Korrektur erfordert eine aktualisierte Firmware.
