# Vorgemerkt für 29. Oktober 2026

Am 28. Oktober mit dem Nutzer festgehalten; noch nicht implementiert:

1. **Täglicher Pumpenlauf im Sommerbetrieb:** Heizungspumpe und Boilerpumpe jeweils einmal täglich für 60 Sekunden laufen lassen (Pumpenschutz). Vor Umsetzung Zeitpunkt, gleichzeitigen oder nacheinander ausgeführten Lauf sowie Zusammenspiel mit normaler Boilerladung, Betriebsart Aus, Mischer und Sicherheitsabschaltungen klären. Der Wunsch betrifft Sommerbetrieb; keine pauschale Aktivierung in sämtlichen Betriebsarten vereinbart.
2. **Zusätzlicher Rücklauffühler:** Einsatz und Anschluss prüfen, insbesondere als Grundlage für Temperaturdifferenz und Restwärmenutzung. Noch kein GPIO zugewiesen und keine Änderung an der Regelung beschlossen.

## Restwärmenutzung – aktueller Stand

Abschalttemperatur bleibt einstellbar; Firmware-Startwert 30 °C. Nutzer hat inzwischen eine Angabe von 35 °C gefunden, deren Quelle im Gespräch nicht vorlag. Kein automatisches Umstellen auf 35 °C vereinbart. Die bisherigen Grenzen sind keine bestätigte Herstellerfreigabe für den Mono Star G15 (Baujahr 1993).

## Ergänzung vom 29. Oktober 2026

**Als nächster Umsetzungsschritt eingeplant:** Waveshare 7-DSI-TOUCH-C (33259), sieben Zoll, als lokale Touch-Bedienung zusätzlich zur Webseite. Zuerst Kabel, Stromversorgung, Pinbelegung, Treiber und PSRAM prüfen; danach native Oberfläche integrieren. Details: [TOUCH_DISPLAY.md](TOUCH_DISPLAY.md).

Die vorgemerkten Punkte Pumpenschutz im Sommerbetrieb und zusätzlicher Rücklauffühler bleiben erhalten und sind noch nicht implementiert.

## Präzisierung Pumpen-/Mischerschutz und Mischerantrieb (29. Oktober)

- Pumpenlauf auf Nutzerwunsch **je 15 Sekunden**, nicht mehr 60 Sekunden.
- Zusätzlich erwogen: Mischer **20 Sekunden Auf, danach 20 Sekunden Zu**, mit Richtungswechselpause.
- Nutzer hat die Frage Referenzfahrt/Schutzablauf ausdrücklich zunächst offengelassen. Noch nicht implementiert.
- Genannter Antrieb: **ESBE Motor Typ 86**. ESBE-Austauschliste nennt 220 V, 3-Punkt, 240 s und 5 Nm. Nutzer hat die Anpassung auf **240 Sekunden** bestätigt; Firmware-Voreinstellung entsprechend geändert. Bestehende gespeicherte Einstellungen bleiben erhalten: am Gerät nach Anmeldung unter „Mischer & Frostschutz → Vollständige Laufzeit“ auf 240 s setzen und speichern. Kein Austausch gegen Serie 90 vorgesehen.
- Gewünscht: echte Rückmeldung für Anfangs-/Endposition prüfen. Für den Typ 86 sind separate Meldekontakte bzw. ein nachrüstbares Hilfsschalterset noch nicht bestätigt. Zubehör des Nachfolgers Serie 90 nicht ungeprüft als passend ansehen. Interne Motorabschaltung ist kein nachgewiesener potentialfreier Meldekontakt.
- Herstellerquellen: https://www.esbe.eu/storage/FCEBFEC8BE2DB8ECFD53C2B307EBFCDF56AAB6B1E29F2AB76C9B4C0C43C2850E/03dd2ff541e446008bcbfe671eb42a5c/pdf/media/7f1c416a099643069250b53a5f3c24bd/ESBE-Replacement-Guide_DE.pdf und https://www.esbe.eu/de/produkte/stellmotoren/90-3-punkt

## Dokumentation und Docmost (31. Oktober 2026)

- Nutzer hat das Markdown-Wiki erfolgreich in den Docmost-Bereich „Heizungssteuerung“ importiert.
- Dauerhafter Arbeitswunsch: Bei künftigen Projektänderungen die betroffene Dokumentation mitpflegen und den Nutzer daran erinnern, auch das Docmost-Wiki zu aktualisieren. Geänderte Wiki-Seiten als Markdown bereitstellen; ein lokaler Export allein aktualisiert Docmost nicht automatisch.
- ESBE-86-Laufzeit wurde laut Nutzer auf der Webseite auf 240 Sekunden gespeichert; dafür war kein neuer Build/Upload nötig.

## Touch-Firmware – 3. November 2026

Native Touch-C-Oberfläche mit vier Seiten, Treibern und PSRAM erfolgreich gebaut. Reale Anzeige, Touch-Ausrichtung und Laufzeitprüfung noch offen; siehe TOUCH_DISPLAY.md. Der frühere Vermerk „noch nicht implementiert“ zum Display beschreibt den Stand vor diesem Datum. Docmost-Seite nach dem Funktionstest aktualisieren.

## Erweiterte Touchbedienung

Alle Einstellwerte mit eigenem Zahlencode (anfangs 1234, am Display änderbar) und Bildschirmruhe nach einer Minute umgesetzt und gebaut. Keine SD-Karte. Gerätetest der neuen Menüs und des Aufweckens steht aus; Nutzer möchte morgen testen. Siehe TOUCH_EINSTELLUNGEN.md.
