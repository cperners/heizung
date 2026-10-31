# ESP32-Heizungssteuerung – Projektdokumentation

Stand: **31. Oktober 2026**. Diese Markdown-Fassung umfasst die Projektdokumentation
und die aktuellen Ergänzungen zu Anmeldung, WLAN und MQTT. Zugangsdaten und
Firmwaredateien sind nicht enthalten.

Die Dokumentation beschreibt den gemeinsam veröffentlichten Quellstand.
Bestätigte Gerätebeobachtungen und ausschließlich lokale Tests werden getrennt
gekennzeichnet. Verweise zwischen Kapiteln führen zum entsprechenden GitHub-Wiki.
Die Kapitel sind auch vollständig in dieser Datei enthalten.

## Inhalt

1. ESP32-Heizungssteuerung
2. Bedienung
3. Anmeldung und Netzwerk
4. Urlaubsbetrieb
5. Geregelte Restwärmeabfuhr in Betriebsart Aus
6. MQTT
7. Sensoren und Fehlerverhalten
8. Installation und Updates
9. Architektur
10. Tests und Fehlersuche


---

## ESP32-Heizungssteuerung

Dieses Wiki beschreibt die bestehende Heizungssteuerung und ihre Bedienung. Die Dokumentation wurde mit dem Quellcode des gemeinsam veröffentlichten Quellstands abgeglichen. Die geregelte Restwärmefunktion ist lokal geprüft; der vollständige Gerätetest steht aus.

Die Firmware verbindet Heizungs- und Boilerregelung, Mischersteuerung, fünf Temperaturfühler, eine lokale LCD-Anzeige und eine Weboberfläche. MQTT dient zur Übertragung von Messwerten und zur Fernbedienung. Die Uhrzeit stammt aus dem Netzwerk; eine separate Hardware-RTC wird nicht benötigt.

### Inhalt

- [Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk): geschützte Einstellungen, Passwortänderung, WLAN, MQTT und offener Update-Zugang.
- [Bedienung](https://github.com/cperners/heizung/wiki/Bedienung): Weboberfläche, Regelungsarten, Betriebsarten und Einstellungen.
- [Geregelte Restwärmeabfuhr in Betriebsart Aus](https://github.com/cperners/heizung/wiki/Restwaermebetrieb): Geregelte Wärmeabfuhr in Aus, Zieltemperatur und Testablauf.
- [Urlaubsbetrieb](https://github.com/cperners/heizung/wiki/Urlaubsbetrieb): Durchgehende Raumtemperatur, Boiler aus und MQTT-Bedienung.
- [MQTT](https://github.com/cperners/heizung/wiki/MQTT): Themen, Nachrichtenformate, Befehle und JSON-Status.
- [Sensoren und Fehlerverhalten](https://github.com/cperners/heizung/wiki/Sensoren-und-Fehlerverhalten): Fühler, Validierung und Reaktion der Ausgänge.
- [Installation und Updates](https://github.com/cperners/heizung/wiki/Installation-und-Updates): Konfiguration, Build, OTA und lokales HTTPS.
- [Architektur](https://github.com/cperners/heizung/wiki/Architektur): Quellcodeaufteilung, Datenübergaben und Bibliotheken.
- [Tests und Fehlersuche](https://github.com/cperners/heizung/wiki/Tests-und-Fehlersuche): Prüfabläufe und bekannte Grenzen.

### Funktionsübersicht

| Bereich | Funktion |
|---|---|
| Messwerte | Küche/Raum, Außen, Kessel, Vorlauf, Boiler und externe Kachelofentemperatur |
| Regelung | Manuell nach Raumtemperatur, manuell nach Außentemperatur oder automatisch nach Kachelofenstatus |
| Betriebsart | Aus, Automatikbetrieb, nur Heizung, nur Boiler oder Urlaub |
| Bedienung | Weboberfläche und lokale LCD-/Tastaturbedienung |
| Zeitprogramm | Tag- und Nachtbeginn mit getrennten Wunschraumtemperaturen |
| Netzwerk | WLAN, MQTT, Netzwerkzeit und OTA-Updates |
| Diagnose | Sensorfehler, Ausgangsstatus, Laufzeit, CPU-Schätzung, Speicher-/Schleifendiagnose und NTP-Abweichungen |

### Einordnung

Das Projekt ist eine individuell angepasste Steuerung. Die eingebauten Fehlerreaktionen ersetzen keine unabhängigen Temperaturbegrenzer oder eine vollständige Prüfung der Anlage. Insbesondere die Mischer-Auffahrt bei Vorlaufsensorfehler ist eine bewusst gewählte, anlagenspezifische Funktion.

Funktionstests wurden am Testgerät und am aktiven Gerät durchgeführt. Eine vollständige Prüfung aller Fehlerkombinationen und ein dokumentierter Langzeittest stehen weiterhin aus. Einzelne Ausgänge werden als geschalteter Zustand angezeigt; eine Rückmeldung der realen Pumpenbewegung oder Brennerfunktion ist nicht vorhanden.

### Neueste Ergänzungen

Die Temperatur-Kacheln enthalten die zugehörigen Ausgangssymbole. Boiler-Solltemperatur, Einschaltdifferenz, Vorlaufmaximum und Kurvensteilheit sind auf der Webseite einstellbar. Nur Boiler begrenzt die erste Mischer-Schließfahrt auf zwei Minuten und prüft nach zehn Minuten einmalig auf fehlende Vorlauf-Abkühlung. Bedienung und noch offene Gerätetests stehen auf [Bedienung](https://github.com/cperners/heizung/wiki/Bedienung) und [Tests und Fehlersuche](https://github.com/cperners/heizung/wiki/Tests-und-Fehlersuche).

### Aktueller Funktionsstand

Neu ist die geregelte Restwärmeabfuhr in Aus mit Brennersperre und gespeicherter Zieltemperatur von 30 bis 40 °C (Standard 35 °C). Hinzu kommen die getrennte 30-Grad-Heizkreisfreigabe bei Außentemperaturregelung, 21 Minuten Gültigkeit für Kachelofenmeldungen sowie die WLAN-/Neustartdiagnose. Die neue Restwärmefunktion ist lokal gebaut und geprüft, der vollständige Gerätetest steht aus. Siehe [Geregelte Restwärmeabfuhr in Betriebsart Aus](https://github.com/cperners/heizung/wiki/Restwaermebetrieb).

Die Raumregelung prüft nun auch bei fehlender Kesselfreigabe, ob ein gültiger Raumwert den Sollwert erreicht hat. Dies kann eine laufende Pumpenanforderung beenden, ohne eine zusätzliche Abschaltung allein wegen niedriger Kesseltemperatur einzuführen. Ein neuer Start benötigt weiterhin die bisherige Kesselfreigabe. Der Parallelbetrieb bleibt erhalten; Details auf [Bedienung](https://github.com/cperners/heizung/wiki/Bedienung).

### Raumwärmebedarf mit geregeltem Vorlauf

In Automatik und Nur Heizung bestimmt bei Raumregelung die Raumtemperatur weiterhin den Bedarf. Während die Pumpe läuft, regelt der Mischer den Vorlauf nach Heizkurve. Ein gültiger Außenfühler ist nun auch für diese Kombination erforderlich. Neue Pumpenstarts bleiben an die bisherige Kesselfreigabe gebunden. Der Parallelbetrieb mit dem Boiler bleibt erhalten.

Normale Mischerimpulse haben jetzt eine unabhängige Zeitfrist und eine Ruhezone von ±0,5 °C. Die Vorlauf-Kachel zeigt Sollwert und Ist-Soll-Differenz. Die normale Boilerladung wird bei Kesseltemperatur höchstens Boilertemperatur auch dann unterbrochen, wenn die übrige Ladeprüfung übersprungen wird. Die lokalen Tests und der Build waren erfolgreich; der vollständige Gerätetest dieser Ergänzungen ist offen.

### Anmeldung und Netzwerk auf der Webseite

Alle Werte bleiben sichtbar; Regelung und Betriebsart frei bedienbar. Die übrigen
Web-Einstellungen benötigen eine Anmeldung. Gespeicherte Anmeldepasswörter können
bis 64 UTF-8-Bytes lang sein. WLAN sowie MQTT-Server, Port, Basispfad und optionale
Broker-Zugangsdaten lassen sich nach Anmeldung testen und speichern. Bei Fehlern
wird die bisherige Konfiguration wieder verwendet. Die Update-Seite und der Upload
bleiben auf ausdrücklichen Wunsch ohne Anmeldung erreichbar.

Die Passwortänderung und der offene Update-Zugang sind am Gerät bestätigt. Weitere
Netzwerk- und Passwortgrenzfälle sind lokal geprüft, aber noch nicht vollständig am
Gerät dokumentiert. Details: [Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk).

---

## Bedienung

### Weboberfläche

Messwerte und Sollwerte bleiben ohne Anmeldung sichtbar. **Regelung** und **Betriebsart**
bleiben frei bedienbar; die übrigen Web-Einstellungen lassen sich erst nach Anmeldung
ändern und speichern. Passwortänderung, WLAN und MQTT stehen im geschützten Bereich.
Die Update-Seite ist ausdrücklich ausgenommen. Details: [Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk).

Die Kopfzeile zeigt Gerätename, Laufzeit seit Neustart, eine grobe CPU-Schätzung für beide Kerne sowie Datum und Uhrzeit. Auf schmalen Displays darf die Kopfzeile umbrechen. Laufzeit und CPU-Schätzung sind hellgelb dargestellt.

Die erste Kartenreihe enthält die fünf direkt angeschlossenen Temperaturfühler. Weitere Karten zeigen Gasverbrauch, Regelungsart, Betriebsart und NTP-Server. Die separate Ausgangsstatus-Kachel ist entfallen. Wunschraumtemperaturen und Kachelofenfunktionen stehen gemeinsam in einer weiteren Reihe.

Die Webseite fragt einmal pro Sekunde einen gemeinsamen Status ab. Der ESP32 stellt diesen in seiner Hauptschleife zusammen. Bei einem veralteten oder nicht erreichbaren Status erscheinen die Ausgangssymbole grau und erhalten einen Hinweis auf den fehlenden Status.

#### Temperaturtrend

Neben den Temperaturen einschließlich Kachelofen steht ein Trendpfeil:

| Zeichen | Bedeutung |
|---|---|
| ↑ orange | Gegenüber der vorherigen gültigen Messung mindestens etwa 0,1 °C gestiegen |
| ↓ blau | Gegenüber der vorherigen gültigen Messung mindestens etwa 0,1 °C gesunken |
| ↔ | Änderung kleiner als 0,1 °C oder erste gültige Messung |
| Kein Pfeil | Kein gültiger aktueller Messwert oder Verbindung ausgefallen |

Der Trend vergleicht aufeinanderfolgende Messungen, keine Stundenmittelwerte. Wiederholte Webabfragen desselben Messwerts verändern den Pfeil nicht. Nach einem erkannten Sensorfehler beginnt der Vergleich neu.

### Regelungsart und Betriebsart

Diese beiden Einstellungen haben unterschiedliche Aufgaben.

**Regelungsart** bestimmt, welche Temperatur für die Raumheizung maßgeblich ist:

- **Manuell Raumtemperatur:** Die Raumtemperatur bestimmt den Heizbedarf, unabhängig vom Kachelofenstatus. In Automatikbetrieb und Nur Heizung regelt der Mischer während laufender Pumpe den Vorlauf nach Heizkurve. Ein gültiger Außenfühler ist dafür ebenfalls erforderlich.
- **Manuell Außentemperatur:** Außenregelung bleibt ausgewählt, unabhängig vom Kachelofenstatus.
- **Automatik:** Bei aktivem Kachelofen wird die Außenregelung verwendet; bei inaktivem Kachelofen die Raumregelung.

**Betriebsart** wählt den Betriebsumfang: Aus, Automatikbetrieb, nur Heizung, nur Boiler oder Urlaub. Die Auswahl wird jeweils mit **Übernehmen** bestätigt. Sensorsperren werden durch diese Auswahl nicht aufgehoben.

### Wunschtemperaturen und Zeitprogramm

Tag- und Nacht-Wunschtemperaturen lassen sich in Schritten von 0,1 °C ändern. Die Webeingabe akzeptiert 5 bis 40 °C. Im selben Formular stehen die gespeicherten Tag- und Nachtbeginnzeiten als Uhrzeitfelder.

Mit **Temperaturen und Zeiten speichern** wird eine Anfrage an die Hauptschleife übergeben. Erst nach erfolgreicher Speicherung erscheint die Bestätigung. Bei einem Speicherfehler bleiben die bisherigen Wunschtemperaturen und Zeiten aktiv.

Der Tagbeginn muss vor dem Nachtbeginn liegen. Ein Tagesintervall über Mitternacht unterstützt dieses Formular nicht. Die bestehende Zeitlogik verwendet strenge Grenzen: Tagbetrieb gilt nach dem Tagbeginn und vor dem Nachtbeginn; exakt an der eingestellten Beginnminute gilt noch Nachtbetrieb. Das ist eine erhaltene Eigenschaft der Bestandslogik.

### Kachelofen

Die externe Temperatur kommt per MQTT. Die Schaltschwellen haben eine Hysterese: Aktivierung ab Einschaltgrenze, Deaktivierung bei oder unter Ausschaltgrenze. Die Einschaltgrenze muss größer als die Ausschaltgrenze sein. Die Tasten ändern die Werte in halben Gradschritten.

Bleibt länger als 21 Minuten eine gültige Temperaturmeldung aus, gilt der Kachelofen als inaktiv. In Automatik führt das zur Raumregelung. Eine manuell gewählte Regelungsart bleibt bestehen.

### Temperatur-Kacheln und Ausgangssymbole

Küche, Außen, Gaskessel, Vorlauf und Boiler zeigen oben die Bezeichnung, darunter Thermometersymbol, Messwert und Einheit. Danach folgen die zugehörigen Ausgangssymbole und anschließend Hinweise und Einstellungen.

| Kachel | Symbol und Bedeutung |
|---|---|
| Gaskessel | Flamme: Pastellrot bei Brennerausgang ein, Grau bei aus; Heizungspumpe: Grün bei Ausgang ein, Grau bei aus |
| Boiler | Boilerpumpe: Grün bei Ausgang ein, Grau bei aus |
| Vorlauf | Dreiwegeventil: Grün bei Fahrt mit ↑ Auf oder ↓ Zu, Grau bei Stopp |

Alle Symbole zeigen die geschalteten Ausgänge, keine mechanische Rückmeldung und keine bestätigte Flammenerkennung. Fehlende Webdaten werden grau mit entsprechendem Statushinweis angezeigt. Gleichzeitig aktive Mischer-Richtungen werden als Ausgangskonflikt angezeigt.

Das LCD zeigt ungültige Sensorwerte als X und aktualisiert nur geänderte Zeichen. Die Rückkehr aus einem Menü muss am Gerät vollständig geprüft werden.

### Boiler-Solltemperatur und Einschaltdifferenz

In der Boiler-Kachel lassen sich Solltemperatur und Einschaltdifferenz mit − / + in 0,1-°C-Schritten einstellen. Beide Felder sind mit den gespeicherten Werten vorbelegt. Der bestehende zulässige Wertebereich beträgt jeweils 0 bis 100 °C; er ist keine Empfehlung für die Anlage.

**Boilereinstellungen speichern** übergibt beide Werte an die Hauptschleife. Erst nach erfolgreichem EEPROM-Commit werden sie aktiv und als gespeichert bestätigt. Bei Fehler bleiben die bisherigen Werte aktiv. Unveränderte Werte lösen keinen erneuten Schreibvorgang aus. LCD und Webseite verwenden dieselben Speicherstellen.

Eine neue Ladung beginnt unter Solltemperatur minus Einschaltdifferenz; eine laufende Ladung endet bei Erreichen der Solltemperatur. Die Boiler-Kachel zeigt dazu den aktuellen Sollwert, die Einschaltdifferenz und die berechnete Startgrenze.

Zusätzlich erscheint **Differenz Kessel − Boiler** mit Vorzeichen und zwei Nachkommastellen: positive Werte in Pastellrot, null oder negative Werte in Pastellblau. Bei ungültigem Kessel- oder Boilersensor steht ein Fehlerhinweis. Dies ist ausschließlich eine Anzeige; die bisherige Pumpenfreigabe bei wärmerem Kessel bleibt unverändert.

### Heizkurve auf der Webseite

In der Vorlauf-Kachel stehen Messwert, berechneter Vorlauf-Sollwert und die Differenz Ist minus Soll. Die Differenz ist bei wärmerem Vorlauf positiv und bei kälterem negativ. Unter dem Mischersymbol stehen die Einstellungen für Vorlaufmaximum, Kurvensteilheit und Restwärmeziel. Beide lassen sich in 0,1-Schritten ändern und gemeinsam mit **Vorlaufeinstellungen speichern** sichern. Vorlaufmaximum: 0 bis 100 °C; Kurvensteilheit: 0,1 bis 10, ohne Einheit.

Die Werte entsprechen den vorhandenen LCD-Einstellungen und wirken bei Außentemperaturregelung. Die bestehende Heizkurvenprüfung bleibt aktiv: Beispielsweise führt ein Vorlaufmaximum unter der gewünschten Raumtemperatur zu einer ungültigen Heizkurve und keiner daraus abgeleiteten Heizanforderung. Bei Speicherfehlern bleiben beide bisherigen Einstellungen aktiv.

### Mischer in Nur Boiler

Beim Wechsel zu **Nur Boiler** und beim Start in dieser Betriebsart wird einmalig für zwei Minuten Mischer zu geschaltet. Danach sind beide Richtungen aus; die Fahrt startet innerhalb derselben Betriebsart nicht erneut. Beim Verlassen wird sie abgebrochen. Die normale Mischerinitialisierung wird währenddessen unterdrückt.

Die Dauer steht separat in `config.h` als `MISCHER_BOILER_ZUFAHRT_MINUTEN` mit Standardwert `2UL`. Die Restwärme-Wartebedingung von Aus gilt hier nicht. Die bestehende Vorlaufsensor-Fehlerreaktion hat Vorrang; die Zwei-Minuten-Frist wird ab Eintritt in Nur Boiler gemessen und bei einer Sensorsperre nicht neu gestartet.

Zehn Minuten nach Ende der ersten Schließfahrt wird der aktuelle Vorlauf mit dem Messwert beim Fahrtende verglichen. Ist die Temperatur gesunken, bleibt der Mischer stehen. Bei gleicher oder höherer Temperatur folgt genau eine weitere Schließfahrt mit derselben Dauer von zwei Minuten; anschließend bleibt der Mischer stehen. Es gibt keine endlose Wiederholung. Jede messbare Abnahme zählt, ohne zusätzliche Temperaturschwelle.

Die Wartezeit steht in `config.h` als `MISCHER_BOILER_NACHPRUEFUNG_MINUTEN` mit Standardwert `10UL`. Bei ungültigem Vergleichswert wird keine Nachfahrt ausgelöst. Ein Vorlaufsensorfehler während der Wartephase verwirft die Referenz; die bestehende Sensorfehlerbehandlung bleibt vorrangig. Verlassen von Nur Boiler bricht den gesamten Ablauf ab. Ein erneuter Eintritt beginnt einen neuen Ablauf.

### Geregelte Restwärme in Aus

Beim Wechsel von Automatik oder Nur Heizung auf **Aus** wird der Brenner abgeschaltet und über die zentrale Ausgangsfunktion gesperrt. Neue Heiz- und Boileranforderungen sind aus. Bei warmem Kessel führt der Heizkreis Restwärme ab: Die Heizungspumpe läuft und der Mischer regelt den Vorlauf nach der vorhandenen Heizkurve. Dies gilt auch bei zuvor gewählter Raumtemperaturregelung; die erreichte Raumtemperatur verhindert diese Restwärmeabfuhr nicht.

Die Zieltemperatur ist in der **Vorlauf-Kachel** in 0,1-°C-Schritten von **30 bis 40 °C** einstellbar. Der Standard bleibt **35 °C**. Mit **Vorlaufeinstellungen speichern** werden Zieltemperatur, Vorlaufmaximum und Kurvensteilheit gemeinsam gespeichert und erst nach erfolgreichem Speichern aktiv. Der Wert bleibt nach Neustart erhalten.

Bei gültiger Kesseltemperatur höchstens Zieltemperatur endet die reguläre Pumpenanforderung. Der Mischer fährt einmalig für die eingestellte Dauer (Standard zwei Minuten) zu und stoppt dann. Erwärmt sich der Kessel erneut über das Ziel, wird die Schließfahrt abgebrochen und die geregelte Restwärmeabfuhr wieder aufgenommen. Das gilt auch beim Start mit gespeicherter Betriebsart Aus.

Die Webseite zeigt **AUS / RESTWAERME** und **RESTWAERME / HEIZKURVE**. Die Heizkurve nutzt Tag-/Nacht-Sollwert und Außentemperatur; ihr Vorlauf-Sollwert bleibt durch das vorhandene Vorlaufmaximum begrenzt. Sensorfehlersperren, begrenzter Fehlernachlauf und die separate Pumpenschutzbewegung bleiben bestehen. Die hydraulische Wärmeabfuhr und der reale Ablauf müssen am Gerät noch geprüft werden. Details: [Geregelte Restwärmeabfuhr in Betriebsart Aus](https://github.com/cperners/heizung/wiki/Restwaermebetrieb).

### Heizkreisfreigabe bei Außentemperaturregelung

In Automatik und Nur Heizung wird der Heizkreis bei Außentemperaturregelung unabhängig von der bisherigen Boiler-Kesselfreigabe behandelt. Bei gültigen Sensoren und gültiger Heizkurve ist die Freigabe ab 30 °C Kesseltemperatur möglich. Darunter werden die reguläre Pumpenanforderung beendet und normale Mischerbewegungen gestoppt; die gesonderte Initialisierung und Sensorfehlerbehandlung bleiben bestehen. Die Boilerfreigabe bleibt unverändert. In der Raumregelung gilt die unten beschriebene Trennung zwischen neuem Pumpenstart und Abschalten am Raum-Sollwert.

Diese 30-Grad-Heizkreisgrenze und die einstellbare Restwärme-Zieltemperatur von standardmäßig 35 °C sind unterschiedliche Parameter. Die erste gilt im Heizbetrieb, die zweite für die Wärmeabfuhr in Aus.

### Systemdiagnose

Eine eigene Karte zeigt freien Arbeitsspeicher, dessen bisheriges Minimum, den größten freien Speicherblock sowie Schleifenabstände. Ein Abstand umfasst alles zwischen zwei Schleifenstarts einschließlich Wartezeiten und ist keine reine CPU-Ausführungszeit. Historische Maxima und Zähler gelten seit Neustart.

Zur Fehlersuche Werte kurz nach dem Start und beim Auftreten einer Verzögerung vergleichen. Eine einzelne Momentaufnahme beweist oder widerlegt kein Speicherleck.

### LoRaWAN-Kachelofentemperatur

Die Quelle liefert alle fünf Minuten. Der Timeout von 21 Minuten überbrückt bei regelmäßiger Fünf-Minuten-Sendung drei ausbleibende Meldungen mit etwas Zeitreserve. Danach wird der Ofen als inaktiv behandelt. Die Temperaturmeldung ist eine reine Zahl mit Dezimalpunkt. Beim geprüften Node-RED/ioBroker-Ausgang musste der Schreibtyp auf `command` (`ack=false`) stehen, damit der MQTT-Adapter sie tatsächlich veröffentlichte. Das hängt von der Adapterkonfiguration ab; ein aktualisierter ioBroker-Datenpunkt allein bestätigt keinen MQTT-Versand.

### Urlaub und WLAN-Kachel

Urlaub hält eine eigene einstellbare Raumtemperatur durchgehend, ohne Boilerbereitung. Die Bedienung, MQTT-Befehle und Grenzen stehen auf [Urlaubsbetrieb](https://github.com/cperners/heizung/wiki/Urlaubsbetrieb). Die normale Tag-/Nachttemperatur bleibt separat erhalten.

WLAN-Netzwerkname und Signalstärke stehen jetzt in einer gemeinsamen Kachel und werden mit dem Webstatus aktualisiert. Der unbeschriftete WLAN-Schieberegler wurde entfernt.

### OTA-Status

In der Systemdiagnose zeigt die Zeile OTA den zuletzt beobachteten Update-Zustand seit Neustart. Weitere Angaben liefert der lesende Geräte-Endpunkt `/ota-status`. Die Diagnose ist erst nach Installation der optimierten Firmware verfügbar und wird beim Neustart zurückgesetzt.

### Kombinierte Raum- und Heizkurvenregelung

In **Automatikbetrieb** und **Nur Heizung** entscheidet bei Raumregelung weiterhin die Raumtemperatur über den Heizbedarf. Der aktuelle Tag-/Nacht-Sollwert und die bisherige Raum-Einschaltdifferenz bleiben erhalten. Während die Heizungspumpe läuft, regelt der Mischer den Vorlauf nach derselben Heizkurve wie bei Außenregelung, begrenzt durch das Vorlaufmaximum.

Ein neuer Pumpenstart benötigt weiterhin die bisherige Kesselfreigabe. Eine laufende Pumpe wird nicht allein wegen Unterschreiten dieser Kesselfreigabe abgeschaltet: Bei weiterem Raumwärmebedarf bleiben Pumpenanforderung und Vorlaufregelung wirksam. Erreicht der Raum seinen Sollwert, endet die reguläre Pumpenanforderung und eine normale Mischerfahrt stoppt. Im Raum-Hystereseband bleibt der vorherige Pumpenzustand erhalten.

Für diese Kombination müssen **Kessel, Vorlauf, Außen und Raum** gültig gemessen werden. Bei einem zugeordneten Sensorfehler bleiben die bestehenden Fehlersperren und der begrenzte Nachlauf bereits laufender Pumpen aktiv. Ungültige Heizkurvenparameter erzeugen keine normale Pumpen- oder Mischeranforderung. Die gesonderte Mischerinitialisierung wird dadurch nicht zur normalen Heizkurvenfahrt.

Der **Urlaubsbetrieb behält seinen bisherigen Sonderablauf**. Bei Aus bleibt die gesonderte Restwärmeregelung maßgeblich. Boiler und Heizkreis können weiterhin parallel arbeiten; es wurde kein Boilervorrang eingeführt.

### Normale Mischerimpulse und Ruhezone

Bei normaler Heizkurvenregelung gilt eine Ruhezone von standardmäßig **±0,5 °C** um den Vorlauf-Sollwert. Innerhalb einschließlich beider Grenzen bleiben beide normalen Stellrichtungen aus. Eine laufende normale Fahrt wird beim Eintritt in die Ruhezone oder einem erkannten Richtungswechsel beendet. Die Ruhezone ist in `config.h` über `MISCHER_RUHEZONE_GRAD` einstellbar.

Ein normaler Impuls dauert höchstens fünf Sekunden. Die Hauptschleife prüft seine eigene Millisekundenfrist unabhängig vom Aufruf des Reglers und vom LCD-Zähler. Ist kein geregelter Heizkreis mehr aktiv, wird der normale Impuls ebenfalls gestoppt. Der vorhandene Abstand von 30 Sekunden zwischen normalen Impulsstarts bleibt bestehen.

Initialisierung, Nur-Boiler-Schließfahrt, deren einmalige Nachfahrt, abschließende Aus-Schließfahrt und Sensorfehler-Auffahrt behalten ihre eigenen Abläufe und Zeitgrenzen. Ein direkter Sonderfahrtbefehl übernimmt die Ausgänge und löscht die Zuordnung zu einem alten normalen Impuls. Die Softwarefrist ist keine unabhängige Hardwareabschaltung bei vollständig blockierter Hauptschleife.

### Boilerpumpe bei fehlender Temperaturdifferenz

Die normale Ladepumpenanforderung wird nun bei jeder zentralen Ausgangsaktualisierung zurückgesetzt, wenn gültige Temperaturen zeigen, dass der Kessel **nicht wärmer als der Boiler** ist. Dies wirkt auch dann, wenn die übrige Boilerprüfung wegen fehlender Kesselfreigabe übersprungen wird. Damit bleibt ein alter Ladepumpenzustand in diesem Fall nicht unverändert eingeschaltet.

Der vorhandene Sensorfehlernachlauf und die separate Pumpenschutzbewegung bleiben ausdrücklich bestehen. Eine tatsächliche Pumpenbewegung ist weiterhin nicht rückgemeldet; die Websymbole zeigen den geschalteten Ausgang.

---

## Anmeldung und Netzwerk

### Frei sichtbar und frei bedienbar

Alle Messwerte, Sollwerte und Diagnosen bleiben ohne Anmeldung sichtbar. Die beiden
Umschalter **Regelung** und **Betriebsart** bleiben jederzeit bedienbar. Alle übrigen
Web-Einstellungen erfordern eine Anmeldung: Wunschtemperaturen und Zeiten,
Boilerwerte, Vorlaufmaximum, Heizkurve, Restwärmeziel, Kachelofen-Schaltschwellen,
WLAN, MQTT und Änderung des Anmeldepassworts. MQTT-Befehle und LCD-Bedienung werden
von dieser Web-Anmeldung nicht verändert.

Die Eingabefelder starten gesperrt. Zusätzlich prüft der ESP jede geschützte
Speicheranfrage. Ein Umgehen der Oberflächensperre gibt damit keinen Speicherzugriff.
Der frühere direkte WebSocket-Pumpenschalter wird nicht mehr angeboten.

### Anmelden und abmelden

Die Anmeldung steht über den Kacheln. Benutzername und anfängliches Passwort werden
für jeden eigenen Build lokal in `src/secrets.h` über `HEIZUNG_WEB_USER` und
`HEIZUNG_WEB_PASSWORD` festgelegt. Produktive Zugangsdaten werden nicht veröffentlicht.
Ein auf dem Gerät gespeichertes Passwort hat beim nächsten Start Vorrang vor dem
Anfangswert der Firmware.

Eine Sitzung gilt für den jeweiligen Browser-Tab und bleibt beim Neuladen desselben
Tabs erhalten. Bis zu vier Sitzungen können bestehen; eine weitere Anmeldung ersetzt
die älteste. Nach 15 Minuten ohne geschützte Anfrage verfällt die Sitzung. Die reine
Hintergrundprüfung verlängert sie nicht. **Abmelden** oder ein ESP-Neustart beendet
sie. Anmeldeversuche sind auf einen Versuch je zwei Sekunden begrenzt.

### Anmeldepasswort ändern

Nach der Anmeldung das bisherige Passwort, das neue Passwort und seine Wiederholung
in der Kachel **Anmeldepasswort ändern** eingeben und speichern. Der Benutzername
bleibt unverändert. Erlaubt sind 1 bis **64 UTF-8-Bytes**; Umlaute und andere
Sonderzeichen können mehrere Bytes benötigen. Die frühere Acht-Zeichen-Grenze ist
aufgehoben. Bestehende achtstellige Passwörter bleiben beim Update gültig.

Erst nach erfolgreicher NVS-Speicherung gilt das neue Passwort. Ein falsches bisheriges
Passwort oder ein Speicherfehler lässt den bisherigen Wert aktiv. Bei Erfolg werden
alle Web-Sitzungen beendet; anschließend mit dem neuen Passwort anmelden. Das Passwort
bleibt bei Neustart und späteren Firmware-Updates erhalten.

### Ausnahme: Update-Seite

**Die Update-Seite und die Firmware-Upload-Endpunkte sind auf ausdrücklichen Wunsch
nicht passwortgeschützt.** Ein späterer Wechsel des Web-Anmeldepassworts aktiviert
dort keine Sperre. Wer den ESP im Netz erreicht, kann damit auch Firmware hochladen.
Die übrigen geschützten Einstellungen bleiben gesperrt.

Die frühere Ablehnung des geänderten Passworts an der separaten OTA-Anmeldung wurde
nicht abschließend erklärt. Der aktuelle Stand umgeht diese getrennte Anmeldung
bewusst; er ist keine nachgewiesene Behebung ihrer Ursache. Der offene Update-Zugang
wurde vom Betreiber am Gerät bestätigt.

Anmeldung und geschützte Einstellungen über den vorhandenen lokalen HTTPS-Proxy
nutzen. Der direkte ESP-Webserver bietet weiterhin HTTP; die Anmeldung verschlüsselt
die Verbindung nicht selbst. Details: [Installation und Updates](https://github.com/cperners/heizung/wiki/Installation-und-Updates).

### WLAN wechseln

1. Anmelden und in der WLAN-Kachel den neuen Netzwerkname und das neue WLAN-Passwort eingeben.
2. **WLAN testen und speichern** wählen. Die Annahmeantwort bestätigt noch keine Speicherung.
3. Der ESP wartet kurz auf die Antwortübertragung und testet das neue Netz ohne Warteschleife im Heizungsablauf.
4. Erst nach fünf Sekunden durchgehender Verbindung mit zugewiesener IP-Adresse werden die Daten als gemeinsamer NVS-Datensatz gespeichert.
5. Scheitert die Verbindung innerhalb von 30 Sekunden oder die Speicherung, versucht der ESP wieder das bisherige WLAN. Ein Stromausfall während des Tests lässt die zuvor gespeicherten Daten bestehen.

Der Name darf 1 bis 32 Bytes enthalten, die WPA/WPA2-Passphrase 8 bis 63 Bytes. Offene
WLANs und rohe 64-stellige PSK-Schlüssel werden in diesem Formular nicht angeboten.
Gespeicherte WLAN-Passwörter werden nicht an den Browser zurückgesendet. Ohne
gespeicherten Datensatz gelten die lokalen Firmware-Vorgaben.

In einem anderen WLAN kann sich die ESP-Adresse ändern. Dann gegebenenfalls
DHCP-Reservierung, DNS und das Weiterleitungsziel des HTTPS-Proxys anpassen. Eine
WLAN-Verbindung allein bestätigt noch keine Erreichbarkeit von Proxy oder MQTT.
Für den Rückfall muss das bisherige WLAN erreichbar sein. Es wird kein offener
Einrichtungshotspot gestartet.

### MQTT konfigurieren

In der geschützten MQTT-Kachel stehen:

| Feld | Bedeutung |
|---|---|
| Server | IPv4-Adresse oder Hostname ohne `http://` oder `mqtt://`, maximal 63 ASCII-Zeichen |
| Port | Ganze Zahl von 1 bis 65535 |
| Gemeinsamer Basispfad | Bis 80 ASCII-Zeichen ohne Leerzeichen, `+` oder `#`; abschließender Schrägstrich wird ergänzt |
| Benutzername | Optional, maximal 63 UTF-8-Bytes |
| Passwort | Optional, maximal 127 UTF-8-Bytes; wird nicht zurückgesendet |
| Gespeichertes Passwort beibehalten | Bei leerem Passwortfeld bisherigen Wert weiterverwenden |

**Benutzername und Passwort beide leer** bedeutet Verbindung ohne Broker-Anmeldung,
auch wenn die Auswahl zum Beibehalten noch gesetzt war. Ein Passwort ohne Benutzername
wird abgelehnt. Bei vorhandenem Benutzer ersetzt ein eingegebenes neues Passwort den
bisherigen Wert. Ohne Beibehalten-Auswahl kann auch ein leeres Passwort gespeichert
werden. Das Limit des Web-Anmeldepassworts gilt nicht für MQTT.

Server, Port, Basispfad und Zugangsdaten werden gemeinsam getestet und gespeichert.
Die Hauptschleife trennt den bisherigen Broker, leert alte Ein-/Ausgangswarteschlangen
und baut eine saubere MQTT-Sitzung auf. Während des Tests werden keine MQTT-Befehle
übernommen und keine Statuswerte veröffentlicht. Erst nach erfolgreicher
Broker-Verbindung wird gespeichert und werden die drei Themen für Steuerbefehle,
Kachelofen und Gaszähler abonniert. Scheitert der Verbindungsversuch innerhalb von
30 Sekunden oder die Speicherung, werden die bisherigen Werte gemeinsam wieder
verwendet. Eine erfolgreiche Verbindung bestätigt nicht sämtliche Broker-ACLs.

Alle Sende- und Empfangsthemen verwenden den neuen Basispfad. Ein führender Schrägstrich
wird beibehalten, aber nicht automatisch hinzugefügt. Node-RED, ioBroker und andere
Teilnehmer müssen ihre Themen ebenfalls umstellen. Historische Retain-Nachrichten
auf dem alten Broker oder Pfad werden nicht gelöscht. Sensor- und Brennersperrstatus
werden im neuen Pfad erneut veröffentlicht. Für den Kachelofen muss nach erfolgreicher
Umstellung eine neue Nachricht eintreffen; der bisherige Wert gilt nicht als frische
Messung aus dem neuen Thema.

Die alten gespeicherten Server-/Themen-Datensätze bleiben lesbar und übernehmen beim
Update zunächst die lokalen Broker-Zugangsdaten. Ein neuer gespeicherter Datensatz
enthält auch Benutzername und Passwort. WLAN und MQTT können nicht gleichzeitig
umgestellt werden. Ein MQTT-Wechsel ändert die ESP-IP-Adresse nicht.

### Prüfstand

Am Gerät bestätigt sind die Änderung des Anmeldepassworts und der anschließend
bewusst offene Update-Zugang. Die lokale Prüfung umfasst zusätzlich die längere
Passwortgrenze, Weiterverwendung alter Passwortdatensätze, Abmeldung, gesperrte
Speicheraufrufe, optionale MQTT-Anmeldung, Migration und Rückfall bei Netzwerk- oder
Speicherfehlern. WLAN-/MQTT-Wechsel und sämtliche Grenzfälle mit langen Passwörtern
sind noch nicht vollständig am Gerät dokumentiert. Siehe [Tests und Fehlersuche](https://github.com/cperners/heizung/wiki/Tests-und-Fehlersuche).

---

## Urlaubsbetrieb

Urlaub ist eine eigene Betriebsart für eine durchgehende, einstellbare Raumtemperatur ohne Boilerbereitung. Der Betrieb bleibt aktiv, bis eine andere Betriebsart gewählt wird. Es gibt keine automatische Rückkehr nach einem Datum.

### Webbedienung

Die Betriebsarten stehen in der Reihenfolge **Aus, Automatikbetrieb, Nur Heizung, Nur Boiler, Urlaub**. Urlaub wählen und Übernehmen drücken. Unter Wunschraumtemperatur besitzt Urlaub eigene Plus-/Minus-Tasten in 0,1-Grad-Schritten. Anfangswert sind 15 °C, zulässig sind 5 bis 40 °C. Mit Temperaturen und Zeiten speichern wird die Einstellung übernommen; die Speicherbestätigung erscheint erst nach erfolgreichem Speichern.

Die Urlaubstemperatur gilt unabhängig vom Tag-/Nachtzeitprogramm. Die wirksame Regelung ist während Urlaub Raumregelung, unabhängig von der normalen Regelungsmodus-Auswahl und dem Kachelofenstatus. Die normale Auswahl bleibt erhalten und wird nach Verlassen von Urlaub wieder maßgeblich.

### Boiler, Sensoren und Mischer

Es entsteht keine Boiler-Heizanforderung. Eine bereits laufende Boilerpumpe kann bei einem zugeordneten Sensorfehler aufgrund des bestehenden Fehlernachlaufs vorübergehend weiterlaufen. Der Urlaubsmodus setzt die bestehenden Brenner-, Sensor- und Mischersperren nicht außer Kraft.

Urlaub ist nicht die Betriebsart Aus. Die temperaturabhängige zweiminütige Mischer-Schließfahrt von Aus wird beim Wechsel auf Urlaub beendet; die normale Mischerinitialisierung wird wieder freigegeben.

### MQTT

Alle Befehle auf dem bestehenden Steuer-Thema senden, im produktiven Aufbau `/SmartHome/Keller/Heizung/set`, als einfachen Text, QoS 1, Retain aus.

| Nachricht | Wirkung | Antwort im JSON-Feld answer |
|---|---|---|
| `seturlaub` | Urlaub aktivieren und speichern | `h_urlaub` |
| `urlaubTemp:16.5` | Urlaubstemperatur speichern; aktiviert Urlaub nicht selbst | `h_urlaubtemp` |
| `setwinter` | Urlaub beenden, Automatikbetrieb wählen | `h_winter` |
| `setheizung` | Urlaub beenden, nur Heizung wählen | `h_heizung` |
| `setboiler` | Urlaub beenden, nur Boiler wählen | `h_boiler` |
| `setoff` | Urlaub beenden, Aus wählen | `h_aus` |

Dezimalpunkt verwenden. Ungültige Temperatur ergibt `h_urlaubtemp_invalid`, fehlgeschlagene Speicherung `h_save_error`. Die Antworten erscheinen im regulären JSON-Status. Das ist weiterhin ein einzelner Antwortpuffer, keine bestätigte Befehlswarteschlange.

Der Status enthält `urlaubAktiv` (Boolean), `urlaubTemperatur` (Zahl in °C) und die Betriebsartkennung `U`. `atRegelung` bleibt die normale ausgewählte Regelungsart (0/1/2); bei aktivem Urlaub ist die wirksame Regelung dennoch Raumregelung. Für eine vollständige Interpretation beide Felder berücksichtigen.

### Speicherung und Prüfung

Modus und Temperatur bleiben nach Neustart erhalten. Die EEPROM-emulierte Speicherung wurde am Ende um Temperatur an Adresse 160 und Aktivflag an Adresse 168 auf 176 Bytes erweitert. Bestehende Adressen blieben gleich. Anfangswert ist in `config.h` als `URLAUB_DEFAULT_TEMP` definiert.

Build, lokale Regelungs-/Speicherprüfungen und Webprüfungen sind erfolgreich. Upload und die neue Auswahlreihenfolge wurden vom Betreiber bestätigt. Die vollständige Urlaubsprüfung einschließlich MQTT, Speichererhalt nach Neustart und aller Sensorfehlerkombinationen steht noch aus.

---

## Geregelte Restwärmeabfuhr in Betriebsart Aus

Lokal gebaut und mit Hosttests geprüft; Gerätetest offen.

In Aus bleibt der Brenner über die zentrale Ausgangsfunktion gesperrt. Boilerbereitung und neue Wärmeanforderungen sind aus. Die Heizungspumpe und der Mischer führen bei warmem Kessel Restwärme über den Heizkreis ab.

| Kesselzustand | Verhalten |
|---|---|
| Gültig, über 35 °C | Heizkreis nach Heizkurve betreiben, auch bei vorheriger Raumregelung |
| Gültig, höchstens 35 °C | Pumpe regulär aus; Mischer einmal für zwei Minuten schließen, dann beide Richtungen aus |
| Erneut über 35 °C | Schließfahrt abbrechen, Restwärme wieder geregelt abführen |
| Ungültig | Kein blindes Zufahren wegen vermeintlicher Abkühlung; Sensorfehlerbehandlung |
| Wechsel aus Aus | Restwärmezustand beenden, vorhandene Betriebsart und Mischerinitialisierung übernehmen |

Die Zieltemperatur ist in der Vorlauf-Kachel von 30 bis 40 °C in 0,1-°C-Schritten einstellbar. Sie wird zusammen mit Vorlaufmaximum und Kurvensteilheit über **Vorlaufeinstellungen speichern** übernommen. Erst nach erfolgreichem Speichern werden die neuen Werte aktiv. Standard bleibt 35 °C; der neue Wert verwendet EEPROM-Adresse 172 innerhalb des bestehenden 176-Byte-Bereichs. Ungültige oder bisher nicht initialisierte Werte werden auf 35 °C gesetzt.

Die Heizkurve verwendet die aktuellen Tag-/Nacht-Raumsollwerte, den Außensensor und die vorhandenen Heizkurvenparameter. Die erreichte Raumtemperatur verhindert die Restwärmeabfuhr nicht. Der berechnete Vorlauf-Sollwert bleibt durch die konfigurierte maximale Vorlauftemperatur begrenzt. Das ersetzt keine unabhängige Hardware-Temperaturbegrenzung.

Konfiguration in `src/config.h`:

- `MISCHER_AUS_KESSEL_MAX_TEMP 35.0f`: Standard der gespeicherten Zieltemperatur; Laufzeitwert ist `restwaermeZielTemp`.
- `MISCHER_AUS_ZUFAHRT_MINUTEN 2UL`: Schließfahrt nach Erreichen des Ziels.
- Vorlaufmaximum und Kurvensteilheit verwenden die vorhandenen gespeicherten Einstellungen.

Sensorfehler behalten den vorhandenen begrenzten Pumpennachlauf für bereits laufende Pumpen. Ausgeschaltete Pumpen starten bei Sensorfehler nicht. Während der Restwärmephase gilt die vorhandene Vorlaufsensor-Fehlerreaktion (einmalige Auffahrt, danach Sperre). Eine ungültige Heizkurve fordert keine Pumpen- oder Mischerregelung an. Die vorhandene Pumpenschutzbewegung bleibt separat bestehen.

Die Restwärmeprüfung gilt auch beim Start mit gespeicherter Betriebsart Aus; ein warmer Kessel wird ohne Einschalten des Brenners abgekühlt. Ein vollständiger Stromausfall beendet naturgemäß die Regelung.

Webanzeige: `AUS / RESTWAERME` und `RESTWAERME / HEIZKURVE`. MQTT: zusätzliche Felder `restwaermeAktiv` und `restwaermeZieltemperatur`; die Betriebsart bleibt Aus.

### Lokale Prüfungen

`python3 tests/host/test_off_heat_recovery.py` führt tatsächliche Quellfunktionen mit simulierten Sensoren und Ausgängen aus. Geprüft werden 35-Grad-Grenze, Brennersperre, Heizkurve trotz ausgewählter Raumregelung, Vorlaufmaximum, Mischer-Fahrimpulse, Zweiminuten-Stopp, Wiedererwärmung, Betriebsartwechsel, kalter Start in Aus, Nacht-Sollwert, ungültige Heizkurve, Sensorfehler und zehnminütiger Fehlernachlauf. Der Hosttest läuft mit AddressSanitizer und UndefinedBehaviorSanitizer. Er prüft keine elektrischen Ausgänge oder Wärmeabfuhr der realen Anlage.

Am Gerät nach Upload: Von Automatik oder Nur Heizung auf Aus wechseln. Brenner muss aus bleiben, bei warmem Kessel muss die Heizungspumpe laufen, der Mischer muss den Vorlauf regeln. Bei gültigem Kesselwert höchstens 35 °C muss die reguläre Pumpenanforderung enden und die Schließfahrt nach der eingestellten Dauer stoppen. Der reale Temperaturverlauf und die Wärmeabfuhr sind noch nicht bestätigt.

### Abgrenzungen

Die Auswahlanzeige bleibt während der Wärmeabfuhr in Betriebsart Aus. Die Restwärmefunktion zündet keinen Brenner und aktiviert keine Boilerladung. Ein gesonderter Sensorfehlernachlauf kann eine bereits laufende Pumpe trotzdem bis zu seiner Frist weiterbetreiben. Die bestehende Pumpenschutzbewegung ist eine unabhängige Ausnahme zur normalen Pumpenanforderung.

Die Zwei-Minuten-Fahrt und Zehn-Minuten-Nachprüfung in **Nur Boiler** sind ein anderer Ablauf. Sie werden durch die Restwärmefunktion in Aus nicht ersetzt.

---

## MQTT

### Verbindung und Themenpräfix

Die anfänglichen Vorgaben für Broker, Port, Benutzername, Passwort, Client-ID und
Themenpräfix stehen in der lokalen `src/secrets.h`. Diese Datei wird nicht veröffentlicht.
**Server, Port, gemeinsamer Basispfad und optionale Broker-Zugangsdaten lassen sich
nach Web-Anmeldung ändern** und bleiben nach erfolgreichem Test gespeichert. Ein
gespeicherter Datensatz hat beim Start Vorrang; die Client-ID bleibt eine lokale
Firmware-Vorgabe. Beide Zugangsdaten leer bedeutet Verbindung ohne Broker-Anmeldung.
Details zu Beibehalten, Migration und Rückfall: [Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk). Test- und
Produktionsgerät benötigen weiterhin getrennte Gerätekennungen und passende Themen.

Im Folgenden bezeichnet **P** das konfigurierte Präfix einschließlich abschließendem Schrägstrich. Im produktiven Aufbau wird `/SmartHome/Keller/Heizung/` verwendet. Groß-/Kleinschreibung und der führende Schrägstrich gehören zum Thema.

### Empfangene Nachrichten

| Thema | Format | Zweck |
|---|---|---|
| `Pset` | Einfacher Text | Steuerbefehl |
| `PkachelofenTemp` | Zahl mit Dezimalpunkt, etwa `65.4` | Externe Kachelofentemperatur |
| `Pgaszaehler` | Fortlaufender Gesamtzählerstand in m³, etwa `685.300` | Gaszähler |

Für Steuerbefehle: Raw/Text, QoS 1 und Retain aus. Keine Anführungszeichen, zusätzlichen Leerzeichen oder Zeilenumbrüche hinzufügen.

#### Regelungsarten

Alle drei Befehle werden auf `Pset` gesendet:

| Nachricht | Gewählter Modus | Bestätigung im JSON-Feld `answer` |
|---|---|---|
| `tin` | Manuell Raumtemperatur | `h_tin` |
| `tau` | Manuell Außentemperatur | `h_tau` |
| `tauto` | Automatik | `h_tauto` |

#### Betriebsarten

| Nachricht auf `Pset` | Wirkung | Antwort |
|---|---|---|
| `setwinter` | Automatikbetrieb | `h_winter` |
| `setboiler` | Nur Boiler | `h_boiler` |
| `setheizung` | Nur Heizung | `h_heizung` |
| `setoff` | Aus | `h_aus` |

#### Weitere bestehende Befehle

Der Code enthält außerdem `up`, `down`, `save`, `normal`, `party`, `seton`, `ntpupdate` und numerische Einstellbefehle. Die Namen sind teilweise historisch: **`save` speichert nicht die Einstellungen, sondern senkt beide Wunschtemperaturen um 2 °C.** `up` erhöht beide um 0,2 °C; `down` senkt den Tagwert um 0,2 und den Nachtwert um 0,1 °C. `party` erhöht beide um 2 °C, `normal` lädt die gespeicherten Werte. Für normale Einstellungen ist das Webformular mit geprüfter Speicherbestätigung vorzuziehen.

### JSON-Status

Der Status wird ungefähr alle zehn Sekunden auf **`PJDATA`** veröffentlicht, sofern die MQTT-Verbindung besteht. Der Versand verwendet QoS 1 und Retain. Retain bedeutet, dass ein neu verbundener Empfänger zunächst auch einen älteren gespeicherten Status erhalten kann.

| Feld | Bedeutung |
|---|---|
| `S0Kessel`, `S1Vorlauf`, `S2Aussen`, `S3Kueche`, `S4Boiler` | Temperaturvariablen der fünf Fühler |
| `T0Room`, `T1Boiler`, `T2Vorlauf` | Aktiver Raum-, Boiler- bzw. Vorlaufsollwert |
| `Day`, `Night` | Tag- und Nachtbeginn |
| `BrennerRelais`, `HeizungsRelais`, `BoilerRelais` | Ausgangszustände als Text `"0"` oder `"1"` |
| `MischerAufRelais`, `MischerZuRelais` | Mischer-Ausgangszustände als Text `"0"` oder `"1"` |
| `atRegelung` | Gewählter Regelungsmodus als Zahl: 0 manuell Raum, 1 manuell Außen, 2 Automatik |
| `betriebsart` | Bestehende Betriebsartkennung |
| `answer` | Letzte noch nicht ausgegebene Befehlsantwort; sonst `---` |
| `U0Uptime`, `B0Brenner`, `Uhrzeit` | Laufzeit, Brenner-Laufzeitzähler und Zeitangabe |
| `WlanRetry`, `WlanRSSI` | WLAN-Diagnose |
| `tvmax`, `taumin`, `n`, `PumpenNachlauf` | Weitere Regelungswerte |

Seit der jüngsten Ergänzung beschreibt `atRegelung` den **gewählten Modus**. Bei Automatik steht dort immer 2; daraus lässt sich die gerade wirksame Raum-/Außenregelung nicht direkt ablesen. Das zusätzliche Textfeld `regelungsModus` wird nicht gesendet.

Die JSON-Temperaturvariablen enthalten keine eigene Gültigkeitskennzeichnung. Für die Bewertung eines Sensorfehlers müssen die separaten Sensorthemen berücksichtigt werden; ein Zahlenwert in JSON allein bestätigt keinen gesunden Fühler.

#### Befehlsbestätigung

Die Antwort erscheint in der nächsten JSON-Statusmeldung und kann daher ungefähr einen Sendezyklus später kommen. Der aktive Code veröffentlicht sie **nicht auf einem eigenen `Panswer`-Thema**. Das Antwortfeld ist ein einzelner Puffer: Bei mehreren Befehlen vor dem nächsten Versand kann eine spätere Antwort eine frühere überschreiben. Es ist keine Bestätigung jeder einzelnen Anfrage.

### Separate Sensormeldungen

| Thema | Inhalt |
|---|---|
| `Psensorstatus/kessel` | Temperatur mit einer Nachkommastelle oder `SENSORFEHLER` |
| `Psensorstatus/vorlauf` | Wie oben |
| `Psensorstatus/aussen` | Wie oben |
| `Psensorstatus/raum` | Wie oben |
| `Psensorstatus/boiler` | Wie oben |
| `Pbrennersperre` | Klartext zur Kesselsensor-Sperre |
| `Psensordiagnose/boiler` | Boiler-Lesestatus bzw. Fehlergrund |

Diese Themen verwenden QoS 1 und Retain. Sie werden bei Änderungen und nach einer erneuten Verbindung veröffentlicht; Sendeversuche werden auf höchstens einmal pro Sekunde begrenzt.

### Gasverbrauch

Der Tagesverbrauch ist die Differenz zwischen aktuellem Gesamtstand und Tagesbasis. Die Basis wird gespeichert. Falls um Mitternacht keine Messung vorliegt, beginnt die Erfassung mit der ersten neuen, nicht als Retain empfangenen Meldung des Tages. Ein alter Retain-Wert setzt keine neue Tagesbasis. Rückläufige Zählerstände werden verworfen. Die Webseite zeigt drei Nachkommastellen; es handelt sich um m³, nicht um kWh.

### Empfangsgrenzen

MQTT-Nachrichten werden im Callback geprüft und in eine Warteschlange übernommen. Die Hauptschleife verarbeitet sie. Aufgeteilte Nachrichten, leere Nachrichten, eingebettete Nullzeichen und Payloads ab 64 Bytes werden verworfen. Kachelofentemperaturen haben eine zusätzliche engere Längenprüfung. Eine volle Warteschlange führt zum Verwerfen neuer Nachrichten. QoS 1 kann doppelte Zustellung bedeuten; relative Befehle wie `up` sind deshalb nicht generell gegen Wiederholung abgesichert.

### Laufzeitdiagnose im JSON

Das Objekt `diagnostics` wird im bestehenden Zehn-Sekunden-JSON mitgesendet:

| Feld | Einheit und Bedeutung |
|---|---|
| `heapFreeBytes` | Aktuell freier 8-Bit-Heap in Bytes |
| `heapMinBytes` | Niedrigster freier Heap laut Heap-Verwaltung seit Start, Bytes |
| `heapLargestBlockBytes` | Größter aktuell zusammenhängender freier Block, Bytes |
| `loopRecentMaxMs` | Größter Schleifenstart-Abstand im zuletzt abgeschlossenen Messfenster von ungefähr einer Sekunde, ms |
| `loopMaxMs` | Größter erfasster Schleifenstart-Abstand seit Beginn der Schleifenmessung, ms |
| `loopOver100ms` | Anzahl erfasster Abstände über 100 ms seit Beginn der Schleifenmessung |

Die Schleifenmessung beginnt in `loop()` und umfasst nicht die komplette Setup-Dauer. Das jüngste Fenster wird beim MQTT-Versand abgelesen; nicht jedes Ein-Sekunden-Fenster wird über MQTT übertragen. Speicherwerte sind Momentaufnahmen, keine vollständige Aufzeichnung jedes kurzfristigen Engpasses.

Kachelofenmeldungen werden nach mehr als **21 Minuten** ohne gültige Aktualisierung als veraltet behandelt.

### Einheitliche Betriebsarten und neue Urlaubskommandos

`setoff`, `seton`, `setboiler`, `setwinter`, `setheizung` und `seturlaub` verwenden dieselbe Speicherfunktion wie die Web-Betriebsarten. Vorher änderten ältere Befehle die Laufzeitwerte direkt und prüften den Speichererfolg nicht. Jetzt werden neue Werte zuerst gespeichert und erst danach aktiv. Bei Fehler bleibt der bisherige Modus erhalten, Antwort `h_save_error`. Wiederholungen eines in diesem Lauf bereits erfolgreich gespeicherten Modus vermeiden weitere Flash-Schreibvorgänge.

Die Urlaubskommandos und Statusfelder sind auf [Urlaubsbetrieb](https://github.com/cperners/heizung/wiki/Urlaubsbetrieb) beschrieben. `urlaubTemp:16.5` setzt nur die Urlaubstemperatur. `urlaubAktiv` kennzeichnet den Vorrang der Urlaubs-Raumregelung gegenüber der normalen Auswahl in `atRegelung`.

Die Befehlsantwort bleibt nun im Puffer, solange der MQTT-Client den Statusversand ablehnt oder der Status nicht erzeugt werden kann. Erst eine erfolgreiche Annahme zum Versand leert den Puffer. Dies ist keine Broker-Quittierung und schützt weiterhin nicht vor dem Überschreiben durch einen späteren Befehl.

### OTA-Objekt

Der reguläre JSON-Status enthält zusätzlich `ota`:

| Feld | Bedeutung |
|---|---|
| `stage` | Beobachtete Phase: Start angefordert, bereit, Übertragung, Erfolgsmeldung, Fehler, bereits offen oder seit 30 Sekunden kein Fortschritt |
| `attempts` | Vom Start-Callback beobachtete Versuche seit Neustart |
| `bytesWritten` | Übertragene Firmwarebytes laut Fortschritts-Callback |
| `httpContentLength` | HTTP-Multipart-Länge einschließlich Verwaltungsdaten; nicht identisch mit Dateigröße |
| `errorCode`, `error` | Fehlercode und Fehlertext, soweit vom Update-Framework geliefert |
| `activityAgeMs` | Alter der letzten beobachteten Callback-Aktivität |

Die Diagnose greift nicht in einen Upload ein und bricht ihn nicht automatisch ab. Ein erfolgreicher Abschluss kann wegen des anschließenden Neustarts nur kurz sichtbar sein oder vor dem nächsten MQTT-Zyklus verschwinden. Das Sendeintervall bleibt ungefähr zehn Sekunden.

### Ergänzte Boiler-Statuswerte

Der regelmäßige JSON-Status enthält neben `T1Boiler` (Boiler-Solltemperatur) nun `boilerEinschaltdifferenz` und `boilerLadestartUnter`, jeweils als Zahlen in Grad Celsius. `boilerLadestartUnter` ist Solltemperatur minus Einschaltdifferenz. Das Sendeintervall bleibt unverändert.

Die farbige Differenzanzeige Kessel minus Boiler ist eine Webanzeige und fügt kein eigenes MQTT-Feld hinzu. Die neuen Webformulare ergänzen keine neuen MQTT-Kommandos. Historische Temperaturkommandos sind damit nicht automatisch auf die neue geprüfte Web-Speicherung umgestellt.

### Restwärme- und Wiederanlaufdiagnose

| Feld | Bedeutung |
|---|---|
| `restwaermeAktiv` | Aus befindet sich im Restwärmepfad; tatsächliche Ausgänge können zusätzlich durch Fehlerbedingungen begrenzt sein |
| `restwaermeZieltemperatur` | Gespeicherte Kessel-Zieltemperatur, Standard 35 °C, Webbereich 30–40 °C |
| `recovery.resetReason` | Neustartgrund |
| `recovery.previousLoopPhase` | Grobe vorherige Programmphase, soweit erhalten und validiert |
| `recovery.wifiDisconnects` | WLAN-Abbrüche seit Start |
| `recovery.wifiReconnectAttempts` | Vom Programm ausgelöste WLAN-Wiederanläufe seit Start |
| `recovery.wifiLastDisconnectReason` | Zuletzt erfasster WLAN-Abbruchgrund; 0 vor dem ersten Abbruch |
| `recovery.watchdogInitResult`, `recovery.watchdogAddResult`, `recovery.watchdogStatusResult` | Ergebnisse der Watchdog-Einrichtung/Registrierungsprüfung |
| `recovery.watchdogTimeoutSeconds` | Konfigurierte Watchdogfrist, derzeit 200 Sekunden |

Die Restwärmefunktion setzt die Betriebsart nicht auf Automatik: Sie bleibt Aus. Die normale Moduswahl in `atRegelung` wird ebenfalls nicht überschrieben. Die Restwärmeregelung verwendet unabhängig davon die Heizkurve. Es wurde kein neuer MQTT-Befehl für die Restwärme-Zieltemperatur ergänzt. Das Statusintervall bleibt ungefähr zehn Sekunden.

---

## Sensoren und Fehlerverhalten

### Anschlüsse und Erkennung

Die fünf Temperaturfühler sind einzeln verdrahtet: ein Fühler pro OneWire-Anschluss. Die direkte Gerätekennungs-Abfrage setzt diesen Aufbau voraus; mehrere Fühler an einem Anschluss sind damit nicht vorgesehen.

| Fühler | GPIO im aktuellen Code | Index |
|---|---|---|
| Kessel | 32 | 0 |
| Vorlauf | 25 | 1 |
| Außen | 26 | 2 |
| Küche/Raum | 27 | 3 |
| Boiler | 33 | 4 |

Die Erkennung speichert den Sensortyp pro Fühler. Unterstützt werden die Familien DS18S20, DS18B20 und DS1822. Fehlgeschlagene Typ-Erkennung wird nach etwa 30 Sekunden erneut versucht. Der Boiler verwendet einen vollständigen OneWire-Leseweg statt der früheren Mischung aus eigener Handroutine und Bibliothek.

### Gültigkeitsprüfung

Die Messung prüft Anwesenheit, Datenblock und Prüfsumme. Komplett aus Null oder 255 bestehende Daten werden verworfen. Werte müssen endlich sein und im Bereich −55 bis 125 °C liegen. Ein Messwert gilt bei fehlerhafter Lesung oder mehr als 30 Sekunden ohne gültige Aktualisierung als ungültig.

Webseite und MQTT-Sensorthemen zeigen Sensorfehler; das LCD zeigt X. Die letzte Temperaturvariable kann intern bestehen bleiben, ist aber bei Ausfall nicht als gültiger Messwert zu interpretieren.

### Brenner

Bei ungültigem oder zu altem Kesselsensor wird der Brenner zentral gesperrt. Eine Anforderung aus einer anderen Funktion kann diese Sensorsperre nicht übergehen. Der Sperrstatus erscheint auf der Webseite und per MQTT.

### Mischer

Außerhalb der Betriebsart Aus ist bei einem Vorlaufsensorfehler die anlagenspezifisch gewünschte Reaktion:

1. Mischer einmalig für 20 Sekunden öffnen.
2. Danach beide Stellrichtungen abschalten und weitere Bewegungen sperren.
3. Kommt vorher ein gültiger Messwert, endet die Fehler-Auffahrt und die normale Regelung wird wieder freigegeben.

Ohne Sensor startet diese Auffahrt auch nach einem Neustart. Öffnen bedeutet bei dieser Anlage mehr heißes Kesselwasser im Heizkreis. Ein unabhängiger Temperaturbegrenzer wurde vom Betreiber bestätigt. Diese Reaktion ist keine allgemeine Empfehlung für andere Anlagen; sie muss zum jeweiligen hydraulischen Aufbau passen. Die Software-Zeitbegrenzung hängt von der laufenden Hauptschleife ab.

### Pumpen

| Ausgang | Zugeordnete Sensoren |
|---|---|
| Boilerpumpe | Kessel und Boiler |
| Heizungspumpe bei kombinierter Raumregelung in Automatik/Nur Heizung | Kessel, Vorlauf, Außen und Raum |
| Heizungspumpe bei Außenregelung oder Restwärme | Kessel, Vorlauf und Außen |
| Heizungspumpe bei Urlaub | Kessel, Vorlauf und Raum |

Bei einem zugeordneten Fehler laufen **nur bereits eingeschaltete Pumpen** bis zu zehn Minuten nach. Ausgeschaltete Pumpen bleiben aus. Nach Ablauf bleibt die betroffene Pumpe gesperrt. Gültige Messwerte geben die Regelung wieder frei.

Dieser Fehlernachlauf ist von den übrigen normalen Pumpenfunktionen zu unterscheiden. Seine tatsächliche Wirkung muss am Anlagenaufbau geprüft werden, einschließlich Restwärmeabfuhr und Wechsel der Regelungsart.

### Heizkurve

Die Berechnung prüft Eingaben und Rechenergebnisse, vermeidet ungültige Divisionen und begrenzt den Vorlaufsollwert auf die eingestellte Maximaltemperatur. Eine ungültige Berechnung erzeugt keine Heizanforderung.

### Grenzen

Fehlersperren decken definierte Softwarefälle ab. Relaiskontaktfehler, falsch angeschlossene Aktoren, elektrische Störungen oder eine mechanisch blockierte Pumpe werden nicht durch eine echte Aktorrückmeldung erkannt. Für die Anlagenbewertung bleiben unabhängige Hardware-Schutzfunktionen und Prüfungen notwendig.

### Restwärmebetrieb und Betriebsart Aus

Der Brenner wird in Aus unabhängig von gespeicherten Wärmeanforderungen gesperrt. Bei warmem oder noch nicht gültig erfasstem Kessel wird die Restwärmephase ausgewählt. Für eine reguläre Wärmeabfuhr müssen Kessel, Vorlauf und Außensensor gültig sein; der Raumfühler ist hierfür nicht erforderlich. Ungültiger Kesselsensor gilt nicht als bestätigte Abkühlung und löst deshalb keine blinde Schließfahrt aus.

In dieser Phase bleibt die vorhandene Vorlaufsensor-Fehlerreaktion aktiv: einmalige 20-Sekunden-Auffahrt, danach Sperre. Die bestehende Pumpenfehlerlogik lässt nur bereits laufende Pumpen begrenzt nachlaufen; ausgeschaltete Pumpen starten bei Sensorfehler nicht. Bei ungültigen Heizkurvenparametern erfolgt keine normale Pumpen-/Mischeranforderung aus der Restwärmeregelung.

Bei gültigem Kesselwert höchstens Restwärmeziel (Standard 35 °C, einstellbar 30–40 °C) endet die geregelte Wärmeabfuhr. Die anschließende zeitlich begrenzte Schließfahrt hat wie zuvor Vorrang vor normaler Initialisierung und Vorlaufsensor-Auffahrt; der Vorlaufsensor ist für diese bereits temperaturfreigegebene Schließfahrt keine zusätzliche Bedingung. Wird der Kessel erneut warm oder ungültig, wird die Schließfahrt abgebrochen und wieder der Restwärmepfad mit seinen Fehlerprüfungen verwendet.

Die Zieltemperatur ist gespeichert, der Standard `MISCHER_AUS_KESSEL_MAX_TEMP` bleibt 35.0f. Die Schließdauer `MISCHER_AUS_ZUFAHRT_MINUTEN` bleibt zwei Minuten. Die Überwachung läuft in Software und ersetzt keine unabhängige Hardwareabschaltung. Details und Testgrenzen: [Geregelte Restwärmeabfuhr in Betriebsart Aus](https://github.com/cperners/heizung/wiki/Restwaermebetrieb).

### Grenzen der normalen Boiler- und Mischerregelung

Die Boilerpumpe erhält im fehlerfreien normalen Ladebetrieb keine Anforderung bei Kesseltemperatur höchstens Boilertemperatur. Der vereinbarte begrenzte Fehlernachlauf und die Pumpenschutzbewegung sind davon getrennte Ausnahmen.

Normale Mischerimpulse haben einen vom LCD-Zähler unabhängigen Zeitstopp und eine konfigurierbare Ruhezone von standardmäßig ±0,5 °C. Die Regelung stoppt normale Stellbewegungen, wenn kein aktiver Heizkreis mehr vorliegt. Sonderfahrten behalten ihre gesonderte Zuordnung und Fehlerreaktion. Ausgänge und echte Bewegung sind weiterhin nicht identisch.

---

## Installation und Updates

### Konfigurationsdateien

- `src/config.h`: Pins, Zeitkonstanten, Fühleroffsets, NTP-Server und weitere Einstellungen.
- `src/secrets.h`: anfängliche WLAN-, MQTT- und Web-Zugangsdaten, lokal und vom Git-Upload ausgeschlossen.
- Auf dem Gerät gespeicherte Web-Passwörter und Netzwerkdaten haben beim Start Vorrang vor den Firmware-Vorgaben.
- `src/secrets.example.h`: Vorlage ohne echte Zugangsdaten.
- `platformio.ini`: Board, Framework und Bibliotheken.

Das aktuelle Ziel verwendet PlatformIO-Umgebung `nodemcuv2`, Board `esp32dev` und Arduino-Framework. Der Umgebungsname ist historisch und bezeichnet hier keinen ESP8266.

Die produktive Kopie verwendet DHCP. Eine DHCP-Reservierung ist hilfreich, wenn ein Reverse Proxy dauerhaft auf denselben ESP32 zeigen soll. Test- und Produktionsgerät dürfen nicht unbeabsichtigt dieselbe Gerätekennung oder die gleichen produktiven MQTT-Steuerthemen verwenden.

### Build

Im Projektordner:

```sh
pio run -e nodemcuv2
```

Das Firmware-Abbild liegt anschließend unter `.pio/build/nodemcuv2/firmware.bin`. Für OTA wird die **BIN-Datei** verwendet, nicht die ELF-Datei. Erst ein erfolgreicher Build bestätigt die Kompilierung; Funktions- und Laufzeittests müssen anschließend am Gerät erfolgen.

Zugangsdaten werden in die Firmware kompiliert. Deshalb gehören auch produktive BIN-Dateien nicht in ein öffentliches Repository oder einen öffentlichen Download.

### OTA-Ablauf

1. Aktuelle Konfiguration und Zugangsdaten der richtigen Projektkopie prüfen.
2. Bekannte funktionierende Firmware für einen Rückweg aufbewahren.
3. Einstellungen dokumentieren, etwa über LCD-Fotos. Die Firmwaredatei ist keine Sicherung der aktuellen Geräteeinstellungen.
4. Neu bauen und den erfolgreichen Abschluss prüfen.
5. Die Update-Seite des Geräts unter `/update` öffnen und die passende BIN-Datei einspielen. Der aktuelle Stand schützt diesen Update-Zugang auf ausdrücklichen Wunsch nicht mit einem Passwort.
6. Neustart abwarten, Webseite neu laden und Fühler, Betriebsart, Regelungsart und Ausgangszustände prüfen.
7. Gespeicherte Einstellungen mit der Sicherung vergleichen.

Die Update-Seite dient zum Einspielen; eine vollständige Sicherung von Flash und Einstellungen ist darüber nicht implementiert. Ein Rückweg setzt voraus, dass das Gerät und seine Update-Seite noch erreichbar sind. Bei einem nicht startenden oder nicht erreichbaren Gerät reicht eine vorhandene alte BIN-Datei allein nicht für eine Fernwiederherstellung.

### Web-Anmeldung und Netzwerkänderungen

Die Web-Anmeldung schützt die Einstellungsformulare mit Ausnahme von Regelung und
Betriebsart. Der Update-Zugang bleibt davon unabhängig offen; ein Passwortwechsel
aktiviert dort keine Anmeldung. Somit kann ein Gerät mit Netz-Zugriff auch ein Update
anstoßen. Bereits gespeicherte Passwörter bleiben beim Upgrade auf bis zu 64 UTF-8-Bytes
zulässige Anmeldepasswörter erhalten. WLAN und MQTT werden nach erfolgreichem Verbindungstest
in Preferences/NVS gesichert. Bedienung, optionale Broker-Zugangsdaten und Rückfall:
[Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk).

### Netzwerkzeit

Die Zeit wird über die native ESP32-Zeitfunktion von drei konfigurierten NTP-Servern bezogen. Nach Neustart wird auf gültige Zeit gewartet; danach läuft die Systemzeit lokal weiter. Zeitprogramme werden vor gültiger Zeit nicht ausgewertet.

Die NTP-Karte fragt die Server zusätzlich einzeln ab und zeigt Abweichung zur ESP32-Uhr, Antwortlaufzeit und Alter der Messung. Diese Diagnoseabfragen stellen die Systemzeit nicht selbst um.

### Lokales HTTPS

HTTPS wird im vorhandenen Aufbau durch einen vorgeschalteten Nginx Proxy Manager auf einer Debian-VM übernommen:

```text
Browser → HTTPS-Proxy → HTTP zum ESP32
```

Der lokale DNS-Name zeigt auf den Proxy. Dessen Weiterleitungsziel bleibt die ESP32-Adresse mit HTTP-Port 80. Der Proxy verwendet ein Zertifikat einer lokalen Zertifizierungsstelle. Clientgeräte müssen dem öffentlichen Stammzertifikat vertrauen. Der private Schlüssel der Zertifizierungsstelle bleibt geschützt auf dem Server.

Die Verschlüsselung endet am Proxy; die Verbindung vom Proxy zum ESP32 bleibt HTTP. Zertifikatsvertrauen fügt keine Benutzeranmeldung zur Heizungsseite hinzu. Zertifikate müssen vor Ablauf erneuert und private Schlüssel gesichert werden.

### Nach dem Update

Neue Oberflächenbestandteile können durch Browser-Caches verzögert erscheinen, insbesondere das Favicon. Die Webseite neu öffnen und bei Bedarf Browserdaten gezielt aktualisieren. Vorher prüfen, ob tatsächlich die richtige Firmware und die richtige Geräteadresse verwendet wurden.

### Wiederkehrende OTA-Abbrüche

Im Betrieb traten wiederholt `Update failed – No Error` und Verbindungsabbrüche auf, auch ohne HTTPS-Proxy. Spätere Versuche waren erfolgreich. Eine beschädigte BIN-Datei wurde dabei nicht nachgewiesen; die lokal geprüften Bilder hatten gültige Prüfsummen und interne Hashes. Die Ursache ist weiterhin offen.

Ein möglicher Codepfad ist ein bereits geöffneter Update-Vorgang: Das Framework kann einen erneuten Start ohne gesetzten Fehlercode ablehnen. Das wurde als Möglichkeit gefunden, am Gerät aber noch nicht bewiesen.

Die neue OTA-Diagnose ist auf der Hauptseite unter Systemdiagnose, über `/ota-status` und im MQTT-Objekt `ota` zugänglich. Sie beobachtet Start, Fortschritt und Abschluss. Nach erfolgreichem Neustart beginnt die Beobachtung neu. Kein Fortschritt nach 30 Sekunden ist eine Diagnose, keine automatische Abbruchanweisung. Weitere Uploadversuche nicht parallel starten; Fehlermeldung, Diagnose und Erreichbarkeit festhalten, bevor neu gestartet wird.

### Fester OTA-Dateiname

Die bereitgestellte aktuelle Produktionsdatei heißt immer **heizung-produktion-ota.bin** und wird bei einem neuen Build aktualisiert. Keine wechselnden Funktionsnamen sind für den normalen Upload nötig. Die Datei enthält die lokal eingebauten Zugangsdaten und wird nicht ins öffentliche Repository hochgeladen. Gerätetests sind nach jedem relevanten Update separat erforderlich.

---

## Architektur

### Datenfluss

```text
Sensoren / MQTT / Webeingaben
              ↓
       Hauptschleife
              ↓
  Messwertprüfung und Regelungslogik
              ↓
 Sensorsperren / Pumpennachlauf / Mischer
              ↓
      Geschaltete Ausgänge
              ↓
 LCD / MQTT-Status / Web-Status-Snapshot
```

Die Darstellung vereinfacht den Ablauf. Netzwerk-Callbacks und NTP-Diagnose haben eigene Ausführungskontexte. Einstellaufträge werden gezielt an die Hauptschleife übergeben, um direkte konkurrierende Änderungen zu reduzieren.

### Quellcodeaufteilung

`src/main.cpp` bindet Bibliotheken, öffentliche Schnittstellen und Firmwareabschnitte ein.

| Eigenständiges Modul | Aufgabe |
|---|---|
| `src/web/page.cpp` | HTML, CSS und JavaScript |
| `src/diagnostics/cpu_load.cpp` | Grobe CPU-Schätzung je Kern |
| `src/ntp_diagnostics/ntp_diagnostics.cpp` | Separate NTP-Diagnose |

Die Dateien unter `src/firmware/` sind `.inc`-Implementierungsabschnitte: `shared_state`, `runtime_support`, `web_access`, `wifi_settings`, `mqtt_settings`, `setup`, `loop`, `web_status`, `web_handlers`, `network`, `mqtt`, `clock`, `uptime`, `sensor_reading`, `sensor_bus`, `heating`, `mixer`, `lcd_menu`, `lcd_display` und `serial_console`.

Sie werden in derselben Übersetzungseinheit eingebunden und teilen weiterhin globale Variablen. Die Struktur trennt Zuständigkeiten sichtbar, ist aber noch keine vollständig unabhängige Modularchitektur. `.inc`-Dateien dürfen nicht einfach zusätzlich als `.cpp` kompiliert werden.

### Webstatus und Eingaben

Die Hauptschleife erzeugt ungefähr einmal pro Sekunde einen JSON-Snapshot. Der Webserver liefert dessen Kopie aus. Die Webseite fragt diesen einmal pro Sekunde ab; eine noch laufende Anfrage verhindert eine zweite gleichzeitige Anfrage.

Wunschtemperaturen und Zeiten werden in der Hauptschleife gespeichert. Eine Anfragekennung ordnet die Speicherbestätigung zu. Änderungen an Regelungsart und Kachelofen-Schaltschwellen werden ebenfalls kontrolliert übergeben. Die Speicherbestätigung der Wunschtemperaturen ist nicht mit der MQTT-Befehlsantwort gleichzusetzen.

### CPU-Schätzung und Laufzeit

Die Laufzeit im Web basiert auf einem 64-Bit-Zeitgeber. Die CPU-Anzeige beobachtet Leerlauf je Kern und mittelt über fünf Sekunden. Sie zeigt Abschnitte ohne beobachteten Leerlauf, keine exakte CPU-Zeitmessung. Kurze Rechenarbeit innerhalb eines Abschnitts kann unterschätzt werden. Die Framework-Laufzeitstatistik ist in diesem Build nicht aktiviert.

### Speicherung

Temperaturen, Zeitprogramm und weitere Bestandsparameter werden in EEPROM-emulierter Speicherung abgelegt. Bei ungültigen gespeicherten Einstellungen werden vorhandene Standardwerte verwendet. Die Tagesbasis des Gaszählers verwendet Preferences. Nicht jede historische MQTT-Einstellung hat dieselbe geprüfte Speichertransaktion wie das neue Webformular.

### Bibliotheken

| Bibliothek | Konfiguration |
|---|---|
| ElegantOTA | 4.0.0, asynchroner Betrieb |
| ESPAsyncWebServer | 3.12.1 |
| AsyncTCP | 3.5.0 |
| ArduinoJson | 7.4.3 |
| OneWire | 2.3.8 |
| AsyncMqttClient | Bereich `^0.9.0` |
| Keypad, Ticker | In PlatformIO ohne feste Versionsangabe |
| LCD-/I2C-Tastaturbibliotheken | Lokale Dateien mit bestehenden Hardwareanpassungen |

Diese Angaben beschreiben die Projektkonfiguration, nicht eine Aussage über die aktuell neueste Veröffentlichung jeder Bibliothek. Ungebundene Versionsangaben können bei einer Neuinstallation andere Versionen ergeben.

### Ergänzte Laufzeitdiagnose

`src/firmware/runtime_diagnostics.inc` erfasst Speicherzustand und Schleifenabstände. Die Hauptschleife aktualisiert die Werte; Webstatus und MQTT lesen diese Diagnose. Speicherwerte werden ungefähr einmal pro Sekunde mit derselben Heap-Kategorie `MALLOC_CAP_8BIT` abgerufen.

Der nicht blockierende Aus-Zustand in `mixer.inc` entscheidet vor der normalen Regelung zwischen geregelter Restwärmeabfuhr und abschließender Schließfahrt. `kein_Betrieb()` hält Wärmeanforderungen und Brenner aus; `runOffHeatRecovery()` verwendet die Heizkurve unabhängig von der ausgewählten Raum-/Außenregelung. Die Heizkurvenregelung bei Raum-/Außen-Heizbedarf und die Restwärme verwenden denselben Mischerimpulsregler. Eine zentrale Ausgangssperre verhindert Brenneranforderungen in Aus.

Der Kachelofen-Timeout ist aktuell noch als 21-Minuten-Konstante in `shared_state.inc` definiert. Eine spätere Auslagerung nach `config.h` wurde vorgemerkt und ist noch nicht umgesetzt.

### Optimierung von Webausgabe und MQTT

`web_page_response.inc` erzeugt eine Antwort mit festgehaltenen Platzhalterwerten. `src/web/template_stream.h` liefert die HTML-Vorlage abschnittsweise aus. Es werden ausschließlich bekannte Tokens ersetzt; Prozentzeichen in CSS, JavaScript und Werten bleiben erhalten. Eine vollständige HTML-String-Kopie pro Anfrage entfällt.

Webstatus und MQTT behalten ihre JSON-Dokumente und Variant-Speicherpools zwischen Zyklen. Bei einem Allokationsfehler wird das jeweilige Dokument zur Erholung zurückgesetzt. Das verringert Speicherbewegungen, ist aber kein Beweis, dass die Langzeitverzögerung behoben ist. Die dauerhaft gehaltenen Pools verändern die Speicher-Ausgangswerte gegenüber der Vorversion.

`ota_diagnostics.inc` beobachtet die ElegantOTA-Callbacks und ergänzt in der Hauptschleife Fehler-/Fortschrittsinformationen. Ein eigener Mutex schützt die Momentaufnahme. Es werden keine automatischen Abbrüche oder zusätzlichen Neustarts ausgelöst. Die Heizungs- und Fehlersperrlogik wurde durch diese Optimierung nicht verändert.

Betriebsarten verwenden für Web und alle sechs MQTT-Modusbefehle `applyOperatingMode()`. Die älteren LCD-Einstellwege sind damit nicht vollständig vereinheitlicht. Die Struktur bleibt eine Mischung eigenständiger Module und gemeinsam eingebundener Implementierungsabschnitte.

### Web-Einstellungen und Ausgangssymbole

Boiler-Solltemperatur/Einschaltdifferenz und Vorlaufmaximum/Kurvensteilheit werden als getrennte, begrenzte Anfragen unter dem vorhandenen kritischen Abschnitt übernommen. Die Hauptschleife speichert die jeweilige Einstellungsgruppe zuerst und ändert die Laufzeitwerte nur bei Erfolg. Bei Fehler wird der EEPROM-Puffer auf die bisherigen Werte zurückgesetzt. Anfragen nutzen eindeutige Kennungen und den bestehenden Ergebnisring mit acht Plätzen; offene Ergebnisse werden nicht überschrieben. Alte Clients können die zusätzlichen Parameter weglassen.

Die Boiler-Differenz wird nur mit zwei gültigen Sensoren angezeigt. Ausgangssymbole verwenden die vorhandenen fünf Ausgangszustände der gemeinsamen Web-Statusabfrage. Die Symbole sind lokal eingebettete SVGs; zusätzliche Icon-Abfragen werden nicht benötigt.

`updateMischerBoilerMode()` betreut in jeder Schleife die einmalige Boiler-Schließfahrt. Nach Abschluss der ersten Fahrt werden Referenztemperatur und Wartebeginn gespeichert. Nach zehn Minuten entscheidet ein einmaliger Vergleich über genau eine Nachfahrt. Der Abschluss bleibt bis zum nächsten Eintritt gespeichert, damit ein Timerüberlauf keine neue Fahrt startet. Beim Übergang nach Aus bleiben dessen bereits gesetzte Ausgänge erhalten. Sensorfehlerlogik hat Vorrang.

### Gespeicherte Restwärmetemperatur

Die Adresse `EEADDRESS_RESTWAERME_ZIEL` liegt bei 172; ein Float passt damit in den bereits bestehenden EEPROM-Bereich von 176 Bytes. Bestehende Felder werden nicht verschoben. Ein unbekannter oder ungültiger neuer Wert wird auf den Standard 35 °C gesetzt. Das Vorlaufformular übergibt den optionalen Parameter `restwaerme`; die Hauptschleife speichert ihn zusammen mit Vorlaufmaximum und optionaler Kurvensteilheit. Erst ein erfolgreicher Commit aktiviert die Werte. Bei Fehler wird auch der bisherige Restwärmewert im EEPROM-Puffer wiederhergestellt. Alte Clients ohne diesen Parameter ändern ihn nicht.

### WLAN-Wiederanlauf und Neustartdiagnose

Ein erfolgreicher IP-Bezug löscht den ausstehenden WLAN-Wiederverbindungsauftrag. Ein verspäteter Auftrag trennt keine bereits erholte Verbindung. Verbindungsaufrufe erfolgen in der Hauptschleife; Versuche haben mindestens 15 Sekunden Abstand und bleiben auch nach ausbleibenden Treiberevents möglich.

Die Diagnose zählt WLAN-Abbrüche und Wiederanlaufversuche und protokolliert den letzten Abbruchgrund. Die Watchdog-Einrichtung erfasst Rückgabewerte und den tatsächlichen Registrierungsstatus; der konfigurierte Timeout bleibt 200 Sekunden. RTC-Speicher hält eine zuletzt markierte grobe Programmphase für geeignete Software-/Watchdog-Neustarts vor, nicht zuverlässig nach vollständigem Stromausfall. Sie ist kein Stacktrace und kein Beweis für die Fehlerursache. Webstatus und MQTT enthalten diese Angaben.

### Raumregelung mit Heizkurve

`controlSpaceHeating()` ruft bei Raumregelung `chckRoom(kesselReady)` auf. Das Argument erlaubt oder verbietet einen neuen Pumpenstart, ohne die Auswertung eines bereits aktiven Heizkreises zu überspringen. Raum-Sollwert und Hysterese bestimmen den Pumpenzustand; ein laufender Heizkreis regelt den Vorlauf mit der Heizkurve auch unterhalb der bisherigen Kesselfreigabe. In Automatik und Nur Heizung wird die Kurve deshalb auch bei gewählter Raumregelung berechnet und validiert. Urlaub behält den bisherigen Sonderweg.

`heatingPumpSensorFault()` ordnet der kombinierten Raumregelung Kessel-, Vorlauf-, Außen- und Raumsensor zu. Bei Außenregelung und Restwärme wird der Raumfühler nicht zusätzlich benötigt. Bei ungültigen Heizkurvenparametern wird eine normale Pumpenanforderung beendet; Sensorfehlernachlauf bleibt zentral behandelt.

### Zuordnung und Frist normaler Mischerimpulse

`startRegulatedMischerPulse()` merkt Startzeit und aktive Zuordnung eines normalen Impulses. `serviceRegulatedMischerPulse()` wird unabhängig vom Regelungszweig in jeder Hauptschleife aufgerufen. Er beendet den Impuls nach `MISCHER_DRIVE * 1000` Millisekunden oder bei fehlendem aktivem Heizkreis. Die Differenz wird mit 32-Bit-Arithmetik gebildet, damit ein Millisekundenüberlauf korrekt behandelt wird.

`regulateMischerToTarget()` beendet normale Fahrten innerhalb der konfigurierten Ruhezone oder bei umgekehrter erforderlicher Richtung. Neue Impulse starten erst nach dem vorhandenen Wartezähler. `MischerAuf()`, `MischerZu()` und `MischerStop()` übernehmen beziehungsweise löschen die alte Impulszuordnung. Zustandswechsel nach Aus und Sensorfehlerfahrten löschen sie ebenfalls; dadurch beendet der normale Zeitdienst keine längere Sonderfahrt.

### Boiler-Ladefreigabe und Webanzeige

`SetOutPin()` prüft die normale Boileranforderung bei jedem Durchlauf gegen die aktuelle Kessel-Boiler-Differenz. Bei fehlerfreien Sensoren und Kesseltemperatur höchstens Boilertemperatur wird die Anforderung verworfen. Die nachgeschaltete bestehende Sensorfehler- und Pumpenschutzlogik bleibt erhalten.

`vorlaufCalculationDisplay()` liefert den berechneten Vorlauf-Sollwert und die Differenz Ist minus Soll als getrennte Texte. Initiale HTML-Platzhalter und gemeinsame Webstatusfelder werden gleichermaßen bedient. Bei deaktivierter Berechnung, ungültigem Ergebnis oder erforderlichem Sensorfehler erscheinen Hinweise statt irreführender Zahlen. Die zusätzlichen Felder `vorlauf-soll` und `vorlauf-diff` werden über die vorhandene Webstatusabfrage aktualisiert; zusätzliche Browseranfragen sind nicht nötig.

### Web-Anmeldung und gespeicherte Netzwerkprofile

`web_access.inc` verwaltet vier RAM-Sitzungen und die Passwortänderung. Schreibende
Web-Routen prüfen einen Sitzungsschlüssel; die HTML-Sperre ist nur die zusätzliche
Bedienungsebene. Login und Auftragsübergaben laufen im Web-Callback; NVS-Speicherung
und Übernahme des Passworts erfolgen in der Hauptschleife. Das unveröffentlichte
Anfangspasswort ist nach Speicherung nicht mehr maßgeblich. Alte 9-Byte-Passwortdatensätze
werden in die erweiterte Kapazität übernommen.

`wifi_settings.inc` und `mqtt_settings.inc` besitzen getrennte Preferences-Namensräume.
WLAN-Daten werden erst nach stabiler Verbindung, MQTT-Daten nach erfolgreicher
Broker-Verbindung als vollständiger Datensatz gespeichert. Fehler führen zur
bisherigen Konfiguration zurück. MQTT-Callbacks verwenden eine konsistente Momentaufnahme
der Empfangsthemen; während des Wechsels werden Befehle verworfen. Eine Themengeneration
löst die erneute Veröffentlichung unveränderter Sensor-/Sperrstatuswerte aus.

Die neue MQTT-Datensatzversion ergänzt optionale Zugangsdaten und liest weiterhin
die vorherige Version ohne diese Felder. Passwörter werden nicht an den Browser
zurückgesendet. NVS-Verschlüsselung wird durch diese Funktion nicht eingerichtet.
Der Zugriffsschutz erstreckt sich ausdrücklich nicht auf ElegantOTA. Details:
[Anmeldung und Netzwerk](https://github.com/cperners/heizung/wiki/Anmeldung-und-Netzwerk).

---

## Tests und Fehlersuche

### Bestätigte Funktionstests

Der Betreiber hat im bisherigen Verlauf unter anderem bestätigt:

- Weboberfläche, MQTT-Empfang und Update-Seite erreichbar.
- Fünf angeschlossene Fühler liefern wieder Werte nach Umstellung der Gerätekennung auf direkte Einzelabfrage.
- Betriebsarten-Auswahl funktioniert.
- Wunschtemperaturen und Kachelofen-Schaltschwellen sind bedienbar; für Schaltschwellen wurde Erhalt nach Neustart bestätigt.
- Tag- und Nachtzeiten sind nach Korrektur der Platzhalter vorbefüllt.
- Mobile Kopfzeile passt auf dem iPhone.
- LCD-Darstellung ohne Wochentag passt.
- Kachelofen-Timeout fällt nach ausbleibenden Meldungen auf inaktiv und automatische Raumregelung zurück.
- Temperaturtrendpfeile passen am Gerät.

Diese Rückmeldungen sind Funktionsprüfungen, keine vollständige Abnahme aller Fehler- und Anlagenzustände.

### Empfohlene Prüfungen

#### Nach einer Firmwareänderung

1. Neustart ohne wiederholte Bootschleife prüfen.
2. Alle fünf Sensorwerte auf Plausibilität prüfen.
3. Webstatus und MQTT-Historie mehrere Minuten beobachten.
4. Regelungs- und Betriebsart gezielt umschalten und Anzeige vergleichen.
5. Wunschtemperaturen und Zeiten speichern; nach Neustart auf Erhalt prüfen.
6. LCD-Menü öffnen und zurückkehren: vier vollständige Zeilen ohne Zeichenreste.

#### Trendanzeige

- Bei zwei neuen steigenden Messungen Pfeil hoch prüfen.
- Bei zwei fallenden Messungen Pfeil runter prüfen.
- Bei nahezu gleichem Wert waagrechten Pfeil prüfen.
- Wiederholte Webabfragen ohne neue Messung dürfen den Trend nicht zurücksetzen.
- Bei ungültigem oder zu altem Messwert darf kein Trend stehen.

#### Langzeittest

Über mehrere Tage Laufzeit, Neustarts, MQTT-Aktualisierung, Webreaktion und Fühlerfehler dokumentieren. Die grobe CPU-Schätzung allein beweist keine ausreichenden Reserven; dynamischer freier Heap und längere Blockierungen sind damit nicht vollständig erfasst. Das Projekt hat noch keine automatisierte vollständige Langzeitmessung.

### Typische Probleme

| Beobachtung | Prüfschritt |
|---|---|
| JSON im MQTT Explorer bleibt scheinbar alt | Suchfilter entfernen, Thema und Historie prüfen; Retain und aktive Verbindung unterscheiden |
| Regelungsmodus im JSON passt nicht | `atRegelung`: 0 Raum, 1 Außen, 2 Automatik; richtige Firmware prüfen |
| Web zeigt Nicht erreichbar/graue LEDs | Netzwerk, Geräteadresse und Statusabfrage prüfen |
| Sensorfehler | Separates MQTT-Sensorthema und bei Boiler die Diagnose ansehen |
| Boiler-Datenblock nur 255 | Leseweg, Verdrahtung und Sensorantwort untersuchen; Fehlerprüfung nicht einfach abschalten |
| HTTPS-Name nicht auffindbar | Lokalen DNS-Eintrag auf Proxy prüfen; VPN kann lokale Namensauflösung stören |
| HTTPS-Verbindung abgelehnt | Proxyadresse, HTTPS-Port und Container prüfen, bevor Zertifikatvertrauen geändert wird |
| Zertifikatswarnung | Name im Zertifikat, Ablauf und Vertrauen der lokalen Zertifizierungsstelle prüfen |
| Chrome scheitert, Safari funktioniert | Lokale Netzwerkfreigabe und browserbezogene DNS-/Verbindungseinstellungen prüfen |

### Bekannte Grenzen und offene Arbeiten

- Eine vollständige Fehlerfallprüfung aller Sensor-/Pumpen-/Mischerkombinationen fehlt.
- MQTT hat einen einzelnen Antwortpuffer; mehrere schnelle Befehle erhalten keine garantierten Einzelbestätigungen.
- JSON-Temperaturen haben keine eigene Sensor-Gültigkeitskennzeichnung.
- Das Tagintervall unterstützt keinen Verlauf über Mitternacht und besitzt strenge Beginn-/Endgrenzen.
- Historische relative MQTT-Befehle sind nicht generell gegen doppelte QoS-1-Zustellung abgesichert.
- Teilbereiche der Firmware teilen weiterhin globale Daten.
- Die CPU-Anzeige ist eine Schätzung, keine exakte Messung.

Änderungen sollten einzeln gebaut, am Testgerät geprüft und erst dann am aktiven System verwendet werden. Beim Fehlertest am aktiven Heizsystem muss die Wirkung auf reale Ausgänge berücksichtigt werden.

### Restwärmeabfuhr bei Aus

Der frühere erste Gerätetest bezog sich auf die alte Warte-/Schließlogik. Die neue geregelte Wärmeabfuhr mit einstellbarer Zieltemperatur ist lokal geprüft, aber noch nicht vollständig am Gerät bestätigt.

1. Bei warmem Kessel von Automatik oder Nur Heizung auf Aus wechseln: Brenner aus, Heizungspumpe an und Vorlauf nach Heizkurve regeln.
2. Vorlaufmaximum und Verhalten bei bereits erreichter Raumtemperatur kontrollieren.
3. Bei höchstens gespeichertem Ziel (Standard 35 °C) muss die reguläre Pumpenanforderung enden und die zweiminütige Schließfahrt beginnen.
4. Danach beide Mischer-Ausgänge aus; bei Wiedererwärmung wieder geregelte Wärmeabfuhr.
5. Zieltemperatur innerhalb 30–40 °C speichern und nach Neustart auf Erhalt prüfen.
6. Betriebsartwechsel muss den Aus-Ablauf abbrechen. Sensorfehlerfälle nur unter kontrollierten Anlagenbedingungen prüfen.

Die Hosttests verwenden tatsächliche Quellfunktionen mit simulierten Sensoren, Ausgängen und EEPROM. Sie prüfen Grenzen bei 30, 35 und 40 °C, Brennersperre, Heizkurve trotz vorheriger Raumregelung, Vorlaufbegrenzung, Mischerimpulse, Schließdauer, Wiedererwärmung und Sensorfehler. Die Speicherprüfung deckt ungültige Werte, unveränderte Einstellungen, Altclients, fehlgeschlagene Commits und die Wiederherstellung des bisherigen Puffers ab. Speicher-/Ablauftests verwenden AddressSanitizer und UndefinedBehaviorSanitizer. Kein Test bestätigt die reale Hydraulik oder Relaisfunktion.

Ausführen im Projekt: `python3 tests/host/test_off_heat_recovery.py`, `python3 tests/host/test_recovery_setting.py`, `python3 tests/host/test_space_heating_gate.py` und `python3 tests/host/test_network_recovery.py`.

### Laufzeitvergleich bei langsamem Webserver

Die erste vom Betreiber gemeldete Diagnose nach etwa drei Minuten Laufzeit enthielt 133260 Bytes freien Heap, 68600 Bytes Minimum und 110580 Bytes als größten Block. Das letzte Schleifenmaximum lag bei 12,674 ms, das bisherige Maximum bei 285,588 ms mit vier Abständen über 100 ms. Das sind Ausgangswerte, kein nachgewiesener Fehler.

Beim nächsten Langsamwerden Laufzeit und dieselben Diagnosewerte vergleichen. Zuvor war die Seite nach knapp sechs Tagen langsam, MQTT kam weiterhin an und die Geräte-Laufzeit lief weiter. Eine konkrete Ursache ist bislang nicht nachgewiesen; die Diagnose stellt noch keine Behebung dar.

### Letzte Optimierungen: Prüfumfang und offene Gerätetests

Erfolgreich durchgeführt wurden der Firmware-Build, Prüfungen der echten MQTT-Modusfunktionen bei erfolgreichem/fehlgeschlagenem Speichern und Wiederholungen, Vergleich der kompletten HTML-Ausgabe bei Puffergrößen von 1 bis 1460 Bytes, Prozentzeichen-/Leerwertfälle, OTA-Callback-/Fehlerfälle sowie JavaScript-Syntaxprüfungen.

Ein lokaler Test der ArduinoJson-Speicherwiederverwendung über 10000 Zyklen zeigte nach der Aufwärmphase stabilen belegten Speicher und weniger Allokationen. Das war ein lokaler Bibliothekstest, keine 10000-Zyklen-Messung am ESP32.

Nach Upload der neuesten Optimierungen noch prüfen:

1. Hauptseite vollständig auf Desktop und Mobilgerät laden; Uhrzeit, Trends und Werte müssen weiter aktualisiert werden.
2. Betriebsarten über Web und MQTT umschalten; Urlaub separat gemäß [Urlaubsbetrieb](https://github.com/cperners/heizung/wiki/Urlaubsbetrieb) prüfen.
3. Neue Heap-Ausgangswerte nach dem Start notieren und beim nächsten Langsamwerden vergleichen.
4. Bei einem späteren OTA-Fehler die Zeile OTA oder `/ota-status` prüfen; Diagnose wird bei Neustart zurückgesetzt.

Die letzte bestätigte Auswahlreihenfolge lautet Aus, Automatikbetrieb, nur Heizung, nur Boiler, Urlaub. Der Gerätetest der neuesten Speicher-/OTA-Optimierungen und ein dokumentierter mehrtägiger Vergleich stehen noch aus.

### Neue Web-Einstellungen und Mischer-Schließfahrt

Lokal erfolgreich geprüft: Firmware-Build, JavaScript-Syntax, vollständige HTML-Platzhalterausgabe, Übernahme beider Wertepaare bei erfolgreichem Speichern, Beibehalten der bisherigen Werte und Puffer bei Speicherfehler, Grenzwerte und nicht endliche Eingaben, Statusfarben und Zuordnung der fünf Ausgänge. Ein Host-Ablauftest prüft die Boiler-Schließfahrt vor/bei Ablauf von 120 Sekunden, dauerhaftes Stoppen, erneuten Eintritt, Betriebsartwechsel, Vorrang der Sensorsperre und Übergang nach Aus. Dies ersetzt keinen Gerätetest.

Nach dem Upload noch gezielt prüfen:

1. Alle fünf Temperatur-Kacheln: Bezeichnung, Temperatur und dann Ausgangssymbole; keine separate Ausgangsstatus-Kachel.
2. Boiler-Sollwert und Einschaltdifferenz gemeinsam speichern und nach einem Neustart vergleichen.
3. Vorlaufmaximum und Kurvensteilheit gemeinsam speichern und nach Neustart vergleichen.
4. Kessel-minus-Boiler-Anzeige mit Vorzeichen und Pastellfarben prüfen; Messwerte bei Sensorfehler nicht als gültige Differenz darstellen.
5. Nur Boiler wählen: einmalige Schließfahrt, nach zwei Minuten beide Ausgänge aus; Betriebsartwechsel muss abbrechen.

Die neuen Funktionen sind noch nicht vollständig am Gerät bestätigt. Die Nachprüfung ist lokal getestet: 120 Sekunden erste Fahrt, 600 Sekunden Wartezeit ab Fahrtende, keine Nachfahrt bei sinkender Temperatur, genau eine weitere 120-Sekunden-Fahrt bei gleichem oder höherem Vorlauf, keine weitere Wiederholung, Abbruch bei Betriebsartwechsel und keine Nachfahrt bei ungültigem Messwert oder verworfener Referenz. Die lokale Prüfung deckt auch das Verhindern einer erneuten Fahrt nach Timerüberlauf bei bereits abgeschlossenem Ablauf ab.

Am Gerät zusätzlich prüfen: Referenztemperatur beim ersten Fahrtende notieren, nach zehn Minuten mit dem Vorlauf vergleichen und die gegebenenfalls folgende zweite Schließfahrt beobachten. Dieser Ablauf ist noch nicht am Gerät bestätigt.

### Unbeabsichtigter Wechsel nach ioBroker-Neustart

Beim manuellen Neustart von ioBroker wurde auf dem Heizungs-Steuerthema `setheizung` beobachtet. Nach Deaktivieren von **Eigene States beim Verbinden publizieren** im MQTT-Client-Adapter blieb die Heizung bei wiederholtem Neustart in Aus und es kam kein neuer Steuerbefehl. Dies bestätigt die Start-Veröffentlichung für den beobachteten Fall. Die Option betrifft alle eigenen States der betreffenden Adapterinstanz; normale spätere Aktualisierungen bleiben möglich. Der nächste reguläre Proxmox-Backup-Lauf war noch zu beobachten.

Das ist von dem früheren IP-Ausfall nach dem Ausschalten zu trennen. Dessen Ursache ist nicht abschließend bewiesen. Die ergänzte WLAN-Korrektur und Wiederanlaufdiagnose liefern Gegenmaßnahmen und weitere Messwerte, aber keinen nachträglichen Absturzbeweis.

### Raumzielprüfung bei kaltem Kessel

Die lokalen Tests in `tests/host/test_space_heating_gate.py` decken zusätzlich ab: laufende Pumpe bei weiterem Wärmebedarf bleibt angefordert, stehende Pumpe startet ohne Kesselfreigabe nicht, Erreichen beziehungsweise Überschreiten des Tag-/Nacht-Sollwerts beendet die Anforderung, im Hystereseband bleibt der bisherige Zustand erhalten, und ungültige/nicht endliche Messwerte lösen keine falsche Raumzielentscheidung aus. Build und lokale Prüfungen waren erfolgreich.

Der Betreiber meldete nach dem Einspielen einen zunächst passenden Eindruck. Ein vollständig dokumentierter Gerätetest aller Grenzfälle steht noch aus. Am Gerät sind insbesondere Abschalten am Raum-Sollwert und Wiederanlauf bei erneuter Wärmeanforderung mit Kesselfreigabe zu prüfen.

### Kombinierte Raumregelung, Mischerimpulse und Boilerprüfung

Aktuell lokal geprüft sind:

- Raumwärmebedarf mit erlaubtem/gesperrtem Pumpenneustart, Weiterbetrieb bei kaltem Kessel, Tag-/Nacht-Sollwert, Hystereseband, Pumpenstopp und normale Mischerabschaltung am Raum-Sollwert.
- Vorlaufregelung bei Raumwärmebedarf mit gültigem Außenfühler, notwendige Sensorzuordnung und Unterdrückung normaler Anforderungen bei ungültiger Kurve.
- Normale Mischerfrist vor/bei fünf Sekunden ohne LCD-Ticks, beide Ruhezonengrenzen und exakter Sollwert, frühes Stoppen bei Sollwerteintritt/Richtungswechsel sowie 32-Bit-Zeitüberlauf.
- Getrennte Initialisierungs-, Aus-, Nur-Boiler- und Sensorfehlerfahrten: Der normale Impulsdienst verkürzt ihre längere Fahrzeit nicht.
- Boilerpumpenanforderung bei negativem beziehungsweise gleichem Kessel-Boiler-Temperaturverhältnis, auch ohne erneuten Aufruf der Ladeprüfung; der vereinbarte Fehlernachlauf bleibt erhalten.
- Berechnete Vorlaufanzeige und Differenzvorzeichen sowie Hinweise bei inaktivem/ungültigem Zustand; Web-JavaScript und vollständige Platzhalterliste.

Die lokalen Prüffälle verwenden tatsächliche Quellfunktionen mit simulierten Sensoren, Zeit und Ausgängen. Sie beweisen keine optimale Abstimmung des Mischermotors auf die reale Hydraulik. Die fünf Sekunden Fahrzeit und der 30-Sekunden-Abstand bleiben vorhandene Parameter; ihre optimale Einstellung erfordert reale Stellzeit und Temperaturverläufe.

Zusätzlich zu den bisherigen Hosttests: `python3 tests/host/test_mixer_pulses.py` und `python3 tests/host/test_vorlauf_display.py`.

Nach Upload noch am Gerät prüfen: Pumpenstart mit Kesselfreigabe bei Raumwärmebedarf, Vorlaufnachführung, Ruhe am Sollwert, Pumpen- und Mischerstopp bei erreichtem Raum-Sollwert, Verhalten beim Wechsel der Regelungsart und Ende der normalen Boilerladung bei nicht positivem Temperaturunterschied. Die neuesten kombinierten Funktionen sind noch nicht vollständig am Gerät bestätigt.

### Anmeldung, WLAN, MQTT und offener Update-Zugang

Am Gerät vom Betreiber bestätigt:

- Änderung des Web-Anmeldepassworts und anschließende Anmeldung.
- Zugriff auf die ausdrücklich wieder ungesperrte Update-Seite.

Lokal geprüft sind außerdem Zugriff ohne/mit Sitzung, Abmeldung und Ablauf nach 15
Minuten, Timerüberlauf, Passwort-Speicherfehler, alte achtstellige Datensätze,
Anmeldung und Speicherung bei 64 Bytes sowie Ablehnung bei 65 Bytes. Die Eingabesperre
und die freien Umschalter wurden mit einer DOM-Simulation geprüft. Diese Prüfung ist
kein vollständiger realer Browsertest.

Die WLAN-/MQTT-Tests verwenden die tatsächlichen Quellfunktionen mit simulierten
HTTP-, Treiber- und NVS-Schnittstellen. Sie decken den stabilen WLAN-Verbindungstest,
Rückfall, Neustart mit alten gespeicherten Daten, optionale Broker-Anmeldung, bewusste
anonyme Verbindung, Passwortbeibehaltung, ungültige Eingaben, Datensatzmigration und
Rückfall aller MQTT-Werte bei fehlender Verbindung oder Speicherfehler ab. Alte
Steuerthemen werden nach Wechsel nicht mehr angenommen.

Ausführung im Projektordner:

```sh
python3 tests/host/test_web_access.py
node tests/host/test_web_access_ui.js
```

Noch am Gerät zu dokumentieren: lange Passwörter einschließlich Neustart,
WLAN-Wechsel mit neuer IP und bewusst falschen Daten, MQTT-Wechsel mit/ohne gültige
Broker-Anmeldung, Bestätigung der neuen Themen und erneuter Kachelofen-Empfang.
Die Heizungsprüfungen der vorherigen Version bleiben weiterhin relevant.
Die frühere Ablehnung des geänderten Passworts an der OTA-Anmeldung bleibt als Ursache
offen; der aktuelle Stand nimmt den Update-Zugang bewusst vom Schutz aus.
