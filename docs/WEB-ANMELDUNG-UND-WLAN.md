# Web-Anmeldung und WLAN-Wechsel

Die Anzeige aller Messwerte, Sollwerte und Diagnosen bleibt frei erreichbar. Die
beiden Umschalter **Regelung** und **Betriebsart** bleiben ohne Anmeldung bedienbar.
Alle anderen Web-Einstellungen erfordern eine Anmeldung: Wunschtemperaturen und
Zeiten, Boiler-Einstellungen, Vorlauf/Heizkurve/Restwaermeziel, Kachelofen-Schwellen,
WLAN-Zugangsdaten, MQTT-Server/Port/Basispfad und Anmeldepasswort. MQTT-Befehle und LCD-Bedienung sind hiervon nicht betroffen.

## Anmeldung

Benutzername und Passwort werden lokal in `src/secrets.h` mit
`HEIZUNG_WEB_USER` und `HEIZUNG_WEB_PASSWORD` konfiguriert. Diese Datei und die
fertige Firmware werden nicht veroeffentlicht. Das Anmeldepasswort darf
maximal 64 UTF-8-Bytes umfassen. Bereits gespeicherte achtstellige Passwoerter bleiben gueltig. Fuer fremde Builds sind eigene Werte
erforderlich; es wird kein allgemeines Standardpasswort eingebaut.

Die Anmeldung steht ueber den Kacheln. Die Formulare sind bereits im ausgelieferten
HTML gesperrt. Der Server prueft jede geschuetzte Speicheranfrage zusaetzlich, auch
wenn jemand die Oberflaechensperre umgehen sollte. Kachelofen-Aenderungen verwenden
POST statt des bisherigen veraendernden GET-Aufrufs. Der alte WebSocket-Toggle fuer
die Heizungspumpe wird nicht mehr registriert und veraendert auch intern keinen
Ausgang mehr.

Pro Browser-Tab wird ein zufaelliger Sitzungsschluessel verwendet, im
Sitzungsspeicher des Browsers, nicht in einer URL. Das Passwort wird weder in die
Webseite eingesetzt noch dort gespeichert. Vier Sitzungen koennen gleichzeitig
bestehen; eine weitere Anmeldung ersetzt die aelteste. Sitzungen verfallen nach
15 Minuten ohne geschuetzte Anfrage. Die Hintergrundpruefung verlaengert sie nicht.
Abmelden verwirft die Sitzung; nach einem ESP-Neustart sind alle Sitzungen beendet.
Anmeldeversuche sind auf einen Versuch je zwei Sekunden begrenzt.

Anmeldung und Einstellungen ueber den vorhandenen lokalen **HTTPS-Proxy** nutzen.
Der direkte ESP-Webserver bietet weiterhin HTTP; die Anmeldung allein verschluesselt
HTTP nicht. Der Update-Zugang ist auf ausdruecklichen Wunsch von der Anmeldung ausgenommen:
ElegantOTA-Portal und Upload-Endpunkte sind ohne Passwort erreichbar. Ein Wechsel
des Web-Anmeldepassworts aktiviert dort keine Sperre. Wer den ESP im Netz erreicht,
kann dadurch auch Firmware hochladen; der geschuetzte Einstellungsbereich bleibt
von dieser Ausnahme unberuehrt.

## Anmeldepasswort aendern

Nach der Anmeldung steht eine eigene Kachel zur Verfuegung. Bisheriges Passwort,
neues Passwort und Wiederholung eingeben (1 bis 64 UTF-8-Bytes) und speichern. Erst nach
erfolgreicher NVS-Speicherung gilt das neue Passwort. Falsches bisheriges Passwort
oder Speicherfehler lassen den bisherigen Wert aktiv. Bei Erfolg werden alle
Web-Sitzungen ungueltig; eine erneute Anmeldung ist erforderlich. Der Update-Zugang ist davon ausgenommen und bleibt ohne Passwort. Der Benutzername bleibt unveraendert.

Ein gespeichertes Passwort bleibt bei Neustart und spaeteren Firmware-Updates
erhalten; die lokale Firmware-Vorgabe ist der Anfangswert ohne gespeicherten Eintrag.

## MQTT-Server und Themen umstellen

Die geschuetzte MQTT-Kachel enthaelt Server (IPv4-Adresse oder Hostname, ohne URL-
Schema), Port 1 bis 65535 und gemeinsamen Basispfad. Benutzername (bis 63 Bytes) und Passwort (bis 127 Bytes) sind optional. Beide leer
bedeutet Verbindung ohne Broker-Anmeldung. Bei vorhandenem Benutzer kann das
bestehende Passwort ueber die entsprechende Auswahl beibehalten werden; es wird
nie zurueckgesendet. Ein neues Passwort ersetzt das bisherige. Ohne die Auswahl
wird auch ein leeres Passwort gespeichert. Nur ein Passwort ohne Benutzername wird
abgelehnt. Die Web-Anmeldung und das MQTT-Passwort haben unabhaengige Laengengrenzen.

Vorhandene gespeicherte Server-/Themen-Einstellungen aus der vorherigen Firmware
bleiben beim Update erhalten; deren Broker-Zugangsdaten werden zunaechst aus der
bisherigen lokalen Konfiguration ergaenzt. Nach dem Speichern umfasst der gemeinsame
NVS-Datensatz Server, Port, Basispfad, Benutzername und Passwort. Verbindungs- oder
Speicherfehler stellen diese Werte gemeinsam wieder her.

Der Basispfad umfasst bis zu 80 ASCII-Zeichen ohne Leerzeichen oder MQTT-Platzhalter.
Ein abschliessender Schraegstrich wird automatisch ergaenzt; ein fuehrender wird
beibehalten, aber nicht automatisch hinzugefuegt. Alle Publisher sowie die drei
Abonnements fuer Befehle, Kachelofen und Gaszaehler nutzen denselben neuen Pfad.
Externe Automatisierungen muessen ebenfalls angepasst werden.

Die Hauptschleife trennt zunaechst den bisherigen Broker und leert die alten
Nachrichtenwarteschlangen. Waehrend des Tests werden keine MQTT-Steuerbefehle
angenommen und keine Statuswerte veroeffentlicht. Erst nach erfolgreicher neuer
Broker-Verbindung wird die Konfiguration als ein NVS-Datensatz gespeichert und die
neuen Themen abonniert. Die Verbindung wird mit sauberer Sitzung aufgebaut; alte
Abonnements werden nicht uebernommen. Ein fehlgeschlagener Verbindungsversuch (30
Sekunden) oder Speicherfehler fuehrt zur bisherigen Konfiguration zurueck. WLAN und
MQTT lassen sich nicht gleichzeitig umstellen.

Nach erfolgreicher Umstellung gilt der bisherige Kachelofen-Wert nicht als frische
Messung aus dem neuen Thema; eine neue Nachricht muss eintreffen. Sensor- und
Brennersperrstatus werden fuer den neuen Pfad erneut veroeffentlicht. Historische
Retain-Werte auf dem bisherigen Broker/Pfad werden nicht geloescht.

Der MQTT-Wechsel aendert die IP-Adresse des ESP nicht. Eine andere IP kann beim
Wechsel des **WLAN-Netzes** entstehen.

## WLAN wechseln

1. Anmelden und in der WLAN-Kachel den neuen Namen und das neue WLAN-Passwort eingeben.
2. **WLAN testen und speichern** waehlen. Die erste Antwort bestaetigt nur den Start.
3. Der ESP wartet ohne blockierende Schleife kurz auf die Uebermittlung dieser Antwort
   und beginnt dann den Verbindungsversuch. Die Heizungsregelung laeuft weiter.
4. Nach fuenf Sekunden durchgehender WLAN-Verbindung mit IP-Adresse werden die Daten
   als ein NVS-Datensatz gespeichert. Erst dann ist der Wechsel bestaetigt.
5. Ohne erfolgreiche Verbindung innerhalb von 30 Sekunden oder bei Speicherfehler
   wird mit den bisherigen Zugangsdaten erneut verbunden. Ein Neustart waehrend
   des Tests verwendet den vorher gespeicherten Datensatz.

Der WLAN-Name darf 1 bis 32 Bytes, die WPA/WPA2-Passphrase 8 bis 63 Bytes enthalten.
Die Laengengrenze des WLAN-Passworts ist unabhaengig vom Anmeldepasswort.
Offene WLANs und 64-stellige rohe PSK-Schluessel werden in dieser Oberflaeche nicht
angeboten. Gespeicherte WLAN-Passwoerter werden nie zurueckgesendet. Ohne gespeicherten
WLAN-Datensatz gelten die bisherigen lokalen Firmware-Zugangsdaten.

In einem anderen Netz kann der ESP eine andere IP-Adresse erhalten. Dann muessen
Browser, DNS und gegebenenfalls das Weiterleitungsziel des HTTPS-Proxys angepasst
werden; ein funktionierendes WLAN garantiert keine Erreichbarkeit des bisherigen
Proxys oder des MQTT-Brokers. Bei einem Rueckfall muss das bisherige WLAN erreichbar
sein. Es wird kein offener Einrichtungs-Hotspot gestartet.

## Pruefstand am Geraet

Bestaetigt sind die Aenderung des Web-Anmeldepassworts und der danach bewusst
offene Update-Zugang. Die Ursache der frueheren Ablehnung des geaenderten Passworts
an der separaten OTA-Anmeldung ist nicht abschliessend geklaert; der offene Zugang
ist eine bewusste Ausnahme.

Noch vollstaendig zu dokumentieren:

- Ohne Anmeldung alle Werte lesbar; Plus/Minus und Speichern gesperrt.
- Regelung/Betriebsart ohne Anmeldung weiterhin bedienbar.
- Anmelden, eine bestehende Einstellung speichern, abmelden und Sperre pruefen.
- Anmeldepasswort aendern, erneute Anmeldung und Beibehaltung nach Neustart pruefen.
- MQTT-Wechsel und Rueckfall bei unerreichbarem Broker pruefen; alle externen Themen
  anpassen und nach dem Wechsel eine frische Kachelofen-Meldung kontrollieren.
- WLAN-Wechsel nur bei verfuegbarem Zugriff auf beide Netze ausprobieren; bewusst
  falsche neue Zugangsdaten muessen zum bisherigen Netz zurueckfuehren.
- Erfolgreichen WLAN-Wechsel, neue Adresse und Beibehaltung nach Neustart pruefen.

Lokale Tests pruefen die tatsaechlichen Anmelde- und WLAN-Funktionen mit simuliertem
HTTP, WLAN und NVS, einschliesslich Sitzungsaustritt, Timerueberlauf, Verbindungs-
und Speicherfehler. Die Bedienungspruefung nutzt eine kleine DOM-Simulation;
sie ersetzt keinen Browser- oder Geraetetest.
