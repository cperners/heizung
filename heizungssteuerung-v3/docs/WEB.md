# Weboberfläche direkt auf dem ESP32-P4-ETH

Die Firmware liefert die Kacheloberfläche unter `http://<Board-IP>/` auf Port 80.
Die zuletzt am Board beobachtete DHCP-Adresse war `192.168.0.50`; der Router kann
sie bei späteren Starts ändern. Eine Weboberfläche besteht aus der eingebetteten
HTML-Datei und benötigt keine Internetdateien, Schriftserver oder CDN-Verbindung.

## Bereiche

- Fünf Temperaturen mit Gültigkeit und Sensorfehlerzählern
- Betriebsart, Raum-/Vorlauf-/Kesselsoll, Frost und Warmwasseranforderung
- Tatsächliche Ausgangsanforderungen, Mischer und geschätzte Position
- Heizkurve und Raumkorrektur, Sommerabschaltung und Vorlaufgrenzen
- Boilerhysterese, Solltemperatur und Warmwasservorrang
- Vier tägliche Zeitperioden mit Raumsollwerten
- Kesselgrenzen, Brennerzeiten, Pumpennachlauf, Mischerzeiten und Frostgrenzen
- Optionale Pufferfühler, Ethernet, Speicher, Laufzeit und Zeitstatus
- MQTT-Server, Benutzername und Passwort zur Laufzeit konfigurierbar
- Kachelofentemperatur und Aktivstatus über ein frei einstellbares externes MQTT-Topic
- Drei NTP-Server zur Laufzeit konfigurierbar
- Eigener Webbenutzer und Webkennwort zur Laufzeit änderbar

Gasverbrauch wird aus einem externen MQTT-Gesamtzählerstand in m³ berechnet.
Topic, Aktualitätshinweis und eine manuelle Tagesbasis sind nach Anmeldung im
Systembereich einstellbar. Ohne Quelle bleibt die Anzeige leer. Details stehen in
[GASZAEHLER.md](GASZAEHLER.md). WLAN wird durch Ethernet ersetzt. PV-Leistungsregelung,
separate Wochenkalender und eine elektrische Ausgangsrückmeldung sind nicht
implementiert. Das tägliche Zeitprogramm steuert die Raumtemperatur, nicht einen
separaten Warmwasser-Ladekalender.

## Erster Start: zunächst nur Anzeige

Auf Nutzerwunsch bleibt die Ersteinrichtung des Webzugangs zunächst aus:
`HEATING_WEB_PASSWORD` ist leer. Die Seite ist ohne Anmeldung lesbar. Alle Änderungen
sind serverseitig gesperrt; es gibt keine voreingestellten Standard-Zugangsdaten.

Für spätere Bearbeitung einmal `scripts/idf.sh menuconfig` aufrufen und unter
„Heizungssteuerung“ den Web-Benutzernamen (Standard `admin`) und ein Webkennwort
mit mindestens 4 und höchstens 96 Zeichen setzen. Benutzername: 1 bis 32 Zeichen,
aus Buchstaben, Zahlen, Punkt, Unterstrich oder Bindestrich. Anschließend neu bauen
und flashen. Auf der Seite „Anmelden“ wählen. Das Webkennwort ist unabhängig vom
MQTT-Passwort. Benutzer und Kennwort können danach im Bereich „Webzugang“ geändert
werden; eine erneute Anmeldung ist erforderlich.

Ein über die Webseite geänderter Webzugang wird in NVS gespeichert und hat nach
Neustart Vorrang vor den Build-Vorgaben. Das Webkennwort wird als zufällig gesalzener
PBKDF2-SHA256-Hash mit 10.000 Iterationen gespeichert. Es wird nicht über die API
zurückgegeben. Der anfängliche Build-Wert ist dagegen Bestandteil der Firmware.
Ohne aktivierte Flash-/NVS-Verschlüsselung ist physischer Zugriff auf das Gerät
kein geschützter Speicherbereich.

## HTTP und Anmeldung

HTTP wurde ausdrücklich gewählt. Die Verbindung einschließlich Basic-Auth-Headern
ist unverschlüsselt. Nur im vertrauenswürdigen lokalen Netz einsetzen, keine
Portfreigabe ins Internet. Login ist keine Transportverschlüsselung. Die Übersicht
ist öffentlich im LAN; Konfiguration und alle Schreibzugriffe erfordern Anmeldung.
Der Browser hält die Anmeldung nur im Arbeitsspeicher der geöffneten Seite und
speichert sie nicht in localStorage oder Cookies. „Abmelden“ löscht diese Angaben
im Seitenzustand. Das Schließen/Neuladen der Seite erfordert erneute Anmeldung.

Schreibzugriffe verlangen zusätzlich ein pro Start zufälliges CSRF-Token und JSON.
Keine CORS-Freigaben. Die Seite darf nicht in fremde Frames eingebettet werden.
Das HTTP-Modul hat keine direkte Relaisschnittstelle und keine Lastfreigabe-API.

## Speichern und Neustart

Regelungsänderungen verwenden dieselbe strikte Validierung und denselben atomaren
NVS-Commit wie MQTT-Updates. Alle Felder eines Updates werden zusammen validiert.
Unzulässige Kombinationen werden nicht übernommen. Jeweils mindestens zwei Sekunden
zwischen Web-Schreibvorgängen lassen.

MQTT- und Zeitserver-Änderungen werden gemeinsam in einem eigenen NVS-Blob gehalten.
Das jeweilige Formular ändert nur seine Felder, bestätigt den bevorstehenden Neustart
und startet die Steuerung drei Sekunden nach erfolgreichem Speichern neu. Die
Reglerkonfiguration bleibt erhalten. Ein leeres MQTT-Passwortfeld bedeutet
„unverändert“. Die zusätzliche Löschoption entfernt ein gespeichertes Passwort.
Weder MQTT- noch Webpasswörter werden in API-Antworten zurückgegeben.

Beim ersten Start ohne gespeicherte Netzwerkdaten gelten die bereits vorhandenen
MQTT-Buildwerte. Die drei NTP-Vorgaben sind:

1. `192.168.0.1`
2. `0.at.pool.ntp.org`
3. `1.at.pool.ntp.org`

Der Router muss NTP anbieten, um als erster Server antworten zu können. Pool-Server
werden ebenfalls in SNTP eingetragen; die konkrete Auswahl der Erreichbarkeit
übernimmt SNTP. Zeitzone bleibt Europe/Vienna mit automatischer Sommerzeit.
Nach Webänderungen haben die gespeicherten Netzwerkdaten Vorrang vor menuconfig.
Ein erneuter Firmware-Flash löscht sie nicht.

## API

- `GET /api/status`: öffentliche Übersicht, frische Sensorgültigkeit, Diagnose,
  tatsächliche Ausgangsanforderungen, Ethernet/MQTT/NTP-Status
- `GET /api/config`: authentifizierte Regelungskonfiguration
- `POST /api/config`: validierte Teiländerungen
- `GET /api/services`: authentifizierte MQTT-/NTP-Konfiguration ohne Passwort
- `POST /api/services`: persistente Teiländerungen, anschließend Neustart
- `POST /api/account`: neuen Webbenutzer und Webkennwort speichern

Schreibzugriffe benötigen Authorization und `X-CSRF-Token`. Maximal 2048 Byte pro
JSON-Update, kurze Empfangszeitlimits, begrenzte Socketzahl. Regelung und
Ausgangsüberwachung laufen weiterhin in getrennten Tasks.

## Verifikation

Firmware mit ESP-IDF v5.5.5 für Chiprevisionen 3.1 bis 3.99 erfolgreich gebaut.
Portablen Regelungstest sowie neue Hosttests für Dienste-Konfiguration, NVS-Fehler,
Passwortredaktion, Anmeldung und Webzugangswechsel ausgeführt. Die Hosttests
verwenden simuliertes NVS und einen OpenSSL-Adapter statt MbedTLS; die tatsächliche
MbedTLS-Integration ist im Firmware-Build enthalten. Browser-JavaScript wurde
syntaktisch geprüft. Ein vorbereiteter Playwright-Test liegt in
`tests/web/browser_test.cjs`, konnte hier aber wegen blockierter Browserstarts in
der Sandbox nicht ausgeführt werden. Deshalb keine behauptete visuelle Browser-
oder Hardwareprüfung. Nach dem Flashen sind Darstellung, Anmeldung, drei
Zeitserver und Speichern am echten Gerät zu prüfen.

Kachelofen-Umschaltung und Einrichtung: [Kachelofen](KACHELOFEN.md).

Die sechs Betriebsarten und Urlaub: [Betriebsarten](BETRIEBSARTEN.md).

Heizkurve, Boilerparameter, Betriebsart und Zeitprogramm werden auch ohne Anmeldung als gesperrte Felder angezeigt und aktualisiert. Nur GET /api/config ist dafür öffentlich. Alle Schreibzugriffe sowie Netzwerk- und Zugangseinstellungen benötigen weiterhin eine Anmeldung. Während der Bearbeitung überschreibt die Statusabfrage keine Eingaben.

Ohne Anmeldung sind auch Kachelofen- und Gaszähler-Topics, drei Zeitserver und NTP-Synchronisationsstatus samt Alter der letzten erfolgreichen Synchronisation sichtbar. MQTT-Zugangsdaten bleiben geschützt. Die Oberfläche zeigt die konfigurierten Server; sie behauptet nicht, welcher Server geantwortet hat.

Die sechs Betriebsarten können auf Nutzerwunsch ohne Anmeldung über das separate Formular umgeschaltet werden. POST /api/mode akzeptiert ausschließlich ein mode-Feld aus off, frost, auto, heating, dhw, holiday. Zusätzliche Felder, Duplikate und der alte manual-Modus werden abgelehnt. CSRF-Token und Schreibbegrenzung gelten weiter. Alle anderen Änderungen benötigen weiterhin Anmeldung.

Das Heizkurvendiagramm zeigt die gespeicherte Gerade von −20 bis +30 °C außen, den begrenzten Sollwert, Minimum, die Sollwertobergrenze (flow_max minus 1 K), die Übertemperatur-Abschaltgrenze sowie die Sommergrenze. Die Sommerzone ist hinterlegt; dort gezeichnete Kurvenwerte bedeuten keinen normalen Heizbedarf. Raumkorrektur und betriebsartabhängige Verschiebungen sind nicht enthalten.

Sicherung und Wiederherstellung nach Anmeldung sowie der öffentliche Kachelofen-Verlauf sind in [SICHERUNG_VERLAUF.md](SICHERUNG_VERLAUF.md) beschrieben.

Firmwareupdates sind nach dem einmaligen USB-Umstieg zusätzlich über die angemeldete Oberfläche möglich. USB bleibt erhalten; Details und Prüfgrenzen stehen in [FIRMWARE_UPDATE.md](FIRMWARE_UPDATE.md).

## Fühlerdiagnose

Die öffentliche Kachel „Fühlerdiagnose · DS18B20“ zeigt für alle acht Eingänge GPIO und letzten Diagnosezustand. Die fünf Hauptfühler werden immer abgefragt; die drei Pufferfühler nur bei aktiviertem Puffer. Aufklappbar sind Präsenzimpuls, Anzahl Versuche/Erfolge, gelesene ROM-Adresse und Scratchpad. Es werden keine zusätzlichen Temperaturmessungen zur Diagnose gestartet; die Diagnose beschreibt den normalen Messzyklus. Zustandswechsel erscheinen auch im seriellen Log unter sensors.

Unterschieden werden Ruhepegel LOW, fehlgeschlagene LOW-Ansteuerung, fehlender Präsenzimpuls, nach Reset nicht freigegebene Leitung, ROM-Prüfsumme, falsche Sensorfamilie, gemeldete Parasitaerversorgung, nicht fertige Temperaturwandlung und Scratchpad-Prüfsumme bzw. unplausibles Datenformat. „Präsenzimpuls“ bedeutet, dass mindestens einer der Resetversuche im aktuellen Zyklus beantwortet wurde; eine spätere Resetantwort kann trotzdem fehlen.

Die Meldungen beweisen allein keinen elektrischen Schaden: Versorgung, gemeinsame Masse, externer 4,7-kΩ-Pull-up nach 3,3 V, Kontaktprobleme und Leitungsstörungen sind ebenfalls mögliche Ursachen. Ein Multimeter-Ruhepegel von rund 3,3 V beweist keine gültige 1-Wire-Kommunikation. Umverdrahten nur ohne Versorgung. In diesem Aufbau Fühler und Pull-up an 3,3 V; der Pull-up darf den ESP-GPIO nicht auf 5 V ziehen.

Die Diagnoseabläufe wurden mit geskripteten GPIO-Lesepegeln geprüft; diese Tests ersetzen keine Messung der realen elektrischen Signale oder Timings.

### Korrektur der Temperaturauswertung

Die CRC-gültigen Rohdaten `8E017F807FFF02108D` und `9C017F807FFF0410F5` wurden zuvor wegen fest erwarteter interner Bytes verworfen. Die Auswertung prüft nun CRC über alle acht Nutzbytes, Konfigurationsformat, Auflösung und Temperaturbereich. Bytes 5–7 werden nicht mehr als feste Temperatur-Gültigkeitsmarker verwendet. Die beiden Originaldaten sind als Regressionstest hinterlegt und ergeben 24,875 °C bzw. 25,75 °C. Daraus folgt kein Nachweis der Sensorherkunft oder Unversehrtheit aller elektrischen Funktionen. Referenz: [Analog Devices DS18B20, Abschnitt Memory](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf).

Der gemeinsame Temperaturverlauf umfasst inzwischen auch die fünf Hauptfühler, mit einzeln auswählbaren Kurven sowie 1-/6-/24-Stunden-Ansicht. Details zur Aufzeichnung und Zusammenfassung stehen in [SICHERUNG_VERLAUF.md](SICHERUNG_VERLAUF.md).

Das Anlagen-Basistopic ist nach Anmeldung im MQTT-Bereich änderbar. [MQTT_BASISTOPIC.md](MQTT_BASISTOPIC.md) beschreibt Untertopics, Migration und Anpassung externer Empfänger.

Einstellbare Restwärmenutzung beim Wechsel auf Aus: [Verhalten und Prüfstand](RESTWAERME.md).

## Dezimaldarstellung der Einstellungsfelder

Beim Laden von Heizkurve und übriger Regelungskonfiguration werden Zahlen entsprechend der Nachkommastellen ihrer Eingabeschrittweite formatiert. Beispielsweise erscheint eine Steilheit von 0,1 ohne binäre Gleitkomma-Reste wie 0,10000000149. Die Formatierung betrifft das Formular, nicht die Messauflösung oder die Regelungsberechnung.
