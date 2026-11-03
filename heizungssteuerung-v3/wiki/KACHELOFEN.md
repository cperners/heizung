<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/KACHELOFEN.md -->

# Kachelofen über externen MQTT-Fühler

Die Erweiterung empfängt eine einzelne Gleitkommazahl in Grad Celsius, zeigt sie
als zusätzliche Temperaturkarte an und unterdrückt bei aktivem Kachelofen die
Raumkorrektur der außentemperaturabhängigen Heizkurve.

## Einrichtung in der Webseite

1. Neue Firmware flashen, Webseite neu laden und anmelden.
2. Unter **System → Kachelofen · externer MQTT-Fühler** das genaue MQTT-Topic
   eintragen, beispielsweise `haus/kachelofen/temperatur`.
3. Kachelofen-Umschaltung aktivieren.
4. Vorgaben prüfen: **aktiv ab 45 °C**, **inaktiv beim Abkühlen auf 32 °C**.
5. Maximales Empfangsalter einstellen; Standard **300 Sekunden**, einstellbar
   von 10 bis 3600 Sekunden. Das normale Sendeintervall muss darunter liegen.
6. Speichern und den angekündigten Neustart abwarten. MQTT abonniert das Topic
   automatisch bei jeder Verbindung. Ein neuer Firmware-Build ist für spätere
   Änderungen dieser Einstellungen nicht erforderlich.

Das Topic bleibt anfangs leer, die Funktion deaktiviert. Es wird kein fremdes
Topic angenommen. Es sind bis zu 191 ASCII-Zeichen ohne Leerzeichen oder
MQTT-Platzhalter `+`/`#` erlaubt. Der konfigurierte eigene Anlagen-Ausgangsbereich (Standard `heizung/anlage/`) und
`heizung/pv/` sind gesperrt, damit die Steuerung sich nicht selbst mit ihren
Statusausgaben als vermeintlich frischem Eingang versorgt. Der Broker muss dem
ESP-Benutzer das Abonnieren des externen Topics erlauben.

## Nachrichtenformat

Beispiel-Payload:

```text
46.5
```

Eine Zahl mit Dezimalpunkt, optional Vorzeichen, Exponent und äußerem Leerraum
wird akzeptiert. Keine JSON-Struktur, Einheit oder Dezimalkomma. Nichtendliche
Werte, ungültiges Format und Werte außerhalb -55 bis 500 °C werden abgewiesen.
Die Grenzen dienen der Plausibilitätsprüfung des externen Wertes und sind keine
Freigabe für einen bestimmten Sensortyp oder dessen Montageort.

Die Quelle muss regelmäßig neue Messwerte senden. Ein beim Abonnieren vom Broker
zugestellter **Retained-Snapshot wird nicht zur Regelung verwendet**, da dessen
Messalter unbekannt ist. Ein normal weitergeleiteter neuer Publish wird verarbeitet,
auch wenn der Sender ihn zusätzlich im Broker als retained speichert. Nach einem
Neustart oder MQTT-Reconnect wartet die Steuerung zunächst auf einen neuen Wert.

## Umschaltung und Hysterese

- Bei mindestens 45 °C: Kachelofen **aktiv**, Raumkorrekturfaktor für den laufenden
  Reglerzyklus null; der Vorlauf wird aus der Außen-Heizkurve abgeleitet.
- Beim Abkühlen auf höchstens 32 °C: Kachelofen **inaktiv**, normaler
  Raumkorrekturfaktor wieder wirksam.
- Zwischen 32 und 45 °C: letzter gültiger Aktivzustand bleibt erhalten.
- Beim ersten gültigen Wert im Zwischenbereich: zunächst inaktiv.
- Bei ungültigem Wert, Ablauf des maximalen Alters oder MQTT-Abbruch: keine aktive
  Ofenübersteuerung; normale Raumkorrektur wird wieder verwendet.

Der gespeicherte Raumkorrekturfaktor, die gewählte Betriebsart und die
Zeitprogramm-Einträge werden nicht geändert. Bei unterdrückter Raumrückführung wird statt der gemessenen Raumtemperatur der
normale konfigurierte Raumsollwert als Bezug für die Absenkung verwendet. Dadurch
bleiben Urlaub und Zeitprogramm als Parallelverschiebung wirksam, solange der
Raumkorrekturfaktor nicht null ist. Ein aktiver Ofen schaltet die Heizung nicht
eigenständig ein. Die getrennten Modi Aus und Frostschutz sind unter
[Betriebsarten](BETRIEBSARTEN.md) beschrieben. Kessel-/Vorlaufgrenzen,
Warmwasservorrang und sämtliche Sicherheitsabschaltungen bleiben wirksam.

## Anzeigen

Oben erscheint die zusätzliche Kachel **Kachelofen · MQTT**:

- Temperatur und Alter des letzten verwendbaren Empfangs
- **AKTIV** oder **INAKTIV**, solange gültige Daten vorliegen
- **Keine aktuellen Daten** bei fehlendem/ungültigem Eingang; keine behauptete
  sichere Aussage über den tatsächlichen Ofenzustand
- **Deaktiviert**, solange die Funktion ausgeschaltet ist

In der Regelungskachel wird die aktuelle Regelungsbasis angezeigt. Die
MQTT-Diagnose `heizung/anlage/diagnostics` enthält zusätzlich `stove` mit
Temperatur, Gültigkeit, Aktivzustand und Fehlerzähler sowie
`room_correction_suppressed`. Bei fehlenden Werten ist die Temperatur `null`.

Die Konfiguration liegt separat in NVS und verändert die vorhandenen MQTT-,
Webzugangs- und Heizungsdaten nicht. Die authentifizierten Endpunkte
`GET /api/stove` und `POST /api/stove` verwenden denselben Login, CSRF-Schutz,
Payloadgrenzen und Neustartablauf wie die übrigen Systemeinstellungen.

## Geprüft

Hosttests: Zahlenformat, Grenzwerte, Hysterese einschließlich exakter Schwellen,
Retained-Ablehnung, Ablauf, ungültiger Eingang, Kommunikationsabbruch, Rückfall,
keine Änderung des gespeicherten Raumfaktors, Übertemperaturabschaltung trotz
aktiver Ofenfunktion, Konfigurationsvalidierung, NVS-Commitfehler und Persistenz.
Regelungsregressionstests und JavaScript-Syntaxprüfung ebenfalls erfolgreich.
Firmware-Build mit ESP-IDF v5.5.5 erfolgreich. Der Nutzer hat den Empfang und den Aktivstatus am echten Gerät bestätigt. Automatisierte Browser-Tests waren in der Entwicklungsumgebung blockiert.
