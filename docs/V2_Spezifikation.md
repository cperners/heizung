# Heizungssteuerung V2 – Lasten- und Pflichtenheft

**Projekt:** Heizungssteuerung ESP32  
**Ausgangsbasis:** bestehendes Projekt `cperners/heizung`  
**Version:** V2 – Entwurf  
**Stand:** 04.10.2026

## 1. Ziel der V2

Die bestehende ESP32-Heizungssteuerung wird funktional weiterentwickelt und strukturell verbessert.

Die wichtigste Erweiterung ist die automatische Erkennung des Betriebs eines holzbefeuerten Kachelofens über einen zusätzlichen externen Temperatursensor, dessen Messwert per MQTT übertragen wird.

Der Kachelofen erwärmt den Raum direkt. Er ist **nicht hydraulisch in den Heizkreis eingebunden**.

Während der Kachelofen ausreichend heiß ist, soll die Heizungssteuerung auf die Außentemperatur-/Heizkurvenregelung umschalten.

Sobald der Kachelofen nicht mehr ausreichend heiß ist, wird automatisch zur normalen Raumtemperaturregelung zurückgeschaltet.

## 2. Grundprinzip

Es gibt zwei Heizungsregelungsarten:

### A – normale Raumregelung

Aktiv, wenn der Kachelofen nicht aktiv ist.

`Raumtemperatur → Raum-Sollwert → Heizanforderung → Heizungssteuerung`

### B – Kachelofenbetrieb / Außentemperaturregelung

Aktiv, solange der Kachelofen ausreichend heiß ist.

`Außentemperatur → Heizkurve → Vorlauf-Soll → Mischerregelung`

Während dieser Betriebsart wird die normale Raumtemperaturregelung **nicht als direkte Heizanforderung** verwendet.

Der Grund: Der Kachelofen erwärmt den Raum selbst.

## 3. Kachelofen-Erkennung

Ein zusätzlicher externer Temperatursensor misst die Temperatur direkt am Kachelofen.

Der ESP32 erhält den Messwert über MQTT.

Beispiel:

`kachelofen/temperature = 65.4`

Der Messwert dient zur Erkennung:

`Kachelofen aktiv / Kachelofen nicht aktiv`

## 4. Einstellbare Kachelofen-Umschaltung

Es werden zwei Temperaturen verwendet:

- **Kachelofen EIN**
- **Kachelofen AUS**

Beispielwerte:

- EIN: `50 °C`
- AUS: `40 °C`

Die tatsächlichen Werte müssen später über Webinterface und MQTT veränderbar sein.

## 5. Hysterese

| Kachelofen-Temperatur | Zustand |
|---:|---|
| < 40 °C | Kachelofen AUS |
| 40–49,9 °C | vorheriger Zustand bleibt erhalten |
| ≥ 50 °C | Kachelofen EIN |

Damit wird ein ständiges Umschalten verhindert.

## 6. Kachelofenbetrieb

Wenn die Kachelofentemperatur die EIN-Schwelle erreicht:

`Kachelofen-Temperatur >= Kachelofen_EIN`

wird:

`KachelofenAktiv = TRUE`

gesetzt.

Die Regelung wechselt auf die Außentemperaturregelung.

## 7. Beendigung des Kachelofenbetriebs

Wenn:

`Kachelofen-Temperatur <= Kachelofen_AUS`

wird:

`KachelofenAktiv = FALSE`

gesetzt.

Danach wird wieder die normale Raumtemperaturregelung verwendet.

## 8. MQTT-Ausfall

Der Kachelofenstatus darf nicht dauerhaft auf dem letzten gültigen MQTT-Wert stehen bleiben.

Jeder gültige Messwert erhält einen Zeitstempel.

Wenn länger als die konfigurierte Timeout-Zeit kein gültiger Messwert empfangen wurde:

`KachelofenAktiv = FALSE`

und die Steuerung kehrt zur normalen Raumregelung zurück.

Empfohlener Startwert:

`MQTT-Timeout = 10 Minuten`

Der Wert soll konfigurierbar sein.

## 9. Normale Raumregelung

Wenn:

`KachelofenAktiv = FALSE`

wird die bestehende Raumtemperaturregelung verwendet.

Beispiel:

- Raum-Soll = `22,0 °C`
- Raum-Ist = `21,5 °C`

→ Heizbedarf.

Wenn Raum-Ist den Sollwert erreicht, besteht kein direkter Raum-Heizbedarf.

Die vorhandene Raumtemperatur-Hysterese soll grundsätzlich erhalten bleiben.

## 10. Kachelofenbetrieb

Wenn:

`KachelofenAktiv = TRUE`

wird die Außentemperaturregelung aktiviert.

Der Raum-Istwert wird in dieser Betriebsart **nicht als direkte Heizanforderung für die Gasheizung verwendet**.

Das ist ausdrücklich gewollt, weil der Kachelofen den Raum direkt erwärmt.

## 11. Heizkurve

Die vorhandene Heizkurvenfunktion soll grundsätzlich erhalten bleiben.

Die benötigte Vorlauftemperatur wird aus den vorhandenen Parametern berechnet, insbesondere:

- Raum-Solltemperatur
- Außentemperatur
- maximale Vorlauftemperatur
- minimale Außentemperatur
- Heizkurvenparameter

Grundprinzip:

`Außen kälter → Vorlauf-Soll höher`

`Außen wärmer → Vorlauf-Soll niedriger`

## 12. Mischerregelung

Der Mischer wird anhand von:

`Vorlauf-Ist gegen Vorlauf-Soll`

geregelt.

Wenn:

`Vorlauf-Ist < Vorlauf-Soll`

→ Mischer öffnen.

Wenn:

`Vorlauf-Ist > Vorlauf-Soll`

→ Mischer schließen.

Die vorhandene Schrittsteuerung des Mischers soll zunächst erhalten bleiben.

## 13. Gasheizung während Kachelofenbetrieb

Während:

`KachelofenAktiv = TRUE`

wird die normale Raumtemperatur-Heizanforderung nicht verwendet.

Die genaue Kopplung zwischen Kachelofenbetrieb, Kessel und Brenner wird bei der Implementierung anhand des vorhandenen Kesselregelungscodes festgelegt.

Die Hardware-Schutzfunktionen bleiben davon unabhängig.

## 14. Boilerbetrieb

Die Warmwasserbereitung bleibt unabhängig vom Kachelofenbetrieb.

Beispiele:

- Kachelofen AUS + Boilerbedarf → Boilerbetrieb möglich
- Kachelofen EIN + Boilerbedarf → Boilerbetrieb weiterhin möglich

Der Kachelofen darf die Warmwasserbereitung nicht unbeabsichtigt blockieren.

Die bestehende Boilerhysterese soll zunächst erhalten bleiben.

## 15. Betriebszustände

Der Kachelofen ist keine eigene manuelle Betriebsart, sondern beeinflusst die automatische Heizungsregelung.

Intern wird mindestens zwischen folgenden Zuständen unterschieden:

- `Kachelofen INAKTIV`
- `Kachelofen AKTIV`

Die bestehenden manuellen Betriebsarten des Projekts bleiben erhalten und werden bei der Implementierung berücksichtigt.

## 16. Beispiele

### Kachelofen aus

`Kachelofen = 30 °C`

→ Kachelofen nicht aktiv.

Bei Raum-Soll `22,0 °C` und Raum-Ist `21,5 °C`:

→ normale Heizungsregelung.

### Kachelofen wird angeheizt

Kachelofen `45 °C`:

→ noch nicht aktiv.

Kachelofen `50 °C`:

→ Kachelofen aktiv.

→ Umschaltung auf Außentemperatur-/Heizkurvenregelung.

### Kachelofen bleibt aktiv

Bei EIN = `50 °C` und AUS = `40 °C`:

Kachelofen `48 °C` → bleibt aktiv.

Kachelofen `45 °C` → bleibt aktiv.

Kachelofen `40 °C` oder niedriger → inaktiv.

### MQTT-Ausfall

Letzter gültiger Wert: `65 °C`.

Danach keine MQTT-Daten.

Nach Ablauf des Timeouts:

→ Kachelofen inaktiv  
→ normale Raumtemperaturregelung.

## 17. Konfigurationsparameter V2

Mindestens folgende Werte sollen konfigurierbar sein:

| Parameter | Beispiel |
|---|---:|
| Kachelofen EIN | 50 °C |
| Kachelofen AUS | 40 °C |
| MQTT Timeout | 10 min |
| Raum Tag | 22,9 °C |
| Raum Nacht | 22,0 °C |
| Raum-Hysterese | 0,1 K |
| Kessel-Soll | 72 °C |
| Kessel-Hysterese | 14 K |
| Boiler-Soll | 55 °C |
| Boiler-Hysterese | 10 K |
| maximale Vorlauftemperatur | 62 °C |
| minimale Außentemperatur | −15 °C |
| Heizkurvenparameter | vorhandene Projektwerte |

Die vorhandenen Werte werden zunächst übernommen.

## 18. Bedienung

Die Kachelofenparameter sollen über das vorhandene Webinterface eingestellt werden können.

Vorgesehene Anzeige:

```text
Kachelofen
────────────────────

Status:
AKTIV

Temperatur:
63,4 °C

Aktiv ab:
50,0 °C

Inaktiv unter:
40,0 °C

MQTT Timeout:
10 min
```

## 19. MQTT

Die bestehende MQTT-Struktur soll erweitert werden.

Beispiel:

- `heizung/kachelofen/temperature`
- `heizung/kachelofen/active`
- `heizung/kachelofen/ein_temp`
- `heizung/kachelofen/aus_temp`
- `heizung/kachelofen/timeout`

Die endgültige Topic-Struktur wird an die vorhandene Projektstruktur angepasst.

## 20. Speicherung

Die Kachelofenparameter müssen einen Neustart des ESP32 überleben.

Mindestens zu speichern:

- Kachelofen EIN
- Kachelofen AUS
- MQTT Timeout

Die vorhandene EEPROM-Lösung kann zunächst weiterverwendet werden.

Es soll geprüft werden, ob ESP32 Preferences/NVS langfristig sinnvoller ist.

## 21. Sensorfehler

Ein ungültiger Kachelofen-Messwert darf nicht zur Aktivierung des Kachelofenbetriebs führen.

Ungültig sind insbesondere:

- fehlender MQTT-Wert
- ungültiger Zahlenwert
- MQTT-Timeout
- offensichtlich fehlerhafter Temperaturwert

In diesen Fällen:

`KachelofenAktiv = FALSE`

## 22. Hardware-Sicherheit

Folgende Funktionen sind hardwareseitig vorhanden und bleiben unabhängig von der Software:

- Brennerschutz
- Kesselschutz
- Überhitzungsschutz
- vorhandene Sicherheitsthermostate

Die V2-Software ersetzt diese Schutzfunktionen nicht.

## 23. Software-Sicherheitsmaßnahmen

Zusätzliche Plausibilitätsprüfungen:

- Sensor ungültig → definierter Fallback
- MQTT ausgefallen → Kachelofenbetrieb beenden
- Konfigurationswert außerhalb des erlaubten Bereichs → Änderung ablehnen
- EIN-Schwelle <= AUS-Schwelle → Änderung ablehnen

## 24. Priorität der Regelungen

Grundsätzlich:

1. Hardware-Sicherheit
2. Fehler-/Fallback-Zustände
3. Boileranforderung
4. Kachelofenstatus
5. Heizungsregelung
6. Mischerregelung

Die genaue Priorisierung von Boiler und Heizkreis wird beim Umbau anhand des vorhandenen Codes überprüft.

## 25. Zielarchitektur

Die bisherige große `main.cpp` soll langfristig logisch aufgeteilt werden.

Vorgesehene Komponenten:

```text
TemperatureSensors
        │
        ▼
HeatingController
        │
        ├── RoomController
        ├── BoilerController
        ├── KachelofenController
        └── MixerController

MqttInterface
WebInterface
ConfigManager
TimeController
OutputController
```

Die Regelungslogik soll möglichst unabhängig von MQTT, Webinterface und Hardwaretreibern werden.

## 26. Testfälle

### Kachelofen

- 30 °C → AUS
- 50 °C → EIN
- 45 °C → EIN
- 40 °C → AUS

### MQTT

- gültiger Wert → normal
- ungültiger Wert → AUS
- Timeout → AUS

### Raumregelung

- Raum unter Soll → Heizbedarf
- Raum über Soll → kein direkter Raum-Heizbedarf

### Heizkurve

- Außen kälter → Vorlauf-Soll höher
- Außen wärmer → Vorlauf-Soll niedriger

### Mischer

- Vorlauf zu kalt → AUF
- Vorlauf zu warm → ZU

### Übergänge

```text
Raumregelung
      ↓
Kachelofen EIN
      ↓
Heizkurve
      ↓
Kachelofen AUS
      ↓
Raumregelung
```

## 27. Designprinzip

Die Kachelofenlogik soll nicht als ungeordnete zusätzliche `if`-Abfragen in die bestehende `main.cpp` eingebaut werden.

Stattdessen soll sie einen eindeutig definierten Regelungszustand liefern:

```text
Kachelofen INAKTIV
        ↓
ROOM_CONTROL

Kachelofen AKTIV
        ↓
OUTDOOR_CURVE_CONTROL
```

Damit bleibt nachvollziehbar, warum die Anlage zu einem bestimmten Zeitpunkt einen bestimmten Zustand einnimmt.

## 28. Zusammenfassung

Die zentrale V2-Änderung lautet:

```text
                 Kachelofenfühler
                       │
                       ▼
              Kachelofen aktiv?
                 /                         NEIN         JA
                │            │
                ▼            ▼
          Raumregelung   Heizkurve
                             │
                             ▼
                           Mischer
```

Der Kachelofen wird ausschließlich über seine Temperatur erkannt.

Er ist eine **raumseitige Wärmequelle** und keine hydraulische Wärmequelle.

Solange er ausreichend heiß ist, wird die Außentemperatur-/Heizkurvenregelung verwendet.

Wenn er auskühlt oder die MQTT-Verbindung ausfällt, übernimmt wieder die normale Raumtemperaturregelung.

Die Umschalttemperaturen und der MQTT-Timeout sind einstellbar und werden dauerhaft gespeichert.
