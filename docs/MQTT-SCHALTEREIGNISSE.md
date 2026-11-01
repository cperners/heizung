# MQTT-Protokoll der Schaltausgaenge

Die Firmware erfasst ab diesem Stand die GPIO-Schreibvorgaenge von Brenner,
Heizungs- und Boilerpumpe sowie Mischer auf/zu. Aendert sich ein Ausgang, wird
sofort im Hauptloop ein Ereignis mit Zeitstempel in einen RAM-Puffer aufgenommen.
Wiederholtes Schreiben desselben Pegels erzeugt keine neue Schaltmeldung.
Auch mehrere Wechsel innerhalb einer Hauptschleife bleiben getrennt erhalten.
Die Regelungsentscheidungen, Fahrzeiten und Sicherheitssperren bleiben unveraendert.

## Empfang

Bei unveraendertem Basispfad lautet das gemeinsame Ereignisthema:

`/SmartHome/Keller/Heizung/schaltereignis`

Bei einer Aenderung des MQTT-Basispfades folgt dieses Thema der neuen Einstellung.
Der Versand verwendet **QoS 1, Retain aus**. Die bisherige `JDATA`-Statusmeldung
mit ihrem Zehn-Sekunden-Intervall bleibt erhalten.

Ein Beispiel einer eingeschalteten Heizungspumpe:

```json
{
  "bootId": "a12b34c5",
  "seq": 42,
  "typ": "change",
  "uptimeMs": 123456,
  "unixMs": 1793500000000,
  "changedMask": 2,
  "BrennerRelais": 0,
  "HeizungsRelais": 1,
  "BoilerRelais": 0,
  "MischerAufRelais": 0,
  "MischerZuRelais": 0,
  "verlorenSeitStart": 0
}
```

Die fuenf Ausgangswerte sind hier **JSON-Zahlen**: `1` bedeutet angesteuert,
`0` ausgeschaltet. Es handelt sich nicht um eine mechanische Rueckmeldung
oder einen Nachweis von Pumpendrehung, Flamme oder Ventilstellung.

## Auswertung

- `typ: change`: tatsaechlicher Pegelwechsel eines Ausgangs.
- `typ: startup`: Ausgangszustand beim Firmwarestart, kein Schaltwechsel.
- `typ: connected`: aktueller Zustand nach der ersten MQTT-Verbindung bzw.
  Wiederverbindung, ebenfalls kein Schaltwechsel.
- `typ: resync`: aktueller Zustand nach einem vollen Ereignispuffer.
- `changedMask`: bei einem Wechsel `1` Brenner, `2` Heizungspumpe, `4` Boilerpumpe,
  `8` Mischer auf oder `16` Mischer zu; bei Zustandsaufnahmen `0`.
- `uptimeMs`: Zeitpunkt der Erfassung in Millisekunden seit Start, 64 Bit.
  Fuer Schalt- und Laufzeitdifferenzen innerhalb desselben Starts verwenden.
- `unixMs`: UTC-Zeitpunkt als Unix-Millisekunden bei der Erfassung. Vor einer
  gueltigen Systemzeit ist der Wert `0`. Uhrzeitkorrekturen koennen ihn verschieben.
- `bootId`: zufaellige achtstellige Kennung dieses Starts.
- `seq`: fortlaufende Ereignisnummer je Start. Gemeinsam mit `bootId` zur
  Erkennung von Wiederholungen verwenden. QoS 1 kann doppelt liefern.
- `verlorenSeitStart`: Zahl verworfener Ereignisse bei vollem RAM-Puffer,
  Stand zum Versandzeitpunkt. Nummernluecken und dieser Zaehler zeigen
  moegliche Luecken an; fuer solche Intervalle keine exakte Laufzeit annehmen.

Beim Replay nach einer Verbindungsunterbrechung gilt der Erfassungszeitpunkt
im Datensatz, nicht die Empfangszeit im Analyseprogramm. Nach einem
Broker-/Basispfadwechsel wird vorhandener Rueckstand an das dann aktive Thema
auf dem aktiven Broker gesendet. Waehrend des Verbindungstests wird nichts gesendet.

## Puffer und Grenzen

Der feste RAM-Puffer fasst **128 Meldungen**, einschliesslich Zustandsaufnahmen.
Es ist hoechstens eine Ereignismeldung ohne MQTT-Empfangsbestaetigung unterwegs.
Erst bei PUBACK wird diese Meldung aus dem Puffer entfernt. Ein abgelehnter
Versand wird fruehestens nach einer Sekunde erneut versucht. Nach Verbindungsabbruch
bleibt die unbestaetigte Meldung zur Wiederholung erhalten.

Bei vollem Puffer werden neue Meldungen verworfen und gezaehlt. Sobald Platz frei
wird, folgt eine aktuelle Zustandsaufnahme. Ein Neustart/Stromausfall loescht den
RAM-Puffer; es gibt kein dauerhaftes Archiv im ESP und keine unbegrenzte Garantie
vollstaendiger Historie. Deshalb muss der externe Empfaenger laufend protokollieren.
Eine Brokerbestaetigung bestaetigt nicht die Speicherung im Analyseprogramm.

Die Erfassung wartet niemals auf Netzwerk oder Flash und veraendert keine
Heizungsfreigabe. Sie ersetzt keine hardwareseitige Sicherheitsueberwachung.

## Pruefung

Lokal automatisiert geprueft: kurze Impulse innerhalb einer Schleife, alle fuenf
Ausgaenge, unveraenderte Pegel, fehlende Systemzeit, 64-Bit-Laufzeit, Einreihenfehler,
fruehe und fremde ACKs, Wiederholung nach Verbindungsabbruch, begrenzter Puffer,
Verlustzaehler und Zustandsaufnahme nach Ueberlauf sowie Sperre beim Brokertest.
Die bisherigen Heizungs-, Mischer-, Netzwerk- und Web-Pruefungen laufen weiter.

**Noch offen am Geraet:** Nach dem Upload das Ereignisthema abonnieren und einen
normalen Pumpenwechsel sowie einen kurzen Mischerimpuls beobachten. Das Ein- und
Ausschalten muss jeweils eine eigene Meldung mit den Erfassungszeiten liefern.
Dann laesst sich die Differenz der beiden `uptimeMs`-Werte auswerten.

## Node-RED: fuenf einzelne Ausgaenge

Einen MQTT-in-Knoten mit einem Function-Knoten verbinden und den Function-Knoten
auf **fuenf Ausgaenge** stellen. Ein zusaetzlicher JSON-Knoten ist nicht noetig.
Die Funktion akzeptiert ein JSON-Objekt, JSON-Text oder einen Buffer.

| Ausgang | Signal | Payload |
|---|---|---|
| 1 | Brenner | Zahl 0 oder 1 |
| 2 | Heizungspumpe | Zahl 0 oder 1 |
| 3 | Boilerpumpe | Zahl 0 oder 1 |
| 4 | Mischer auf | Zahl 0 oder 1 |
| 5 | Mischer zu | Zahl 0 oder 1 |

Die vereinbarte Einstellung ist `nurAenderungen = false`: **jede empfangene
Ereignismeldung liefert alle fuenf Werte**. Das erzeugt keine regelmaessigen
Meldungen zwischen den Ereignissen. Bei `true` wird fuer Schaltwechsel nur der
betroffene Ausgang bedient; Zustandsaufnahmen bedienen weiterhin alle fuenf.

`msg.payload` enthaelt die Zahl, `msg.topic` den Ausgangsnamen, `msg.mqttTopic`
das urspruengliche MQTT-Thema und `msg.ereignis` den vollstaendigen Datensatz.
Im Debug-Knoten "vollstaendige Nachricht" waehlen. Danach Fertig und Deploy.
Diese Funktion entfernt keine doppelten Ereignisse und berechnet noch keine
Laufzeiten. Dafuer Boot-ID/Sequenz und Erfassungszeiten auswerten.

```javascript
// Function-Knoten: Anzahl der Ausgaenge auf 5 stellen.
const nurAenderungen = false;
const felder = [
    "BrennerRelais",
    "HeizungsRelais",
    "BoilerRelais",
    "MischerAufRelais",
    "MischerZuRelais"
];

let daten = msg.payload;
try {
    if (Buffer.isBuffer(daten)) daten = daten.toString("utf8");
    if (typeof daten === "string") daten = JSON.parse(daten);
    if (!daten || typeof daten !== "object" || Array.isArray(daten)) {
        throw new Error("JSON-Objekt erwartet");
    }
    for (const feld of felder) {
        if (daten[feld] !== 0 && daten[feld] !== 1) {
            throw new Error("Ungueltiger Ausgangswert: " + feld);
        }
    }
    if (nurAenderungen && daten.typ === "change" &&
        (!Number.isInteger(daten.changedMask) ||
         daten.changedMask < 1 || daten.changedMask > 31)) {
        throw new Error("Ungueltige changedMask");
    }
} catch (fehler) {
    node.error("Schaltereignis: " + fehler.message, msg);
    return null;
}

return felder.map((feld, index) => {
    if (nurAenderungen && daten.typ === "change" &&
        (daten.changedMask & (1 << index)) === 0) {
        return null;
    }
    return {
        ...msg,
        mqttTopic: msg.topic,
        topic: feld,
        payload: daten[feld],
        ereignis: { ...daten }
    };
});
```
