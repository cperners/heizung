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
