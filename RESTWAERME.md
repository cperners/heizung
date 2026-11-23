# Geregelte Restwärmeabfuhr in Betriebsart Aus

Stand: 29. Oktober 2026. Lokal gebaut und mit Hosttests geprüft; Gerätetest offen.

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

## Lokale Prüfungen

`python3 tests/host/test_off_heat_recovery.py` führt tatsächliche Quellfunktionen mit simulierten Sensoren und Ausgängen aus. Geprüft werden 35-Grad-Grenze, Brennersperre, Heizkurve trotz ausgewählter Raumregelung, Vorlaufmaximum, Mischer-Fahrimpulse, Zweiminuten-Stopp, Wiedererwärmung, Betriebsartwechsel, kalter Start in Aus, Nacht-Sollwert, ungültige Heizkurve, Sensorfehler und zehnminütiger Fehlernachlauf. Der Hosttest läuft mit AddressSanitizer und UndefinedBehaviorSanitizer. Er prüft keine elektrischen Ausgänge oder Wärmeabfuhr der realen Anlage.

Am Gerät nach Upload: Von Automatik oder Nur Heizung auf Aus wechseln. Brenner muss aus bleiben, bei warmem Kessel muss die Heizungspumpe laufen, der Mischer muss den Vorlauf regeln. Bei gültigem Kesselwert höchstens 35 °C muss die reguläre Pumpenanforderung enden und die Schließfahrt nach der eingestellten Dauer stoppen. Der reale Temperaturverlauf und die Wärmeabfuhr sind noch nicht bestätigt.
