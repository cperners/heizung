# Betriebsarten

Die Auswahl steht unter **Betriebsart & Raumtemperatur**. Änderungen werden sofort
übernommen und in NVS gespeichert; kein Neustart ist erforderlich.

| Betriebsart | Raumheizung | Warmwasser |
|---|---|---|
| Aus | Keine normale Anforderung, kein Frostschutz | Aus |
| Frostschutz | Nur Frostschutz des Heizkreises | Aus |
| Automatik | Heizkurve mit täglichem Zeitprogramm | Ein, temperaturabhängig |
| Nur Heizung | Heizkurve mit täglichem Zeitprogramm | Aus |
| Nur Boiler | Kein normaler Heizbedarf; Frostschutz bleibt möglich | Ein, temperaturabhängig |
| Urlaub | Konstanter einstellbarer Absenksollwert, zunächst 15 °C | Aus |

Damit ist der separate Boiler-Aus-Betrieb über **Nur Heizung** erreichbar; es gibt
keinen zusätzlichen widersprüchlichen Freigabeschalter. Der Vorrang-Schalter bleibt
eine unabhängige Einstellung für Betriebsarten mit Warmwasserladung.

**Aus ist kein elektrischer Not-Aus.** Der Brenner wird beim Wechsel auf Aus sofort
ausgeschaltet, auch innerhalb der Mindestlaufzeit. Bereits erforderlicher
Pumpennachlauf und die vorhandene Reaktion auf Übertemperatur bleiben wirksam.
Eine noch laufende Boilerladung samt Boilerpumpen-Nachlauf wird bei Aus,
Frostschutz, Nur Heizung und Urlaub beendet. Je nach vorherigem Zustand kann die
Heizkreispumpe wegen Nachlauf bzw. restlicher Brenner-Mindestlaufzeit noch laufen.

In allen Betriebsarten bleiben Sensorüberwachung, Brennerabschaltung bei
Kessel-/Vorlauffehler, Übertemperaturgrenzen und Mischer-Verriegelung wirksam.
Bei Aus werden keine neuen Frostschutzanforderungen erzeugt. Die Frostschutz-
Betriebsart schützt den Heizkreis über gültige Außen-/Kessel-/Vorlaufwerte;
es gibt dabei keine Boiler-Ladeanforderung und keinen zugesicherten Frostschutz
für einen hydraulisch getrennten Warmwasserspeicher. Eine passende unabhängige
Anlagensicherheit bleibt erforderlich.

## Urlaub

Der Wert **Raumsoll im Urlaub** ist zwischen 5 und 25 °C einstellbar, Standard 15 °C.
Der Modus bleibt aktiv, bis eine andere Betriebsart gewählt wird; keine automatische
Rückkehr nach Datum. Sommerabschaltung und Frostschutz bleiben wirksam.

Bei ausgeschalteter Raumrückführung durch den Kachelofen wird die Raum-Isttemperatur
nicht verwendet. Die Sollwertabsenkung für Urlaub oder das Zeitprogramm bleibt
über den Unterschied zum normalen Raumsoll und den eingestellten Raumkorrekturfaktor
in der Heizkurve berücksichtigt. Der gespeicherte Faktor wird nicht verändert.
Bei Faktor null entfällt auch diese Parallelverschiebung; dann ist die Heizkurve
entsprechend zur Anlage auszulegen.

## MQTT und Web-API

Das Feld `mode` akzeptiert:

- `off`
- `frost`
- `auto`
- `heating`
- `dhw`
- `holiday`

`holiday_room_target` stellt den Urlaubssollwert ein. Beispiel für das Topic
`heizung/anlage/config/set` (nicht retained):

```json
{"mode":"holiday","holiday_room_target":15}
```

Der frühere Wert `manual` wird für vorhandene Konfigurationen und API-Clients
weiterhin unterstützt. Die Webauswahl zeigt ihn nur an, wenn er bereits aktiv ist.

## Übernahme vorhandener Daten

Die Heizungs-Konfiguration verwendet jetzt Version 2 im NVS-Schlüssel `config_v2`.
Ist dieser nicht vorhanden, wird eine gültige Version 1 einmalig übernommen und
nach erfolgreichem Commit gespeichert. Alle bisherigen Temperaturen, Zeiten und
Perioden bleiben erhalten; der neue Urlaubswert wird auf 15 °C gesetzt.
Ein früheres `off` wird zu `frost`, weil der alte Aus-Modus Frostschutz enthielt.
Erst die bewusste Auswahl des neuen **Aus** deaktiviert auch Frostschutz.
Fehlerhafte vorhandene V2-Daten führen zu einer Fehlermeldung, nicht zu einem
stillen Rückfall auf ältere Daten. MQTT-, Zeitserver-, Kachelofen- und Webzugangsdaten
liegen separat und werden nicht verändert.

## Verifikation

Hosttests prüfen alle sechs Modi, Beginn/Ende der Boilerladung, Warmwasservorrang,
Zeitprogramm bei Nur Heizung, Urlaub mit aktiver Kachelofen-Umschaltung, unmittelbare
Brennerabschaltung bei Aus, Pumpennachlauf, Sicherheitsvorrang sowie Migration mit
Commitfehlern und Neustartpersistenz. Firmware mit ESP-IDF v5.5.5 gebaut.
Keine zusätzliche Hardwareprüfung oder visuelle Browserprüfung durchgeführt.

Einstellbare Restwärmenutzung beim Wechsel auf Aus: [Verhalten und Prüfstand](RESTWAERME.md).
