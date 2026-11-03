<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/REGELUNG.md -->

> Die aktualisierte Modus-Auswahl ist in [Betriebsarten](BETRIEBSARTEN.md) beschrieben.
> Die dortige Trennung von Aus/Frostschutz ersetzt die frühere gemeinsame Bedeutung.

# Regelung und Fehlerreaktionen

## Normalbetrieb

Der Regler arbeitet alle 100 ms mit einer Kopie der Konfiguration und einer
Sensorkopie. Sensoren messen ungefähr einmal pro Sekunde; jeder Sensor wird
separat bewertet. Heizkurve: `curve_zero - curve_slope * Außentemperatur`, ergänzt
um `room_gain * (Raumsoll - Raumtemperatur)`. Standard: −10 °C → 55 °C,
0 °C → 45 °C, +10 °C → 35 °C bei passender Raumtemperatur. Vorlaufsoll wird auf
`flow_min .. flow_max-1` begrenzt. Ab `summer_temp` endet der normale Heizbedarf.

`auto` verwendet vier tägliche, nach Startminute aufsteigende Zeitperioden.
Die letzte Periode läuft über Mitternacht bis zum ersten Beginn. Standard:
06:00/21 °C, 08:30/18 °C, 16:30/21 °C, 22:30/17,5 °C. Kein Wochenkalender.
`manual` verwendet einen festen Raumsollwert, behält aber sämtliche Reglergrenzen.
`off` sperrt normale Raumheizung und Warmwasser einschließlich neuer Frostschutzanforderungen.
Für Frostschutz gibt es jetzt den separaten Modus `frost`.
Eine laufende normale Brenner-Mindestlaufzeit und Pumpennachlauf können bei
Umschaltung auf `off` zunächst weiterlaufen. `off` ist deshalb kein Not-Aus.

Kesselsoll ist Vorlaufsoll plus `kessel_margin`, bei Warmwasser
`boiler_target + dhw_lift`, maximal `kessel_max-3`. Die Brenneranforderung startet
unter Soll minus `burner_hysteresis` und endet ab Soll. Mindestlauf- und
Stillstandszeit verwenden monotone Zeit und sind unabhängig von NTP. Nach jedem
Neustart wird zunächst die volle Mindeststillstandszeit eingehalten.
Bei Rest-Mindestlaufzeit ohne Bedarf läuft die Heizkreispumpe, Mischer ZU.

Warmwasser startet unter Soll minus Boilerhysterese und endet bei Soll.
Die Boilerpumpe startet erst bei Kesseltemperatur > Boiler + 2 °C, läuft bis zum
Temperaturausgleich und stoppt spätestens bei Boiler-Soll. Bei kaltem Kessel
kann zunächst nur der Brenner angefordert sein: der Kessel muss eine eventuell
erforderliche interne Zirkulation selbst gewährleisten. Warmwasservorrang stoppt
die Heizkreispumpe und schließt den Mischer. Nachlauf der Boilerpumpe erfolgt nur,
wenn Kessel/Boiler gültig sind, der Kessel wärmer ist und Boiler-Soll nicht erreicht
ist. Warmwasservorrang unterdrückt Heizkreis-Nachlauf während der Ladung.

Der Mischer öffnet/schließt außerhalb `mixer_band` um Vorlaufsoll, sonst STOP.
Vor dem ersten Öffnen erfolgt eine Referenzfahrt ZU. Eine ununterbrochene Fahrt
ist auf `mixer_travel_s` begrenzt. Nach Referenzfahrt werden Position und Endpunkte
nur geschätzt. Richtungswechsel benötigen mindestens `mixer_dead_ms`, auch wenn
ein STOP-Befehl dazwischen liegt. Neustart verwirft die Positionsschätzung.

## Sicherheit und Verfügbarkeit

| Zustand | Reaktion |
|---|---|
| Kesselfühler fehlt/CRC/ungültig/veraltet | Brenner sofort AUS; beide Pumpen AUS; Mischer ZU |
| Vorlauffühler fehlt/CRC/ungültig/veraltet | gleiche Abschaltung |
| Kessel ≥ `kessel_max` oder Vorlauf ≥ `flow_max` | Brenner sofort AUS; Heizkreispumpe EIN bei gültigen Kessel-/Vorlauffühlern; Boilerpumpe AUS; Mischer ZU |
| Boilerfühler defekt | keine Warmwasserladung; Raumheizung bleibt möglich |
| Außenfühler defekt | normaler Heizbedarf gesperrt; Warmwasser und Frost aus lokalen Wasserfühlern bleiben möglich |
| Raumfühler defekt | reine Heizkurve ohne Raumkorrektur |
| Messwertalter ≥ `sensor_stale_s` | Sensor wird als ungültig behandelt |
| Keine NTP-Synchronisation oder letzte Synchronisation >24 h alt | Auto-Modus verwendet `unsynced_room`; Fehlerbit |
| Ethernet/MQTT ausgefallen | lokale Regelung mit letzter gültiger Konfiguration läuft weiter; Fehlerbit |
| Regler liefert >500 ms keine Ausgangsanforderung | separate Ausgangstask schaltet alles AUS; Fehler verriegelt bis Neustart |
| Task hängt insgesamt | ESP-Task-Watchdog; unabhängige physische Sicherheitskette erforderlich |
| Ungültige NVS-Konfiguration | Start ohne Lastfreigabe; Fehler im seriellen Log; kein automatisches Löschen |

Die temperaturbedingten Abschaltungen und Sensorfehler umgehen Mindestlaufzeiten.
Nach wieder gültigen Werten bzw. Abkühlung ist die normale Regelung wieder möglich;
Brenner-Stillstandszeit bleibt wirksam. Es gibt keinen separaten quittierbaren
Übertemperatur-Latch; die unabhängige mechanische Kesselsicherheit bleibt maßgeblich.

Frost wird unter `frost_outside` oder bei einem gültigen Kessel-, Vorlauf- oder
Boilerwert unter `frost_water` erkannt. Es erzeugt Heizbedarf auch bei `off`, mit
mindestens `frost_flow`. Kritische Sensorfehler und Übertemperatur haben Vorrang.
Die angemessene Reaktion auf Übertemperatur (Pumpen an/aus, Wärmeabfuhrweg) muss
zur Hydraulik passen; die hier gewählte Heizkreiszirkulation mit geschlossenem
Mischer ist nicht für jede Anlage ausreichend. Keine automatische Entladung in
Boiler oder Puffer über deren zulässige Temperaturen hinaus.

Fehlerbits in `diagnostics.faults`: Kessel=1, Vorlauf=2, Kessel heiß=4,
Vorlauf heiß=8, Boiler=16, Außen=32, Raum=64, Zeit=128, Netzwerk=256.
`output_timeout` separat. Sensorfehlerzähler sind kumulativ seit Start.
