<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/KONFIGURATION.md -->

# Konfiguration und MQTT

## Transport

Broker, Benutzer und Passwort werden zunächst beim Build in `menuconfig` gesetzt;
später sind sie mit Webanmeldung unter MQTT änderbar und persistent. Gespeicherte
Webwerte haben nach Neustart Vorrang vor den Buildwerten. Details: [Weboberfläche](WEB.md).
Relaispolarität und physische Lastfreigabe werden ausschließlich beim Build gesetzt. Zugangsdaten stehen dadurch in der Firmware
und lokal in `sdkconfig`; diese Datei nicht veröffentlichen. DHCP ist aktiv.
Für `mqtts://` ist das ESP-Zertifikatsbundle eingebunden; eine private CA muss
in das Bundle aufgenommen werden. Vor erfolgreichem NTP kann TLS-Zertifikatsprüfung
scheitern und wird beim automatischen Reconnect erneut versucht.

Ein Broker mit ACLs muss `heizung/anlage/config/set` nur für berechtigte Nutzer
freigeben. Aktuell ist `mqtt://192.168.0.1:1883` ohne Benutzername und Passwort konfiguriert.
Diese Verbindung ist unverschlüsselt. Netzwerk ist nur Ethernet; kein WLAN.

## Topics

Alle Status- und Messwertpublikationen verwenden QoS 1 und retained. Der Last Will
ist `heizung/anlage/status = offline`; nach Verbindung wird `online` veröffentlicht.
Der ESP-MQTT-Client verbindet automatisch nach 5 s erneut. Telemetrie wird alle
5 s angeboten; die Outbox ist auf 16 KiB begrenzt, sodass fehlende Bestätigungen
keinen unbegrenzten Speicherverbrauch verursachen. Volle Outbox kann Statuswerte
verwerfen; der nächste Zyklus bietet sie erneut an. QoS 1 ist keine Garantie für
die Annahme einer Konfiguration: dafür `config/result` und `config` prüfen.

| Topics unter `heizung/anlage/` | Payload |
|---|---|
| `aussen`, `raum`, `kessel`, `vorlauf`, `boiler` | Temperatur in °C oder `null` |
| `puffer/oben`, `puffer/mitte`, `puffer/unten` | Temperatur oder `null`, standardmäßig deaktiviert |
| `brenner`, `heizungspumpe`, `boilerpumpe`, `mischer_auf`, `mischer_zu` | tatsächlich geschriebene Ausgangsanforderung: `0`/`1` |
| `soll/raum`, `soll/vorlauf`, `soll/boiler` | Sollwert in °C |
| `mode` | `off`, `auto`, `manual` |
| `status` | `online` / Last Will `offline` |
| `diagnostics` | JSON: Fehler, Freigabe, Frost, Ladebedarf, Sensorzähler, Position und PV |
| `config` | aktuell gültige Konfiguration als JSON, ohne Zugangsdaten |
| `config/set` | JSON-Teiländerung, **nicht retained** senden |
| `config/result` | nicht retained: `{"result":"ok"}` oder verständlicher Fehler |

`online` bedeutet MQTT verbunden, keine Aussage über fehlerfreien Heizbetrieb.
Die Ausgänge haben keine elektrischen Rückmeldekontakte. `mixer_position=-1`
bedeutet unbekannt, ansonsten geschätzter Anteil 0..1.

PV-Eingänge: `heizung/pv/power`, `surplus`, `grid_import`, `grid_export`;
Payload eine endliche Zahl 0..1.000.000 in **Watt**, nicht retained.
Nach 30 s ohne gültigen Empfang erscheint der jeweilige Wert in der Diagnose
als `null`. Reihenfolge im Diagnosearray entspricht dieser Aufzählung.
Daten werden schon empfangen, lösen aber keine Lastschaltung aus.

## Änderungen

Beispiel-Payload für `heizung/anlage/config/set`:

```json
{"mode":"auto","room_target":21,"boiler_target":50,"curve_slope":1.0}
```

Beispiel vollständiges tägliches Zeitprogramm:

```json
{"periods":[
 {"start_minute":360,"room_target":21},
 {"start_minute":510,"room_target":18},
 {"start_minute":990,"room_target":21},
 {"start_minute":1350,"room_target":17.5}
]}
```

Unbekannte Felder, falsche Typen, doppelte Schlüsselnamen, leere Objekte,
NUL-Bytes, ungültige Werte und widersprüchliche Grenzen werden abgewiesen.
`periods` muss bei Änderung vier vollständige Einträge enthalten.
Payload maximal 2048 Byte; MQTT-Fragmente werden zusammengebaut. Die Queue
fasst vier Befehle, persistente Änderungen höchstens alle 2 s. Retained Befehle
werden nicht ausgeführt. Änderungen sind absolute Zuweisungen und dadurch bei
QoS-1-Wiederholung idempotent; Wiederholungen können als zu häufig abgewiesen
werden. Es gibt kein Transaktions-ID-Protokoll.

Alle Felder eines Updates werden gemeinsam validiert und erst nach erfolgreichem
NVS-Commit übernommen. Ungültige Änderungen verändern weder aktive Konfiguration
noch NVS. Der Blob hat Version und Größenprüfung; NVS prüft seine Speicherdaten.
Für spätere Schemaänderungen ist eine explizite Migration erforderlich.

## Einstellwerte

| JSON-Feld | Standard | zulässiger Bereich |
|---|---:|---|
| `mode` | `auto` | `off`, `auto`, `manual` |
| `curve_zero` | 45 °C | 20..65 |
| `curve_slope` | 1 | 0..3 |
| `room_gain` | 3 | 0..10 |
| `room_target` | 21 °C | 5..30 |
| `unsynced_room` | 17,5 °C | 5..30 |
| `flow_min` | 25 °C | 10..40 |
| `flow_max` | 60 °C | 30..75 |
| `summer_temp` | 20 °C | 10..30 |
| `boiler_target` | 50 °C | 35..65 |
| `boiler_hysteresis` | 5 K | 1..15 |
| `kessel_max` | 85 °C | 60..95 |
| `kessel_margin` | 10 K | 3..20 |
| `burner_hysteresis` | 5 K | 1..15, kleiner als Kesselreserve |
| `dhw_lift` | 10 K | 5..20 |
| `frost_outside` | 3 °C | −5..10 |
| `frost_water` | 5 °C | 2..10 |
| `frost_flow` | 30 °C | 15..40, höchstens `flow_max` |
| `mixer_band` | 1 K | 0,3..5 |
| `burner_min_on_s` | 120 s | 10..1800, ganze Zahl |
| `burner_min_off_s` | 180 s | 10..1800, ganze Zahl |
| `pump_overrun_s` | 120 s | 0..900, ganze Zahl |
| `mixer_travel_s` | 240 s | 30..600, ganze Zahl |
| `mixer_dead_ms` | 1000 ms | 500..10000, ganze Zahl |
| `sensor_stale_s` | 5 s | 3..10, ganze Zahl |
| `dhw_priority` | `true` | boolesch |
| `buffer_enabled` | `false` | boolesch |
| `pv_enabled` | `false` | nur `false` bis Leistungsteil implementiert |

Zusätzliche Bedingungen: `flow_min < flow_max`,
`flow_max + kessel_margin <= kessel_max - 3`,
`boiler_target + dhw_lift <= kessel_max - 3`. Änderungen mehrerer abhängiger
Grenzen gemeinsam schicken. Standardwerte sind keine Auslegungsempfehlung.
Warmwasserhygiene/Legionellenprogramm ist nicht implementiert; eine erforderliche
Hygienefunktion muss unabhängig geplant werden.
