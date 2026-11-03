<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/RESTWAERME.md -->

# Einstellbare Restwärmenutzung beim Ausschalten

Auf Nutzerwunsch für den Harreither Mono Star G15, Baujahr 1993: Startwert **30 °C**, maximale Laufzeit zunächst **1800 Sekunden (30 Minuten)**. Das ist eine gewählte Einstellung, keine bestätigte Herstellerfreigabe für diese Auskühlung. Dieser Hinweis steht auch direkt am Formular.

## Verhalten

Restwärmenutzung ist standardmäßig eingeschaltet. Sie startet einmalig beim Wechsel aus Automatik, Nur Heizung, Urlaub oder dem alten manuellen Heizmodus auf Aus, wenn unmittelbar vorher eine Heizkreispumpen- oder Brenneranforderung bestand, die Kesseltemperatur oberhalb des Abschaltwertes liegt und Kessel-/Vorlaufwerte gültig sind. Aus „Nur Boiler“ oder „Frostschutz“ wird sie nicht gestartet. Nach einem Neustart in Aus beginnt keine Restwärmenutzung.

Der Brenner wird sofort abgeschaltet und kann während Aus auch durch die Restwärmenutzung nicht neu angefordert werden. Die Boilerpumpe bleibt aus. Die Heizungspumpe läuft; der Mischer regelt nach dem letzten im Heizbetrieb berechneten Vorlauf-Sollwert. Dieser wird weiterhin durch Minimum und die Sollwertobergrenze begrenzt. Der Mischer wird also weder pauschal geschlossen noch pauschal vollständig geöffnet.

Die Restwärmenutzung endet beim Erreichen/Unterschreiten der Kessel-Abschalttemperatur oder bei Ablauf der maximalen Zeit. Danach endet auch der Heizkreispumpenlauf ohne zusätzliche normale Nachlaufzeit und der Mischer fährt Zu. Bleibt der Kessel warm, startet die Funktion in derselben Aus-Phase nicht erneut. Wie weit der Kessel tatsächlich abkühlt, hängt von Hydraulik und Wärmeabnahme ab; 30 °C werden nicht garantiert. Ein Rücklauffühler und eine Bestimmung des tatsächlichen nutzbaren Temperaturunterschieds sind nicht vorhanden.

Ein ungültiger/veralteter Kessel- oder Vorlaufwert, Übertemperatur, Deaktivierung der Funktion oder Wechsel der Betriebsart beendet die Restwärmenutzung. Bestehende Sicherheitsreaktionen haben Vorrang: Bei Übertemperatur kann die Heizkreispumpe zur Wärmeabfuhr trotz beendeter Restwärmenutzung weiterlaufen, während der Mischer schließt. Eine Abkühlung über die mechanisch tatsächlich vorhandenen Wärmewege ist damit noch nicht nachgewiesen.

Bei ausgeschalteter Restwärmefunktion gilt der bisherige zeitgesteuerte Pumpennachlauf mit geschlossenem Mischer. Die beiden Zeitwerte haben unterschiedliche Aufgaben: regulärer Pumpennachlauf und maximale Laufzeit der Restwärmenutzung.

## Bedienung und Speicherung

Der Bereich „Restwärmenutzung beim Ausschalten“ zeigt den aktiven Zustand und die verbleibende Zeit bis zum Zeitlimit. Einstellungen benötigen Anmeldung: einschalten/ausschalten, Abschalttemperatur 20–60 °C und maximale Laufzeit 60–7200 Sekunden. Änderungen werden ohne Neustart gespeichert; während aktiver Restwärmenutzung wirken Änderungen am Grenzwert und Zeitlimit bereits auf diesen Lauf. Das Zeitlimit wird weiterhin vom ursprünglichen Beginn gerechnet.

Die Einstellungen liegen in einem eigenen NVS-Namensraum und verändern die vorhandene Heizkonfiguration nicht. Einstellungssicherungen verwenden jetzt Schema 3 und enthalten die Funktion. Alte Sicherungen der Schemata 1 und 2 werden weiterhin gelesen; fehlende Restwärmeeinstellungen erhalten die neuen Startwerte aktiviert/30 °C/1800 Sekunden. Ein Firmware-Rückfall auf eine alte Version verwendet wieder deren bisherige Nachlauflogik.

MQTT-Diagnose enthält `rest_heat_active` und `rest_heat_remaining_s`. Web-Updates bleiben während tatsächlichem Pumpen-/Mischerbetrieb gesperrt. Falls nötig, Restwärmenutzung deaktivieren und das Stillstehen der Ausgänge abwarten, bevor ein Web-Update gestartet wird.

## Prüfung

Lokal geprüft: Start nur beim passenden Übergang, Brennersperre, Mischerregelung, Temperaturende, maximales Zeitlimit, Fühlerausfall, Übertemperaturvorrang, Deaktivieren, Betriebsartwechsel, kein erneuter Start bei erneut steigender Kesseltemperatur in Aus, kein Start nach Kaltstart sowie klassischer Nachlauf bei ausgeschalteter Funktion. Zusätzlich geprüft sind Validierung, Speicherfehler, Neustartpersistenz und Sicherungsimport. Ein Test an der realen Kessel-/Pumpen-/Mischer-Hydraulik steht aus.
