# Dynamische Heizkurvengrafik

Unter den Temperaturkacheln befindet sich eine eigene Kachel "Heizkurve"
ueber die gesamte verfuegbare Breite. Die Grafik ist auf- und zuklappbar.
Sie zeigt die gespeicherten Einstellungen ohne Anmeldung. Die Bearbeitung der
Heizkurveneinstellungen bleibt wie bisher durch die Anmeldung geschuetzt.

- Waagrechte Achse: Aussentemperatur in Grad Celsius. Auf breiten Anzeigen
  Temperaturmarken in 5-Grad-Schritten mit dezenten senkrechten Rasterlinien;
  auf schmalen Anzeigen automatisch groessere Beschriftungsabstaende.
- Senkrechte Achse: Vorlauftemperatur in Grad Celsius.
- Gruene Linie: gespeicherte Heizkurve mit der gleichen Potenzformel wie die Regelung.
- Gruener Punkt: aktuell von der Steuerung berechneter Vorlauf-Sollwert.
- Pastellrotes Quadrat: gemessene Vorlauftemperatur bei der aktuellen Aussentemperatur.
- Gestrichelte senkrechte Linie: aktuelle Aussentemperatur.

Die Achsen passen sich an die Parameter und die Messwerte an. Dadurch bleibt
auch eine gemessene Vorlauftemperatur oberhalb des eingestellten Maximums sichtbar.
Das Maximum begrenzt den berechneten Sollwert, nicht unmittelbar die Isttemperatur.

Die Kurve folgt der aktiven Raum-Solltemperatur, dem gespeicherten Vorlaufmaximum,
der Auslegungs-Aussentemperatur und dem gespeicherten Faktor. Noch nicht gespeicherte
Tastenaenderungen werden nicht als aktive Einstellungen dargestellt. Nach dem
Speichern, einem Tag-/Nachtwechsel oder einer neuen Messung erfolgt die Aktualisierung
ueber die bestehende sekundenweise Statusabfrage. Es gibt keine zusaetzlichen Abfragen,
externen Diagrammbibliotheken oder neue Steuerungsbefehle.

Ist die Heizkurvenberechnung nicht aktiv oder nicht verfuegbar, bleibt die
Parameterkurve sichtbar. Bei Aus, Nur Boiler und Urlaub basiert diese Vorschau
auf der eingestellten Tag-/Nacht-Raumtemperatur, da dort kein aktiver
Heizkurven-Raumsollwert vorliegen muss. Der aktuelle Sollwertpunkt wird ausgeblendet und
ein Hinweis angezeigt. Bei ungueltigem Aussenfuehler fehlt die aktuelle Markierung;
bei ungueltigem Vorlauffuehler fehlt das Ist-Quadrat. Ungueltige Kurvenparameter und
Verbindungsfehler werden als nicht verfuegbar angezeigt. Die Grafik ist eine Anzeige,
keine Freigabe- oder Sicherheitsfunktion und kein historischer Temperaturverlauf.

Der Browser berechnet die Linie aus dem numerischen Modell im Statusobjekt
`heatingCurve`. Der Sollwertpunkt stammt aus dem echten Firmware-Sollwert `target`.
Die Heizungs-, Pumpen- und Mischerregelung wird dadurch nicht veraendert.

Lokal geprueft sind Kurvenform und Grenzen, Darstellungscode fuer schmale und breite
Kacheln, Statusaktualisierung, Wiederverwendung unveraenderter Grafiken sowie
ungueltige und veraltete Daten. Build und bestehende Funktionspruefungen sind erfolgreich. Die Darstellung und Aktualisierung auf dem ESP32 und iPhone sind noch offen.

## Formel und Kurvenform

Mit R = Raum-Solltemperatur, A = Aussentemperatur, Vmax = Vorlaufmaximum,
Amin = Auslegungs-Aussentemperatur und n = Kurvenfaktor:

```text
q = begrenze((R - A) / (R - Amin), 0, 1)
Vorlauf-Soll = R + (Vmax - R) * q^(1/n)
```

Die Firmware schneidet den berechneten Wert auf zwei Nachkommastellen ab.
Bei `n = 1` ist der Verlauf zwischen den Endpunkten linear, bei `n > 1`
gekruemmt. Unterhalb von Amin bleibt er bei Vmax, oberhalb von R bei R.
R aendert sich mit der wirksamen Tag-/Nacht-Solltemperatur. Das allein bedeutet
nicht, dass die Heizung freigegeben ist: Pumpen-, Betriebsart- und Sensorsperren
werden weiterhin gesondert ausgewertet.
