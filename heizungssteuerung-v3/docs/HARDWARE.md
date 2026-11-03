# Anschluss und Boardprüfung

Die Pinbelegung wurde am 8. Oktober 2026 gegen den offiziellen
[Waveshare-Schaltplan](https://files.waveshare.com/wiki/ESP32-P4-ETH/ESP32-P4-ETH-datasheet.pdf)
und die [ESP-IDF-Ethernet-Dokumentation](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32p4/api-reference/network/esp_eth.html)
geprüft. Die tatsächliche Boardrevision, Bestückung und Steckernummerierung müssen
vor Anschluss vor Ort verglichen werden. Ein Schaltplanvergleich ersetzt keine
Messung am vorhandenen Board.

## Ethernet reserviert

| Funktion | GPIO |
|---|---:|
| CRS_DV | 28 |
| RXD0 / RXD1 | 29 / 30 |
| MDC / MDIO | 31 / 52 |
| TXD0 / TXD1 | 34 / 35 |
| TX_EN | 49 |
| Externer 50-MHz-Referenztakt vom PHY | 50 |
| PHY Reset | 51 |

Diese Pins dürfen nicht für Sensoren oder Lasten verwendet werden. Die in der
Spezifikation vorgeschlagenen Anwendungspins kollidieren mit diesen Netzen nicht;
eine Umbelegung ist dafür nicht erforderlich. IP101 wird automatisch gesucht
(`phy_addr=-1`); bei abweichender Bestückung lässt sich die Adresse in menuconfig
fest einstellen. Taktrichtung und Resetleitung sind für das Waveshare-Board gesetzt.
Andere ESP32-Boards haben andere RMII-Pins.

## Temperatursensoren

| Sensor | GPIO | Zustand |
|---|---:|---|
| Außen | 2 | aktiv |
| Raum | 3 | aktiv |
| Kessel | 4 | aktiv |
| Vorlauf | 5 | aktiv |
| Boiler | 6 | aktiv |
| Puffer oben | 19 | optional |
| Puffer mitte | 20 | optional |
| Puffer unten | 21 | optional |

Jeder Bus erhält **genau einen DS18B20**, einen eigenen 4,7-kΩ-Widerstand zwischen
DQ und 3,3 V und eine eigene Datenleitung. VDD an 3,3 V, GND an gemeinsame
Kleinspannungsmasse. Kein Pull-up nach 5 V und keine parasitäre Versorgung.
Der Treiber lehnt Parasite Power ab. Kabel und Pull-up müssen für die Leitungslänge
geeignet sein; lange Leitungen und Störungen sind praktisch zu prüfen.

GPIO20/21 werden auch am CSI-Kamerastecker geführt. Keine Kamera oder anderes
Gerät dort anschließen, wenn die Pufferfühler verwendet werden. Die Firmware
initialisiert auch die optionalen Sensorpins als freigegebene Open-Drain-Eingänge,
führt jedoch bei deaktiviertem Puffer keine Sensortransaktionen darauf aus.
Steckerpositionen anhand des eigenen Boards prüfen; GPIO-Nummer ist keine
Stecker-Pinnummer. Keine Display-/Kamera-/Audio-/SD-Treiber zusätzlich aktivieren,
ohne sämtliche Netze erneut zu prüfen.

## Relais und Leistungsteil

| Funktion | GPIO |
|---|---:|
| Brenneranforderung | 14 |
| Heizkreispumpe | 15 |
| Boilerpumpe | 16 |
| Mischer AUF | 17 |
| Mischer ZU | 18 |
| Heizstab, reserviert und dauerhaft AUS | 46 |

Standard ist aktiv HIGH. Für ein vollständig aktiv-LOW-Relaismodul gibt es eine
Buildoption; gemischte Polaritäten werden nicht unterstützt. Auch bei gesperrten
Lastausgängen wird der konfigurierte AUS-Pegel aktiv ausgegeben. **Polarität vor
Anschluss von Lasten prüfen.** GPIO46 wird bereits auf AUS initialisiert; auch dort
kein anderes Boardzubehör verwenden.

Nur passende 3,3-V-Treiber mit galvanischer Trennung einsetzen. Relaiskontakte,
Versorgung, Freilauf-/Entstörmaßnahmen und Netzspannungsinstallation passend
zur Last auslegen. GPIOs treiben keine Netzlast und keine Relaisspule direkt.
Externe Widerstände müssen bei Boot, Reset und unversorgtem ESP den AUS-Zustand
sicherstellen (Pull-down bei aktiv HIGH, geeigneter Pull-up bei aktiv LOW).
Firmware kann vor ihrem Start keine sicheren Pegel garantieren.

Für den Mischer zusätzlich eine **elektrische gegenseitige Verriegelung** der
Richtungsschütze und geeignete Endschalter verwenden. Die Software sperrt beide
Richtungen zentral und erzwingt Totzeit; sie schützt nicht gegen klebende Kontakte.
Position ist nur eine Laufzeitschätzung und hat keinen Rückmeldekontakt.

Die unabhängige Kesselsicherheitskette darf durch das Brenneranforderungsrelais
nicht überbrückt werden. Frostschutz ist bei fehlenden Sensoren, Stromausfall oder
gesperrten Ausgängen nicht gewährleistet. Anlagen mit Fußbodenheizung benötigen
einen unabhängigen Vorlaufbegrenzer und passende niedrigere Temperaturgrenzen.
