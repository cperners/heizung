# Heizungssteuerung – Projektwiki

Stand: 3. November 2026. Waveshare ESP32-P4-ETH, Chiprevision 3.2, ESP-IDF 5.5.5. Dieses Wiki beschreibt den Projektstand; es ist kein Export der aktuellen Gerätekonfiguration.

[Aktueller Stand: Display erfolgreich angebunden](STATUS_2026-11-03.md)

## Bedienung und Betrieb

- [Weboberfläche und Anmeldung](WEB.md)
- [Betriebsarten](BETRIEBSARTEN.md)
- [Regelungslogik](REGELUNG.md)
- [Restwärmenutzung](RESTWAERME.md)
- [Kachelofen über MQTT](KACHELOFEN.md)
- [Gaszähler und Tagesverbrauch](GASZAEHLER.md)
- [MQTT-Basistopic](MQTT_BASISTOPIC.md)
- [Einstellungssicherung und Temperaturverlauf](SICHERUNG_VERLAUF.md)

## Anschluss, Wartung und Entwicklung

- [Pinbelegung und Anschlüsse](HARDWARE.md)
- [Firmwareupdate per USB und Webseite](FIRMWARE_UPDATE.md)
- [Lokaler Build](LOKALER_BUILD.md)
- [Chiprevision 3.2](CHIPREVISION_3_2.md)
- [Konfigurationsreferenz](KONFIGURATION.md)
- [Inbetriebnahme](INBETRIEBNAHME.md)
- [Prüfstand und offene Vorhaben](PROJEKTSTAND.md)
- [Touchdisplay](TOUCH_DISPLAY.md)

## Aktuelle Vereinbarungen

Der vorhandene Mischerantrieb bleibt ein ESBE Typ 86. Die vollständige Laufzeit beträgt **240 Sekunden**; der Nutzer hat sie bereits auf der Webseite gespeichert. Endlagenrückmeldungen sind nicht eingerichtet. Die Position wird aus der Laufzeit geschätzt.

Die Wärmenutzung beim Ausschalten endet standardmäßig bei 30 °C Kesseltemperatur oder nach 30 Minuten. Beide Grenzen sind einstellbar. Die Temperatur ist eine Nutzervorgabe, keine bestätigte Herstellerfreigabe für den Harreither Mono Star G15 von 1993.

Passwörter und Firmwaredateien sind nicht Bestandteil dieses Wiki-Pakets.

[Alle Einstellungen am Touchdisplay und Bildschirmruhe](TOUCH_EINSTELLUNGEN.md).
