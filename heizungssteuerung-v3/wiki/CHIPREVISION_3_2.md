<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/CHIPREVISION_3_2.md -->

# Vorhandenes Board: ESP32-P4 Revision 3.2

Beim Flashversuch hat esptool das Board als Revision 3.2 erkannt. Der vorherige
Build mit ESP-IDF v5.5.1 ist nur fuer Revisionen bis 1.99 geeignet. Der Abbruch
ist korrekt; niemals mit `--force` umgehen oder den Imageheader manipulieren.

Espressif empfiehlt fuer Revision 3.2 im 5.5-Zweig mindestens v5.5.5, insbesondere
wegen einer Watchdog-Aenderung bei externem 32-kHz-Quarz. Das Projekt wird deshalb
auf v5.5.5 aktualisiert und mit deaktivierter Alt-Revisionsauswahl fuer v3.x gebaut.
Mindestrevision ist 3.1; sie schliesst das vorhandene v3.2-Board ein.

Im VS-Code-Terminal nacheinander ausfuehren:

```bash
cd "/Users/christianpernerstorfer/Documents/Codex/heizungsteuerung V3"
scripts/setup-local.sh
scripts/idf.sh fullclean
scripts/idf.sh build
```

Nur wenn alle Schritte erfolgreich waren:

```bash
scripts/idf.sh -p /dev/cu.usbmodem5C940425091 flash
```

Bei einem Fehler im Setup oder Build nicht mit Flashen fortfahren; Meldung zur
Pruefung weitergeben. Die Setup-Datei aktualisiert die lokale unveraenderte
ESP-IDF-Installation samt Submodulen, Werkzeugen, Python-Paketen und VS-Code-Pfaden.
Sie bricht bei lokalen Aenderungen an ESP-IDF ab, statt diese zu ueberschreiben.
Der Buildwrapper blockiert die bisherige ESP-IDF-Version, damit keine weiteren
alten Images fuer dieses Board gebaut oder geflasht werden.

Status: Umstellung vorbereitet, Shell-/Python-Syntax geprueft. Installation und
Firmware-Build mit v5.5.5 wurden hier wegen eingeschraenktem Netzwerkzugriff noch
nicht ausgefuehrt. Lastausgaenge bleiben gesperrt; MQTT-Konfiguration bleibt erhalten.

Quellen:
- [Espressif: Kompatibilitaetshinweis Revision 3.2](https://documentation.espressif.com/AR2026-004_Compatibility_Advisory_for_ESP32-P4_Chip_Revision_v3.2_EN.html)
- [ESP-IDF v5.5.5 Revisionskonfiguration](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_hw_support/port/esp32p4/Kconfig.hw_support)
