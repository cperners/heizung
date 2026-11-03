# Verifikationsstand – 27. Oktober 2026

## Lokal ausgeführt

`tests/run_host_tests.sh` übersetzt dieselben portablen C-Reglermodule wie die
Firmware mit C11, `-Wall -Wextra -Werror -pedantic` und führt Assertions aus.
Geprüft werden gültige/ungültige Konfiguration, Brennerlauf-/Stillstandszeit,
Sicherheitsabschaltung während Mindestlaufzeit, unabhängige Sensorfehler,
NaN/veraltete Werte, Warmwasservorrang, Heizkurve/Raumfallback, Zeitperioden über
Mitternacht, NTP-/Netzfehler, OFF/Frost/Sommer, Pumpennachlauf, Mischerreferenz,
Richtungswechsel auch über STOP, 10.000 wechselnde Mischerbefehle,
DS18B20-CRC/Temperaturdekodierung und Pufferverfügbarkeit. Zusätzlich wurden
20.000 wechselnde Reglerzyklen gegen Sicherheitsinvarianten und beide Wiener
Sommerzeitumstellungen 2026 mit derselben POSIX-Zeitzonenregel geprüft.

Ergebnis: PASS, auch mit AddressSanitizer und UndefinedBehaviorSanitizer
(`CFLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer -g' tests/run_host_tests.sh`).
Keine dabei gemeldeten Speicher- oder undefinierten Operationen. Diese Tests überprüfen Regelentscheidungen, keine elektrische
Funktion. Der Host-Test verwendet keinen ESP-IDF-Stub als Ersatz für einen
Firmware-Build.

## Neuer Web-Build

Der vollständige lokale Firmware-Build mit ESP-IDF v5.5.1 und der ARM64-P4-Toolchain
GCC 14.2.0 ist am 27. Oktober 2026 erfolgreich abgeschlossen worden.
`scripts/idf.sh build` erzeugt `build/heizungssteuerung.bin` (813.776 Byte),
Bootloader, Partitionstabelle und ELF. Die App-Partition hat 22 % Reserve.
Die Regelungstests wurden erneut erfolgreich ausgeführt. Die Buildkonfiguration
sperrt weiterhin die Lastausgänge; MQTT nutzt die dokumentierte Beispieladresse.

Die lokale Umgebung und VS-Code-Projekteinstellungen sind eingerichtet.
Reproduzierbare Schritte stehen in [Lokaler Build](LOKALER_BUILD.md).
Die GitHub-Prüfung wurde nicht ausgeführt. Keine Hardware wurde geflasht oder geprüft.

Vor Einsatz sind insbesondere GPIO-/Relaispegel, Boot/Reset, PHY-Adresse und
RMII-Takt am eigenen Board, reale 1-Wire-Timings unter Ethernetlast, lange
Sensorkabel, CRC-Störungen, MQTT-Fragmentierung/Reconnect/ACL/TLS,
NVS-Stromausfall beim Schreiben, NTP/DST auf dem Gerät, Task-Watchdog/Timeout,
Mischermechanik und die konkrete Sicherheitsreaktion der Hydraulik zu testen.

## Verwendete Primärquellen

- [Waveshare ESP32-P4-ETH](https://www.waveshare.com/wiki/ESP32-P4-ETH)
- [Waveshare offizieller Schaltplan](https://files.waveshare.com/wiki/ESP32-P4-ETH/ESP32-P4-ETH-datasheet.pdf)
- [ESP-IDF v5.5 Ethernet](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32p4/api-reference/network/esp_eth.html)
- [ESP-IDF v5.5.1 P4-EMAC-Konfigurationsheader](https://github.com/espressif/esp-idf/blob/v5.5.1/components/esp_eth/include/esp_eth_mac_esp.h)
- [Offizieller ESP-MQTT-API-Header](https://github.com/espressif/esp-mqtt/blob/v1.0.0/include/mqtt_client.h)

Quellen wurden für Pinbelegung und API-Abgleich verwendet; sie bestätigen keinen
Build dieses Projekts und keine Eignung für die konkrete Heizungsanlage.

Die Firmware enthält inzwischen HTTP-Webserver, geschützte Einstellungen,
MQTT-/NTP-Konfiguration und Webzugang. Aktueller Prüfstand und Einschränkungen
stehen in [Weboberfläche](WEB.md). Der neue Build wurde hier nicht auf das Board
übertragen; der Browser-Teststart war durch die Sandbox blockiert.
