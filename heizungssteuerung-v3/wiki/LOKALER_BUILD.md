<!-- Wiki-Export vom 31.10.2026; Projektquelle: docs/LOKALER_BUILD.md -->

# Lokaler Build auf diesem Mac

Stand 27. Oktober 2026: ESP-IDF v5.5.1, native macOS-ARM64-Toolchain
riscv32-esp-elf GCC 14.2.0, Python 3.9.6 in eigener virtueller Umgebung,
CMake 3.30.2 und Ninja 1.12.1 sind im Projekt unter `.local/` installiert.
Die Installation verändert keine globale Python- oder Shellkonfiguration.
`.local/` ist von Versionsverwaltung und VS-Code-Suche ausgeschlossen.

## Bauen

Im Projektordner:

```sh
scripts/idf.sh build
```

Das Skript aktiviert automatisch die lokale Entwicklungsumgebung. Für Einstellungen:

```sh
scripts/idf.sh menuconfig
```

Die Firmware entsteht unter `build/heizungssteuerung.bin`, außerdem werden
Bootloader, Partitionstabelle, ELF und `compile_commands.json` erzeugt.
Der zuletzt geprüfte Build umfasst 813.776 Byte App-Code und passt in die
1-MiB-App-Partition. Der konfigurierte Flashbereich beträgt 2 MiB; die tatsächliche
Flashbestückung vor dem ersten Flashen am Board prüfen. Kein Flashvorgang wurde ausgeführt.

Lastausgänge sind gesperrt (`CONFIG_HEATING_ACTUATORS_ENABLED` nicht gesetzt).
Die MQTT-Adresse ist auf `mqtt://192.168.0.1:1883` gesetzt; Benutzername und Passwort sind auf Nutzerwunsch leer.
Diese Werte vor Inbetriebnahme passend einstellen; Lastfreigabe erst nach den
Prüfungen in der Inbetriebnahmeanleitung aktivieren.

## VS Code

Die Erweiterung „Espressif IDF“ 2.3.0 war bereits installiert. Die lokale Installation
ist über `.local/espressif/esp_idf.json` registriert; `.vscode/settings.json` wählt sie
für dieses Projekt aus. IntelliSense verwendet die generierte Compiler-Datenbank.

Über **Terminal → Buildaufgabe ausführen** bzw. **⇧⌘B** steht die Aufgabe
„Heizung: Firmware bauen“ bereit. Zusätzlich gibt es „Heizung: Regelungstests“.
Beim letzten UI-Abgleich befand sich der Projektordner noch im eingeschränkten
VS-Code-Modus. Falls dieser weiterhin angezeigt wird, muss der Nutzer gezielt
für diesen Projektordner Vertrauen freigeben, bevor VS Code Tasks/Erweiterungen
ausführt. Die globale Vertrauensprüfung wurde nicht deaktiviert.
Der oben beschriebene lokale Build funktioniert unabhängig davon.

## Neu einrichten

Nach Entfernen der lokalen Werkzeuge oder auf einem gleich eingerichteten Mac:

```sh
scripts/setup-local.sh
scripts/idf.sh set-target esp32p4
scripts/idf.sh build
```

Dafür werden Netzwerkzugriff, Git, Python ab 3.9 und Apple Command Line Tools benötigt.
Das Setup installiert nur Werkzeuge für ESP32-P4 sowie CMake/Ninja und erzeugt
die VS-Code-Pfade neu. Nach Verschieben des Projektordners erneut
`python3 scripts/configure-vscode.py` ausführen und das Buildverzeichnis mit
`scripts/idf.sh fullclean` neu aufbauen, weil CMake absolute Pfade speichert.

Der optionale IDF Component Manager ist im Buildskript und in den VS-Code-Variablen
über die vorgesehene Einstellung `IDF_COMPONENT_MANAGER=0` deaktiviert. Das Projekt
benötigt keine externen Registry-Komponenten. Damit ist keine Prozesslistenabfrage
dieses optionalen Werkzeugs nötig, die in der Codex-Sandbox blockiert wurde.
Die ESP-IDF-Quellen selbst wurden nicht dafür verändert.
