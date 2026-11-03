# Firmware per USB und Webseite aktualisieren

Beide Wege bleiben verfügbar. Die vorhandene NVS liegt unverändert bei 0x9000, die Basisanwendung weiterhin bei 0x10000. Die neue Tabelle reserviert je 2 MiB für Basisanwendung und zwei OTA-Anwendungen sowie OTA-Auswahldaten. Das vorhandene Waveshare-Board hat laut Flash-Erkennung 32 MiB; die Firmware verwendet diese Größe.

## Einmalige Einrichtung per USB

1. Die vorhandene Einstellungssicherung herunterladen.
2. Den neuen Build mit dem vollständigen Standard-Flash übertragen:

```sh
scripts/idf.sh -p /dev/cu.usbmodem5C940425091 flash
```

Dabei werden Bootloader, Partitionstabelle, Basisanwendung und initiale OTA-Auswahldaten übertragen. **Kein erase-flash nötig.** Die Konfiguration bleibt an derselben Adresse. Nach dem Neustart Webseite neu laden und anmelden. Die vollständigen Flash-Argumente enthalten auch `ota_data_initial.bin` bei 0x610000.

## Web-Update

Unter „Firmware aktualisieren“ erscheint der laufende Build und Speicherplatz.

1. Betriebsart **Aus** wählen und Pumpennachlauf abwarten. Brenner, Pumpen und Mischer müssen stillstehen.
2. Anmelden und ausschließlich die passende **build/heizungssteuerung.bin** auswählen. Keine Bootloader-, Partitions- oder zusammengeführte Flash-Datei.
3. Installation bestätigen. Versorgung während Upload, Neustart und Teststart eingeschaltet lassen.
4. Nach Neustart bis zu 45 Sekunden Teststart abwarten. Der Status wechselt anschließend von Teststart zu gestartet.

Der Upload benötigt Anmeldung und den aktuellen CSRF-Token. Die Datei wird gestreamt statt vollständig im RAM gehalten. Vor Flashzugriff werden Größe, ESP32-P4, Chiprevision, Projektname und das Hash-Kennzeichen im Dateikopf geprüft. ESP-IDF prüft das vollständige Bild am Ende. Erst danach wird es als nächste Startpartition gewählt. Das ist eine Integritätsprüfung; eine digitale Herstellersignatur ist nicht eingerichtet. Nur selbst erstellte/vertraute Firmware verwenden.

Nach dem ersten gültigen Dateikopf werden die Ausgänge bis zum Neustart verriegelt. Bei einem abgebrochenen Upload bleibt die vorherige Startauswahl erhalten; der Server versucht einen Neustart nach drei Sekunden. Bei Verbindungsabbruch am Browser den Gerätestatus prüfen und gegebenenfalls manuell neu starten. Ein Wiederholen des Uploads während des Neustarts ist nicht sinnvoll.

## Teststart und Rückfall

Ein Web-Update startet zunächst mit ausstehender Bestätigung. Nach 45 Sekunden bestätigt die neue Anwendung sich nur, wenn die Initialisierung bis Webserver/Grunddiensten abgeschlossen ist, die Regler-Task aktuell läuft und kein Ausgangs-Timeout vorliegt. Nicht angeschlossene Sensoren, fehlende MQTT-Verbindung oder fehlende Zeit allein verhindern diese technische Startprüfung nicht. Sie ist kein Funktionstest der angeschlossenen Heizung.

Schlägt die Startprüfung fehl, startet das Gerät neu. Der ESP-IDF-Bootloader verwirft die unbestätigte Version und wählt eine vorherige gültige Anwendung, beim ersten OTA gegebenenfalls die Basisversion. Ein Absturz oder Reset vor der Bestätigung kann ebenfalls den Rückfall auslösen. Nach erfolgter Bestätigung erfolgt kein automatischer Rückfall für beliebige spätere Fehler. NVS-Einstellungen werden durch Firmware-Rückfall nicht zurückgesetzt; künftige Konfigurationsmigrationen müssen deshalb rückwärtsverträglich bleiben.

## Weitere USB-Updates

Der bisherige vollständige `flash`-Befehl bleibt gültig und setzt die OTA-Auswahl auf die neu geflashte Basisversion zurück. Die OTA-Slots werden dabei nicht als aktive Version ausgewählt. Ein bloßes `app-flash` setzt die Auswahl nicht zurück und ist deshalb nach Web-Updates nicht der empfohlene USB-Weg.

## Verifiziert / noch offen

Lokal geprüft: Dateikopf und Projekt-/Chip-/Revisionsprüfung, fragmentierter Stream, Übergröße, aktive Betriebsart/Pumpe, ausstehende Startbestätigung, Abbruch, Flash-Schreibfehler, Bildprüfungsfehler, fehlgeschlagene Startauswahl und Teststartentscheidung. Die Tests für Upload und Bestätigung verwenden ausdrücklich Flash-/HTTP-Doubles; der vollständige Firmware-Build prüft die echten ESP-IDF-APIs. Die erzeugte Partitionstabelle und Flash-Liste wurden auf beibehaltene NVS-/Basisadressen und Rücksetzung der OTA-Auswahl geprüft.

Der erste USB-Umstieg, echter Web-Upload, Stromausfall und tatsächlicher Bootloader-Rückfall müssen am Board geprüft werden. Ein funktionierender USB-Zugang bleibt der Wiederherstellungsweg.
