# MQTT-Wiederverbindung nach einem Router-Neustart

Nach einem Router-Neustart war die ESP32-Webseite wieder erreichbar, während
MQTT ausfiel. Der bisherige Verbindungsversuch konnte verloren gehen, wenn
die MQTT-Bibliothek die alte Verbindung noch als aktiv behandelte.

Die Hauptschleife erkennt einen WLAN- oder IP-Verlust auch dann, wenn der
ESP32 zwischen zwei Durchläufen bereits wieder eine IP-Adresse bekommen hat.
Sie beendet die alte MQTT-Verbindung und versucht bei verfügbarer IP erneut,
den Broker zu erreichen. Solange der Broker nicht erreichbar ist, folgen
weitere Versuche. Zwischen den Versuchen liegen mindestens zwei Sekunden.
Ein Verbindungsaufbau ohne Ergebnis wird nach 30 Sekunden abgebrochen.
Nach erfolgreicher Anmeldung abonniert der ESP32 seine MQTT-Themen erneut.

## Prüfstand

Firmware-Build und lokale Tests für schnellen WLAN-Wechsel, verspätet
erreichbaren Broker und fehlenden Trennungsrückruf waren erfolgreich.
Die lokale Prüfung der WLAN- und MQTT-Einstellungen war ebenfalls erfolgreich.

Der Router-Neustart mit laufendem ESP32 ist noch am Gerät zu prüfen.
Nach dem Aufspielen der Firmware den Router neu starten und beobachten,
ob nach der WLAN-Rückkehr wieder MQTT-Werte eintreffen und eingehende
MQTT-Daten verarbeitet werden.
