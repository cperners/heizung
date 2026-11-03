# Gaszähler über MQTT

Unter **System → Gaszähler** anmelden, Erfassung aktivieren und das MQTT-Topic exakt eintragen. Topic und Aktualitätshinweis sind zur Laufzeit änderbar; Speichern startet die Steuerung neu. Der Broker bleibt der bereits eingerichtete MQTT-Server. Das Topic muss sich vom Kachelofen-Topic unterscheiden.

Die Nachricht ist ein fortlaufender Gesamtzählerstand als einzelne Zahl in **m³**, etwa `12345.678`. Dezimalpunkt verwenden, keine Einheit und kein JSON. Der Zähler umfasst alle darüber versorgten Verbraucher. Es gibt keine Hochrechnung aus Brennerlaufzeit und keine Umrechnung in kWh.

## Tagesverbrauch

Die Tagesgrenze folgt der eingerichteten Wiener Ortszeit einschließlich Sommerzeit. Für eine Tagesbasis ist eine synchronisierte Gerätezeit nötig. Der erste frische Messwert jedes Tages bildet die Basis; angezeigt wird die Differenz zum letzten Gesamtstand. Die Anzeige ist deshalb ausdrücklich ein **Teilwert seit dem angegebenen Zeitpunkt**, kein exakt auf Mitternacht interpolierter Verbrauch. Der erste Wert ergibt null; erst folgende Änderungen ergeben Verbrauch.

Sinkt der Gesamtstand, beginnt eine neue Teilbasis und der Rücksetzungszähler steigt. Ein Topic-Wechsel beginnt ebenfalls eine neue Basis. Gesamtstand und Tagesbasis werden getrennt verwaltet. Die Basis wird sofort, der laufende Stand spätestens alle 15 Minuten gespeichert. Nach einem Neustart gleicht die nächste kumulative Messung fehlende Zwischenstände wieder aus. Ohne neue Nachricht wird der letzte gespeicherte Stand als veraltet angezeigt; die letzten Minuten vor einem Neustart können bis dahin fehlen.

## Manuelle Korrektur

Nach der Webanmeldung unter **Gas · Tagesbasis korrigieren** den bekannten Gesamtzählerstand zu Beginn der heutigen Erfassung eintragen. Er muss zwischen null und dem zuletzt verarbeiteten Gesamtstand liegen. Eine gültige Basis für den heutigen Tag muss bereits vorhanden sein. Die Korrektur wird gespeichert, startet die Steuerung neu und erzeugt keinen zusätzlichen Logeintrag. Der eingelesene Gesamtstand wird nicht verändert. Die angezeigte Startzeit bleibt die Zeit der ursprünglichen Tagesbasis.

## Empfang und Aktualität

Alte Retained-Nachrichten gelten nicht als aktuelle Messung. Die Quelle soll regelmäßig auch einen unveränderten Stand senden. Standardmäßig erscheint nach 3600 Sekunden ohne aktuelle Nachricht ein Hinweis; einstellbar sind 10 bis 86400 Sekunden. Fehlende oder ungültige Gasdaten beeinflussen die Heizungsregelung nicht.

Die lokale Prüfung umfasst Zahlenformat, Tageswechsel, Zählerrücksetzung, Korrekturgrenzen, fehlende Zeit, Retained-Nachrichten, Speichern, Neustart und fehlgeschlagene Schreibvorgänge. Browserbedienung und Empfang vom echten Gaszähler müssen am Gerät geprüft werden.
