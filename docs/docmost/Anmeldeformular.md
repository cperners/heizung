# Anmeldefelder erst nach Klick anzeigen

Stand: **2. November 2026**.

## Bedienung

Beim Oeffnen der Heizungswebseite sind weiterhin alle Mess- und Einstellwerte
sichtbar. Im Anmeldebereich wird zunaechst nur die Schaltflaeche **Anmelden**
angezeigt. Benutzername und Passwort sind ausgeblendet.

1. **Anmelden** anklicken. Die beiden Eingabefelder erscheinen; der Fokus steht
   im Benutzernamenfeld.
2. Benutzername und Passwort eingeben und im Formular **Anmelden** anklicken.
3. Bei erfolgreicher Anmeldung verschwinden die Felder. Die geschuetzten
   Einstellungen werden freigegeben und **Abmelden** wird angezeigt.
4. **Abbrechen** schliesst das Formular ohne Anmeldung, leert das Passwortfeld
   und zeigt wieder die einzelne Anmelden-Schaltflaeche.
5. Nach Abmeldung oder Ablauf der Sitzung sind die Felder wieder ausgeblendet.

Waehrend der Eingabe bleibt das geoeffnete Formular trotz regelmaessiger
Sitzungspruefung geoeffnet. Bei fehlerhaften Zugangsdaten bleibt es fuer eine
Korrektur offen. Der Browser kann seine normale Passwortverwaltung verwenden;
die Firmware liefert kein gespeichertes Passwort an die Webseite zurueck.

## Bestehende Zugriffsregeln

Dies ist eine Aenderung der Darstellung. Die serverseitige Anmeldung und die
Speicherpruefungen bleiben erhalten. Regelung und Betriebsart bleiben ohne
Anmeldung bedienbar; alle anderen geschuetzten Einstellungen benoetigen die
bestehende Anmeldung. Der bewusst offene OTA-Update-Zugang bleibt davon getrennt.

## Update und Pruefung

Die aktuelle Firmware unter dem festen Dateinamen `heizung-produktion-ota.bin`
hochladen und die Webseite neu laden. Dann zuerst den geschlossenen Zustand,
Anmelden, Abbrechen, erfolgreiche Anmeldung und Abmelden pruefen.

Build, JavaScript-Syntaxpruefung und lokale UI-Pruefung sind erfolgreich.
Die lokale Pruefung deckt auch das Offenbleiben waehrend der Sitzungsabfrage,
das Leeren des Passwortfelds und die unveraenderten frei bedienbaren Schalter ab.
**Die Darstellung und Bedienung mit dieser Aenderung sind am Geraet noch zu bestaetigen.**

## Technische Umsetzung

Die Eingabefelder liegen von Anfang an in einem mit `hidden` ausgeblendeten
Formular. Eine eigene Anmelden-Schaltflaeche oeffnet es. Sichtbarkeit und
Anmeldestatus werden getrennt verwaltet. Eine gezielte CSS-Regel stellt sicher,
dass die Flex-Darstellung des Formulars das `hidden`-Attribut nicht uebersteuert.

Betroffen sind `src/web/page.cpp` und `tests/host/test_web_access_ui.js`.
