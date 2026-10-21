#include "page.h"

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>
/* Darstellung der Heizungsuebersicht */
* { box-sizing: border-box; }
html { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #20343c; text-align: left; }
body { margin: 0; background: #f1f5f6; line-height: 1.5; }
.topnav { background: #163b45; padding: 14px 20px; border-bottom: 4px solid #36b6a5; }
.topnav h1 { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 8px 12px; max-width: 1120px; margin: 0 auto; font-size: clamp(.9rem, 2.5vw, 1.4rem); line-height: 1.4; color: white; white-space: normal; }
.topnav h1 > span, .topnav time { min-width: 0; max-width: 100%; overflow-wrap: anywhere; }
@media (max-width: 540px) { .topnav h1 { flex-direction: column; align-items: flex-start; } }
.topnav time { font-size: inherit; font-weight: inherit; }
.topnav hr { border: 0; border-top: 1px solid #ffffff30; margin: 10px 0; }
.layout { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 16px; max-width: 1168px; margin: 20px auto; padding: 0 24px; }
.layout > p, body > p { display: none; }
.layout > div { min-width: 0; padding: 20px; border: 1px solid #dce5e8; border-radius: 16px; background: white; box-shadow: 0 4px 16px #163b4508; overflow-wrap: anywhere; }
.layout > .content { padding: 0; }
.content { margin: 0; max-width: none; }
.card { padding: 22px; border-radius: 16px; background: white; box-shadow: none; }
h3 { margin: 0 0 12px; color: #20343c; font-size: 1rem; font-weight: 600; }
.state { margin: 12px 0; font-size: 1.1rem; color: #536b74; }
.state span { display: inline-block; padding: 4px 12px; border-radius: 20px; background: #edf3f5; color: #20343c; font-weight: 700; }
p { font-size: 1rem; margin: 14px 0; }
.dht-labels { display: block; font-size: .9rem; color: #607681; padding-bottom: 8px; }
#troom, #taussen, #tkessel, #tvorlauf, #tboiler, #tkachelofen { font-size: 1.8rem; font-weight: 700; letter-spacing: -.04em; color: #163b45; }
.units { font-size: .9rem; color: #607681; }
.temperature-trend { display: inline-block; margin-left: 6px; font-size: 1.3rem; font-weight: 700; color: #607681; vertical-align: baseline; }
.temperature-trend.up { color: #c2410c; }
.temperature-trend.down { color: #0369a1; }
strong { display: block; margin-top: 8px; color: #14796c; font-size: 1rem; }
.button, input[type="submit"] { border: 0; border-radius: 9px; background: #14796c; color: white; padding: 10px 18px; font: inherit; font-weight: 600; cursor: pointer; }
.button:hover, input[type="submit"]:hover { background: #105f55; }
button:focus-visible, input:focus-visible { outline: 3px solid #36b6a5; outline-offset: 3px; }
input[type="number"] { border: 1px solid #bdcdd3; border-radius: 7px; padding: 8px; margin: 5px 0 10px; font: inherit; }
input[type="range"] { width: 100%; accent-color: #14796c; }
form { line-height: 2; font-size: .9rem; }
@media (max-width: 540px) { .layout { grid-template-columns: 1fr; padding: 0 16px; gap: 12px; margin: 16px auto; } .topnav { padding: 20px 16px; } }

/* Bedientasten fuer die Kachelofen-Schaltschwellen */
.threshold-card { grid-column: span 1; }
@media (max-width: 540px) { .threshold-card { grid-column: span 1; } }
.threshold-form { line-height: 1.5; }
.threshold-form label { display: block; margin: 18px 0 8px; font-size: .85rem; font-weight: 600; color: #536b74; }
.threshold-row { display: grid; grid-template-columns: 32px minmax(0, 1fr) 32px; gap: 4px; align-items: center; }
.threshold-row button { height: 38px; border: 1px solid #b9d7d2; border-radius: 9px; background: #eaf5f2; color: #126a5f; font: inherit; font-size: 1.4rem; cursor: pointer; }
.threshold-row button:hover { background: #d8eee7; }
.threshold-value { display: flex; align-items: center; justify-content: center; gap: 3px; min-width: 0; white-space: nowrap; }
.threshold-value input[type="number"] { width: 5ch; min-width: 0; max-width: 100%; flex: 0 1 5ch; margin: 0; padding: 8px 0; border: 0; background: transparent; text-align: center; font-size: 1.2rem; font-weight: 700; color: #163b45; appearance: textfield; -moz-appearance: textfield; }
.threshold-value input::-webkit-inner-spin-button, .threshold-value input::-webkit-outer-spin-button { appearance: none; margin: 0; }
.threshold-value span { flex: 0 0 auto; white-space: nowrap; color: #607681; font-size: .9rem; }
.threshold-hint { font-size: .78rem; color: #607681; margin: 16px 0; }
.threshold-save { width: 100%; padding: 12px; border: 0; border-radius: 9px; background: #14796c; color: white; font: inherit; font-weight: 600; cursor: pointer; }
.threshold-save:disabled { opacity: .5; cursor: default; }

.output-status {display:grid;gap:12px;}
.output-row {display:flex;flex-direction:row-reverse;align-items:center;justify-content:flex-end;gap:12px;}
.output-indicator {min-width:62px;flex-shrink:0;}
.output-led {display:inline-block;width:16px;height:16px;border-radius:50%;background:#94a3b8;box-shadow:inset 0 0 3px #0005;flex-shrink:0;}
.output-led.on {background:#16a34a;box-shadow:0 0 6px #16a34a66;}
.output-led.off {background:#dc2626;}
.output-indicator {display:flex;align-items:center;gap:8px;}
#boiler-delta { color: #607681; font-weight: 600; }
#boiler-delta.warmer { color: #d98888; }
#boiler-delta.cooler { color: #7a9ecb; }
#burner-flame { width: 28px; height: 32px; color: #929ba1; vertical-align: middle; margin-right: 6px; }
#burner-flame.on { color: #d98888; }
#heating-pump-icon, #boiler-pump-icon { width: 30px; height: 30px; color: #929ba1; vertical-align: middle; margin-right: 6px; }
#heating-pump-icon.on, #boiler-pump-icon.on { color: #69a886; }
.temperature-line { display: flex; align-items: center; flex-wrap: wrap; gap: 4px; }
.equipment-status { display: flex; align-items: center; gap: 10px; margin-top: 12px; }
#mixer-symbol { width: 32px; height: 32px; color: #929ba1; vertical-align: middle; }
#mixer-symbol.moving { color: #69a886; }
.mixer-status { font-size: .9rem; color: #607681; }
</style>
<title>ESP32 Heizungs-Server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" 
        integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAzMiAzMiI+PHJlY3Qgd2lkdGg9IjMyIiBoZWlnaHQ9IjMyIiByeD0iNyIgZmlsbD0iIzE2M2I0NSIvPjxwYXRoIGQ9Ik0xNyAzYzEgNi01IDctNCAxMiAyLTEgMy0zIDMtNSA1IDQgOSA4IDcgMTQtMSA0LTQgNi04IDYtNSAwLTktNC05LTkgMC01IDQtOCA1LTExIDAgNCAxIDUgMiA2LTEtNSA1LTcgNC0xM3oiIGZpbGw9IiNmYmJmMjQiLz48cGF0aCBkPSJNMTYgMTdjMSAzLTMgNC0zIDcgMCAyIDEgNCAzIDRzNC0yIDQtNGMwLTMtMi00LTQtN3oiIGZpbGw9IiNmYjkyM2MiLz48L3N2Zz4K">
</head>
<body>
  <div class="topnav">
    <h1><span>ESP32 Heizung</span><span style="font-size:.65em;color:#fef08a;">Laufzeit: <span id="uptime">--</span></span><span style="font-size:.65em;color:#fef08a;" title="Anteil der Ticks ohne Leerlauf, keine exakte CPU-Zeitmessung">CPU-Sch&auml;tzung: <span id="cpu-load">--</span></span><time id="header-clock">%WEBTIME%</time></h1>
  </div>
  <section class="layout">
  <div>
    <span class="dht-labels">K&uuml;che</span> 
    <div class="temperature-line">
    <svg viewBox="0 0 16 24" width="16" height="24" style="color:#059e8a;vertical-align:middle;margin-right:5px" role="img" aria-label="Temperatur"><path d="M5 14.5V5a3 3 0 0 1 6 0v9.5a5 5 0 1 1-6 0Z" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M8 7v10" stroke="currentColor" stroke-width="2"/><circle cx="8" cy="18" r="2.5" fill="currentColor"/></svg>
    <span id="troom">%TROOM%</span>
    <sup class="units">&deg;C</sup>
    </div>
  </div>
  <div>
    <span class="dht-labels">Aussen</span> 
    <div class="temperature-line">
    <svg viewBox="0 0 16 24" width="16" height="24" style="color:#059e8a;vertical-align:middle;margin-right:5px" role="img" aria-label="Temperatur"><path d="M5 14.5V5a3 3 0 0 1 6 0v9.5a5 5 0 1 1-6 0Z" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M8 7v10" stroke="currentColor" stroke-width="2"/><circle cx="8" cy="18" r="2.5" fill="currentColor"/></svg>
    <span id="taussen">%TAUSSEN%</span>
    <sup class="units">&deg;C</sup>
    </div>
  </div>
  <div>
    <span class="dht-labels">Gaskessel</span> 
    <div class="temperature-line">
    <svg viewBox="0 0 16 24" width="16" height="24" style="color:#059e8a;vertical-align:middle;margin-right:5px" role="img" aria-label="Temperatur"><path d="M5 14.5V5a3 3 0 0 1 6 0v9.5a5 5 0 1 1-6 0Z" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M8 7v10" stroke="currentColor" stroke-width="2"/><circle cx="8" cy="18" r="2.5" fill="currentColor"/></svg>
    <span id="tkessel">%TKESSEL%</span>
    <sup class="units">&deg;C</sup>
    </div>
    <div class="equipment-status"><svg id="burner-flame" viewBox="0 0 24 24" role="img" aria-label="Brennerstatus wird geladen"><title>Brennerstatus wird geladen</title><path fill="currentColor" d="M13.5 1.5c.5 5-3.5 5.5-3.5 9 0 1 .5 1.8 1.2 2.3-.2-2.3 1.5-3.3 2.3-4.8 3 2.2 5 5 5 7.8a6.5 6.5 0 0 1-13 0c0-3.5 2-6.4 4.5-9-.2 2.5.4 3.5.4 3.5C10 6.5 14 5 13.5 1.5Z"/></svg><svg id="heating-pump-icon" viewBox="0 0 24 24" role="img" aria-label="Heizungspumpenstatus wird geladen"><title>Heizungspumpenstatus wird geladen</title><g fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M2 10h4m12 4h4M2 8v4m20 0v4M8 19l-1 3h10l-1-3"/><circle cx="12" cy="12" r="7"/></g><path fill="currentColor" d="M10 8l6 4-6 4Z"/></svg></div>
    <div id="brennersperre" role="status">%BRENNERSPERRE%</div>
  </div>
  <div>
    <span class="dht-labels">Vorlauf</span> 
    <div class="temperature-line">
    <svg viewBox="0 0 16 24" width="16" height="24" style="color:#059e8a;vertical-align:middle;margin-right:5px" role="img" aria-label="Temperatur"><path d="M5 14.5V5a3 3 0 0 1 6 0v9.5a5 5 0 1 1-6 0Z" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M8 7v10" stroke="currentColor" stroke-width="2"/><circle cx="8" cy="18" r="2.5" fill="currentColor"/></svg>
    <span id="tvorlauf">%TVORLAUF%</span>
    <sup class="units">&deg;C</sup>
    </div>
    <div class="equipment-status mixer-status">
      <svg id="mixer-symbol" viewBox="0 0 32 32" role="img" aria-label="Mischerstatus wird geladen"><title>Mischerstatus wird geladen</title><g fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M2 13h7m14 0h7M16 20v10M1 9v8m30-8v8M12 31h8"/><path d="m9 7 14 12V7L9 19ZM16 13v7M16 7V2m-5 0h10"/></g></svg>
      <span id="mixer-direction">Wird geladen</span>
    </div>
    <form id="flow-limit" class="threshold-form" action="/vorlaufmaximum" method="post">
      <label for="flow-max">Maximale Vorlauftemperatur</label>
      <div class="threshold-row">
        <button type="button" aria-label="Maximale Vorlauftemperatur senken" onclick="document.getElementById('flow-max').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="flow-max" name="maximum" type="number" min="0" max="100" step="0.1" value="%VORLAUFMAX%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Maximale Vorlauftemperatur erh&ouml;hen" onclick="document.getElementById('flow-max').stepUp()">+</button>
      </div>
      <small>Begrenzt den Sollwert bei Au&szlig;entemperaturregelung.</small>
      <label for="flow-curve">Kurvensteilheit</label>
      <div class="threshold-row">
        <button type="button" aria-label="Kurvensteilheit senken" onclick="document.getElementById('flow-curve').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="flow-curve" name="steilheit" type="number" min="0.1" max="10" step="0.1" value="%KURVENSTEILHEIT%" readonly required></div>
        <button type="button" aria-label="Kurvensteilheit erh&ouml;hen" onclick="document.getElementById('flow-curve').stepUp()">+</button>
      </div>
      <p id="flow-save-status" class="threshold-hint" role="status">&Auml;nderungen erst mit Speichern &uuml;bernehmen.</p>
      <button id="flow-save" class="threshold-save" type="submit">Heizkurveneinstellungen speichern</button>
    </form>
  </div>
  <div>
    <span class="dht-labels">Boiler</span> 
    <div class="temperature-line">
    <svg viewBox="0 0 16 24" width="16" height="24" style="color:#059e8a;vertical-align:middle;margin-right:5px" role="img" aria-label="Temperatur"><path d="M5 14.5V5a3 3 0 0 1 6 0v9.5a5 5 0 1 1-6 0Z" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M8 7v10" stroke="currentColor" stroke-width="2"/><circle cx="8" cy="18" r="2.5" fill="currentColor"/></svg>
    <span id="tboiler">%TBOILER%</span>
    <sup class="units">&deg;C</sup>
    </div>
    <div class="equipment-status"><svg id="boiler-pump-icon" viewBox="0 0 24 24" role="img" aria-label="Boilerpumpenstatus wird geladen"><title>Boilerpumpenstatus wird geladen</title><g fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M2 10h4m12 4h4M2 8v4m20 0v4M8 19l-1 3h10l-1-3"/><circle cx="12" cy="12" r="7"/></g><path fill="currentColor" d="M10 8l6 4-6 4Z"/></svg></div>
    <p style="font-size:.9rem;">Differenz Kessel &minus; Boiler: <span id="boiler-delta">%BOILERDELTA%</span></p>
    <form id="boiler-setpoint" class="threshold-form" action="/boilertemperatur" method="post">
      <label for="boiler-soll">Boiler-Solltemperatur</label>
      <div class="threshold-row">
        <button type="button" aria-label="Boiler-Solltemperatur senken" onclick="document.getElementById('boiler-soll').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="boiler-soll" name="soll" type="number" min="0" max="100" step="0.1" value="%BOILERSOLL%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Boiler-Solltemperatur erh&ouml;hen" onclick="document.getElementById('boiler-soll').stepUp()">+</button>
      </div>
      <label for="boiler-diff">Einschaltdifferenz</label>
      <div class="threshold-row">
        <button type="button" aria-label="Einschaltdifferenz senken" onclick="document.getElementById('boiler-diff').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="boiler-diff" name="differenz" type="number" min="0" max="100" step="0.1" value="%BOILERDIFF%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Einschaltdifferenz erh&ouml;hen" onclick="document.getElementById('boiler-diff').stepUp()">+</button>
      </div>
      <p id="boiler-save-status" class="threshold-hint" role="status">&Auml;nderungen erst mit Speichern &uuml;bernehmen.</p>
      <button id="boiler-save" class="threshold-save" type="submit">Boilereinstellungen speichern</button>
    </form>
    <p id="boiler-settings" style="font-size:.8rem;color:#607681;">%BOILERINFO%</p>
  </div>
  <p>
  </section>
  <section class="layout">
  <div>
    <h3>Gasverbrauch heute</h3>
    <strong id="gas-day">%GASDAY%</strong> m&sup3;
    <p>Seit erster Meldung des Tages</p>
    <small>Gesamtstand: <span id="gas-total">%GASTOTAL%</span> m&sup3;</small>
  </div>
  <div>
    <h3>Regelung</h3>
    <strong id="regelungsart">%REGELUNGSART%</strong>
    <form action="/regelung" method="post">
      <p><label for="regelung-auswahl">Regelungsart ausw&auml;hlen</label></p>
      <p><select id="regelung-auswahl" name="modus" style="width:100%;padding:12px;font:inherit;border:1px solid #cbd9dd;border-radius:8px">
        <option value="auto" %MODEAUTO%>Automatik</option>
        <option value="raum" %MODERAUM%>Raumtemperatur</option>
        <option value="aussen" %MODEAUSSEN%>Au&szlig;entemperatur</option>
      </select></p>
      <button class="threshold-save" type="submit">&Uuml;bernehmen</button>
    </form>
  </div>
  <div>
    <h3>Betriebsart</h3>
    <strong id="betriebsart">%BETRIEBSART%</strong>
    <form action="/betriebsart" method="post">
      <p><label for="betrieb-auswahl">Betriebsart ausw&auml;hlen</label></p>
      <p><select id="betrieb-auswahl" name="modus" style="width:100%;padding:12px;font:inherit;border:1px solid #cbd9dd;border-radius:8px">
        <option value="aus" %BETRIEBAUS%>Aus</option>
        <option value="auto" %BETRIEBAUTO%>Automatikbetrieb</option>
        <option value="heizung" %BETRIEBHEIZUNG%>Nur Heizung</option>
        <option value="boiler" %BETRIEBBOILER%>Nur Boiler</option>
        <option value="urlaub" %BETRIEBURLAUB%>Urlaub</option>
      </select></p>
      <button class="threshold-save" type="submit">&Uuml;bernehmen</button>
    </form>
  </div>
  <div><h3>NTP-Zeitserver</h3>
    <p>Abweichung zur ESP32-Uhr: Plus = Server voraus</p>
    <pre id="ntp-abweichung" style="white-space:pre-wrap;font:inherit;overflow-wrap:anywhere">Messung wird geladen</pre>
    <small>Messung etwa jede Minute; Netzwerkwege beeinflussen die Genauigkeit.</small>
  </div>
  </section>
  <section class="layout">
  <div class="threshold-card">
    <h3>Wunschraumtemperatur</h3>
    <form id="room-setpoints" class="threshold-form" action="/raumtemperaturen" method="post">
      <label for="room-tag">Tag</label>
      <div class="threshold-row">
        <button type="button" aria-label="Tagtemperatur senken" onclick="document.getElementById('room-tag').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="room-tag" name="tag" type="number" min="5" max="40" step="0.1" value="%RAUMTAG%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Tagtemperatur erh&ouml;hen" onclick="document.getElementById('room-tag').stepUp()">+</button>
      </div>
      <label for="room-nacht">Nacht</label>
      <div class="threshold-row">
        <button type="button" aria-label="Nachttemperatur senken" onclick="document.getElementById('room-nacht').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="room-nacht" name="nacht" type="number" min="5" max="40" step="0.1" value="%RAUMNACHT%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Nachttemperatur erh&ouml;hen" onclick="document.getElementById('room-nacht').stepUp()">+</button>
      </div>
      <label for="room-urlaub">Urlaub</label>
      <div class="threshold-row">
        <button type="button" aria-label="Urlaubstemperatur senken" onclick="document.getElementById('room-urlaub').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="room-urlaub" name="urlaub" type="number" min="5" max="40" step="0.1" value="%RAUMURLAUB%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Urlaubstemperatur erh&ouml;hen" onclick="document.getElementById('room-urlaub').stepUp()">+</button>
      </div>
      <small>Im Urlaubsbetrieb gilt diese Raumtemperatur durchgehend; die Boilerbereitung ist aus. Fehlernachlauf kann eine laufende Boilerpumpe vor&uuml;bergehend weiter betreiben.</small>
      <label for="day-start">Tagbetrieb ab</label>
      <input id="day-start" name="tagzeit" type="time" step="60" value="%TAGZEIT%" required style="max-width:100%;font:inherit;padding:8px;border:1px solid #bdcdd3;border-radius:7px;">
      <label for="night-start">Nachtbetrieb ab</label>
      <input id="night-start" name="nachtzeit" type="time" step="60" value="%NACHTZEIT%" required style="max-width:100%;font:inherit;padding:8px;border:1px solid #bdcdd3;border-radius:7px;">
      <small>Tagbeginn muss vor Nachtbeginn liegen.</small>
      <p id="room-save-status" class="threshold-hint" role="status">&Auml;nderungen werden erst mit Speichern &uuml;bernommen.</p>
      <button id="room-save" class="threshold-save" type="submit">Temperaturen und Zeiten speichern</button>
    </form>
  </div>
  <div>
    <span class="dht-labels">Kachelofen</span>
    <span id="tkachelofen">%TKACHELOFEN%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>Kachelofen: <strong id="kachelofen-status">%KACHELOFENSTATUS%</strong></div>
  <div class="threshold-card">
    <form class="threshold-form" action="/kachelofen" method="get">
  <h3>Kachelofen-Schaltschwellen</h3>
  <label for="threshold-ein">Aktiv ab</label>
  <div class="threshold-row">
    <button type="button" aria-label="Einschalttemperatur senken" onclick="adjustThreshold('threshold-ein', -1)">&minus;</button>
    <div class="threshold-value"><input id="threshold-ein" name="ein" type="number" min="10" max="150" step="0.5" value="%KACHELOFENEIN%" readonly required><span>&deg;C</span></div>
    <button type="button" aria-label="Einschalttemperatur erhöhen" onclick="adjustThreshold('threshold-ein', 1)">+</button>
  </div>
  <label for="threshold-aus">Inaktiv bei oder unter</label>
  <div class="threshold-row">
    <button type="button" aria-label="Ausschalttemperatur senken" onclick="adjustThreshold('threshold-aus', -1)">&minus;</button>
    <div class="threshold-value"><input id="threshold-aus" name="aus" type="number" min="0" max="140" step="0.5" value="%KACHELOFENAUS%" readonly required><span>&deg;C</span></div>
    <button type="button" aria-label="Ausschalttemperatur erhöhen" onclick="adjustThreshold('threshold-aus', 1)">+</button>
  </div>
  <p class="threshold-hint" id="threshold-hint" aria-live="polite">Änderungen werden erst mit Speichern übernommen.</p>
  <button class="threshold-save" id="threshold-save" type="submit">Schaltschwellen speichern</button>
</form>
<script>
function adjustThreshold(id, direction) {
  const field = document.getElementById(id);
  if (direction > 0) field.stepUp(); else field.stepDown();
  const ein = document.getElementById('threshold-ein').valueAsNumber;
  const aus = document.getElementById('threshold-aus').valueAsNumber;
  const valid = Number.isFinite(ein) && Number.isFinite(aus) && aus < ein;
  document.getElementById('threshold-save').disabled = !valid;
  document.getElementById('threshold-hint').textContent = valid
    ? 'Änderungen werden erst mit Speichern übernommen.'
    : 'Die Ausschalttemperatur muss niedriger als die Einschalttemperatur sein.';
}
</script>
  </div>
  </section>
  <p>
  <section class="layout">
  </section>
  <p>
  <section class="layout">
  <div><h3>WLAN</h3>
    <p>Netzwerk: <span id="wifi-ssid">%WIFISSID%</span><br>
    Signalst&auml;rke: <span id="wifi-rssi">%WIFIRSSI%</span> dBm</p>
  </div>
  <div><h3>Systemdiagnose</h3>
    <p>Speicher frei: <span id="heap-free">--</span><br>
    Minimum seit Start: <span id="heap-min">--</span><br>
    Gr&ouml;&szlig;ter freier Block: <span id="heap-largest">--</span></p>
    <p>Schleifenabstand, zuletzt max.: <span id="loop-recent">--</span><br>
    Maximum seit Start: <span id="loop-max">--</span><br>
    Abst&auml;nde &uuml;ber 100 ms: <span id="loop-slow">--</span></p>
    <p>OTA: <span id="ota-status">Noch keine Diagnose</span></p>
    <small>Abstand zwischen zwei Schleifenstarts inklusive Wartezeiten; keine reine CPU-Rechenzeit. Historische Maxima bleiben bis zum Neustart erhalten.</small>
  </div>
  </section>
  <p>
<script>
function roomSaveRequest(url, method, body) {
  return new Promise(function(resolve, reject) {
    var request = new XMLHttpRequest();
    request.open(method, url, true); request.timeout = 2500;
    if (body) request.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
    request.onload = function() {
      if (request.status < 200 || request.status >= 300) { reject(new Error(request.responseText || 'Anfrage abgelehnt')); return; }
      try { resolve(JSON.parse(request.responseText)); } catch (error) { reject(error); }
    };
    request.onerror = request.ontimeout = function() { reject(new Error('Keine Antwort')); };
    request.send(body || null);
  });
}
document.getElementById('room-setpoints').addEventListener('submit', async function(event) {
  event.preventDefault();
  var button = document.getElementById('room-save');
  var status = document.getElementById('room-save-status');
  button.disabled = true; status.textContent = 'Wird gespeichert...';
  try {
    var body = new URLSearchParams(new FormData(event.target)).toString();
    var accepted = await roomSaveRequest('/raumtemperaturen', 'POST', body);
    var confirmed = false;
    for (var attempt = 0; attempt < 20; ++attempt) {
      await new Promise(function(resolve) { setTimeout(resolve, 500); });
      var result = await roomSaveRequest('/raumtemperaturen-status?id='+accepted.id, 'GET');
      if (result.state === 2) { status.textContent = 'Temperaturen und Zeiten gespeichert.'; confirmed = true; break; }
      if (result.state === 3) { status.textContent = 'Speichern fehlgeschlagen. Bisherige Werte bleiben aktiv.'; confirmed = true; break; }
    }
    if (!confirmed) status.textContent = 'Speichern noch nicht bestätigt. Bitte neu laden und Werte prüfen.';
  } catch (error) {
    status.textContent = 'Speichern nicht bestätigt: ' + error.message;
  } finally { button.disabled = false; }
});

document.getElementById('boiler-setpoint').addEventListener('submit', async function(event) {
  event.preventDefault();
  var button = document.getElementById('boiler-save');
  var status = document.getElementById('boiler-save-status');
  button.disabled = true; status.textContent = 'Wird gespeichert...';
  try {
    var body = new URLSearchParams(new FormData(event.target)).toString();
    var accepted = await roomSaveRequest('/boilertemperatur', 'POST', body);
    var confirmed = false;
    for (var attempt = 0; attempt < 20; ++attempt) {
      await new Promise(function(resolve) { setTimeout(resolve, 500); });
      var result = await roomSaveRequest('/boilertemperatur-status?id='+accepted.id, 'GET');
      if (result.state === 2) { status.textContent = 'Boilereinstellungen gespeichert.'; confirmed = true; break; }
      if (result.state === 3) { status.textContent = 'Speichern fehlgeschlagen. Bisheriger Sollwert bleibt aktiv.'; confirmed = true; break; }
    }
    if (!confirmed) status.textContent = 'Speichern noch nicht bestätigt. Bitte neu laden und Wert prüfen.';
  } catch (error) {
    status.textContent = 'Speichern nicht bestätigt: ' + error.message;
  } finally { button.disabled = false; }
});

document.getElementById('flow-limit').addEventListener('submit', async function(event) {
  event.preventDefault();
  var button = document.getElementById('flow-save');
  var status = document.getElementById('flow-save-status');
  button.disabled = true; status.textContent = 'Wird gespeichert...';
  try {
    var body = new URLSearchParams(new FormData(event.target)).toString();
    var accepted = await roomSaveRequest('/vorlaufmaximum', 'POST', body);
    var confirmed = false;
    for (var attempt = 0; attempt < 20; ++attempt) {
      await new Promise(function(resolve) { setTimeout(resolve, 500); });
      var result = await roomSaveRequest('/vorlaufmaximum-status?id='+accepted.id, 'GET');
      if (result.state === 2) { status.textContent = 'Heizkurveneinstellungen gespeichert.'; confirmed = true; break; }
      if (result.state === 3) { status.textContent = 'Speichern fehlgeschlagen. Bisheriger Sollwert bleibt aktiv.'; confirmed = true; break; }
    }
    if (!confirmed) status.textContent = 'Speichern noch nicht bestätigt. Bitte neu laden und Wert prüfen.';
  } catch (error) {
    status.textContent = 'Speichern nicht bestätigt: ' + error.message;
  } finally { button.disabled = false; }
});

var temperatureKeys = ['troom','taussen','tkessel','tvorlauf','tboiler','tkachelofen'];
temperatureKeys.forEach(function(key) {
  var arrow = document.createElement('span');
  arrow.id = key+'-trend'; arrow.className = 'temperature-trend';
  document.getElementById(key).insertAdjacentElement('afterend', arrow);
});
function showTemperatureTrend(key, direction) {
  var arrow = document.getElementById(key+'-trend');
  var label = direction === 1 ? 'Temperatur steigt' : direction === -1 ? 'Temperatur sinkt' : direction === 0 ? 'Temperatur gleichbleibend oder erste Messung' : 'Kein gueltiger Trend';
  arrow.textContent = direction === 1 ? '\u2191' : direction === -1 ? '\u2193' : direction === 0 ? '\u2194' : '';
  arrow.className = 'temperature-trend' + (direction === 1 ? ' up' : direction === -1 ? ' down' : '');
  arrow.title = label; arrow.setAttribute('aria-label', label);
}
var statusTextKeys = ['troom','taussen','tkessel','tvorlauf','tboiler','tkachelofen','header-clock','brennersperre','regelungsart','betriebsart','gas-day','gas-total','ntp-abweichung','kachelofen-status','uptime','cpu-load','wifi-ssid','wifi-rssi','ota-status','boiler-settings','boiler-delta'];
var sharedStatusPending = false;
var diagnosticKeys = ['heap-free','heap-min','heap-largest','loop-recent','loop-max','loop-slow'];
function showDiagnostics(health) {
  var fields = ['heapFreeBytes','heapMinBytes','heapLargestBlockBytes','loopRecentMaxMs','loopMaxMs','loopOver100ms'];
  var valid = health && fields.every(function(key) { return typeof health[key] === 'number' && Number.isFinite(health[key]) && health[key] >= 0; });
  diagnosticKeys.forEach(function(key, index) {
    var text = '--';
    if (valid) {
      var value = health[fields[index]];
      text = index < 3 ? (value/1024).toFixed(1)+' KiB' : index < 5 ? value.toFixed(1)+' ms' : String(value);
    }
    document.getElementById(key).textContent = text;
  });
}
var lastStatusGeneration = null;
var lastStatusChange = Date.now();
function unavailableSharedStatus() {
  showBurnerFlame(null);
  showHeatingPump(null);
  showBoilerPump(null);
  showMixerStatus(null, null);
  showDiagnostics(null);
  temperatureKeys.forEach(function(key) { showTemperatureTrend(key, null); });
  statusTextKeys.forEach(function(key) {
    document.getElementById(key).textContent = key.charAt(0) === 't' ? '--' : 'Nicht erreichbar';
  });
  updateBoilerDeltaColor();
}
function showMixerStatus(open, close) {
  var icon = document.getElementById('mixer-symbol');
  var known = typeof open === 'boolean' && typeof close === 'boolean';
  var valid = known && !(open && close);
  var label = !known ? 'Nicht erreichbar' : !valid ? 'Ausgangskonflikt' : open ? '\u2191 Auf' : close ? '\u2193 Zu' : 'Stopp';
  icon.setAttribute('class', valid && (open || close) ? 'moving' : '');
  icon.setAttribute('aria-label', 'Mischer: ' + label);
  icon.querySelector('title').textContent = 'Mischer: ' + label;
  document.getElementById('mixer-direction').textContent = label;
}
function showHeatingPump(active) {
  var icon = document.getElementById('heating-pump-icon');
  var label = active === true ? 'Heizungspumpenausgang ein' : active === false ? 'Heizungspumpenausgang aus' : 'Heizungspumpenstatus nicht erreichbar';
  icon.setAttribute('class', active === true ? 'on' : '');
  icon.setAttribute('aria-label', label);
  icon.querySelector('title').textContent = label;
}
function showBoilerPump(active) {
  var icon = document.getElementById('boiler-pump-icon');
  var label = active === true ? 'Boilerpumpenausgang ein' : active === false ? 'Boilerpumpenausgang aus' : 'Boilerpumpenstatus nicht erreichbar';
  icon.setAttribute('class', active === true ? 'on' : '');
  icon.setAttribute('aria-label', label);
  icon.querySelector('title').textContent = label;
}
function showBurnerFlame(active) {
  var flame = document.getElementById('burner-flame');
  var label = active === true ? 'Brennerausgang ein' : active === false ? 'Brennerausgang aus' : 'Brennerstatus nicht erreichbar';
  flame.setAttribute('class', active === true ? 'on' : '');
  flame.setAttribute('aria-label', label);
  flame.querySelector('title').textContent = label;
}
function updateBoilerDeltaColor() {
  var element = document.getElementById('boiler-delta');
  var difference = parseFloat(element.textContent);
  element.className = Number.isFinite(difference) ? (difference > 0 ? 'warmer' : 'cooler') : '';
}
updateBoilerDeltaColor();
function updateSharedStatus() {
  if (sharedStatusPending) return;
  sharedStatusPending = true;
  var request = new XMLHttpRequest();
  request.open('GET', '/webstatus', true); request.timeout = 2500;
  request.onload = function() {
    if (request.status !== 200) { unavailableSharedStatus(); return; }
    try {
      var data = JSON.parse(request.responseText);
      if (!data.values || !Array.isArray(data.outputs) || data.outputs.length !== 5 || typeof data.generation !== 'number') throw new Error();
      statusTextKeys.forEach(function(key) { if (typeof data.values[key] !== 'string') throw new Error(); });
      data.outputs.forEach(function(value) { if (value !== 0 && value !== 1) throw new Error(); });
      if (!Array.isArray(data.trends) || data.trends.length !== temperatureKeys.length) throw new Error();
      data.trends.forEach(function(value) { if (value !== null && value !== -1 && value !== 0 && value !== 1) throw new Error(); });
      if (lastStatusGeneration !== data.generation) { lastStatusGeneration = data.generation; lastStatusChange = Date.now(); }
      if (Date.now()-lastStatusChange > 6000) { unavailableSharedStatus(); return; }
      statusTextKeys.forEach(function(key) { document.getElementById(key).textContent = data.values[key]; });
      showBurnerFlame(data.outputs[0] === 1);
      showHeatingPump(data.outputs[2] === 1);
      showBoilerPump(data.outputs[1] === 1);
      showMixerStatus(data.outputs[3] === 1, data.outputs[4] === 1);
      updateBoilerDeltaColor();
      showDiagnostics(data.diagnostics);
      temperatureKeys.forEach(function(key, index) { showTemperatureTrend(key, data.trends[index]); });
    } catch (error) { unavailableSharedStatus(); }
  };
  request.onerror = request.ontimeout = unavailableSharedStatus;
  request.onloadend = function() { sharedStatusPending = false; };
  request.send();
}
setInterval(updateSharedStatus, 1000); updateSharedStatus();
</script>
</body>
</html>)rawliteral";
