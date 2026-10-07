#include "page.h"

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
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
</style>
<title>ESP32 Heizungs-Server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" 
        integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
<link rel="icon" href="data:,">
</head>
<body>
  <div class="topnav">
    <h1><span>ESP32 Heizung</span><span style="font-size:.65em;color:#fef08a;">Laufzeit: <span id="uptime">--</span></span><span style="font-size:.65em;color:#fef08a;" title="Anteil der Ticks ohne Leerlauf, keine exakte CPU-Zeitmessung">CPU-Sch&auml;tzung: <span id="cpu-load">--</span></span><time id="header-clock">%WEBTIME%</time></h1>
  </div>
  <section class="layout">
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">K&uuml;che</span> 
    <span id="troom">%TROOM%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Aussen</span> 
    <span id="taussen">%TAUSSEN%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Gaskessel</span> 
    <span id="tkessel">%TKESSEL%</span>
    <div id="brennersperre" role="status">%BRENNERSPERRE%</div>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Vorlauf</span> 
    <span id="tvorlauf">%TVORLAUF%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Boiler</span> 
    <span id="tboiler">%TBOILER%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <p>
  </section>
  <section class="layout">
  <div><h3>Ausgangsstatus</h3>
    <div class="output-status">
      <div class="output-row"><span>Brenner</span><span class="output-indicator"><span id="led-brenner" class="output-led" aria-hidden="true"></span><span id="status-brenner">Wird geladen</span></span></div>
      <div class="output-row"><span>Boilerpumpe</span><span class="output-indicator"><span id="led-boiler" class="output-led" aria-hidden="true"></span><span id="status-boiler">Wird geladen</span></span></div>
      <div class="output-row"><span>Heizungspumpe</span><span class="output-indicator"><span id="led-heizung" class="output-led" aria-hidden="true"></span><span id="status-heizung">Wird geladen</span></span></div>
      <div class="output-row"><span>Mischer auf</span><span class="output-indicator"><span id="led-mischerauf" class="output-led" aria-hidden="true"></span><span id="status-mischerauf">Wird geladen</span></span></div>
      <div class="output-row"><span>Mischer zu</span><span class="output-indicator"><span id="led-mischerzu" class="output-led" aria-hidden="true"></span><span id="status-mischerzu">Wird geladen</span></span></div>
    </div>
    <small>Gr&uuml;n: Ausgang ein &middot; Rot: Ausgang aus</small>
  </div>
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
        <option value="auto" %BETRIEBAUTO%>Automatikbetrieb</option>
        <option value="boiler" %BETRIEBBOILER%>Nur Boiler</option>
        <option value="heizung" %BETRIEBHEIZUNG%>Nur Heizung</option>
        <option value="aus" %BETRIEBAUS%>Aus</option>
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
      <p id="room-save-status" class="threshold-hint" role="status">&Auml;nderungen werden erst mit Speichern &uuml;bernommen.</p>
      <button id="room-save" class="threshold-save" type="submit">Wunschtemperaturen speichern</button>
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
  <div>WLAN-SSID: <span id="wifi_rssi">%WIFISSID%</span></div>
  <div>WLAN-Signal: <span id="wifi_rssi">%WIFIRSSI%</span></div>
  <div><input type="range" onchange="updateSliderTimer(this)" id="wifi_rssis" min="-100" max="-10" value="%WIFIRSSI%" step="1" class="slider2"></div>
  </section>
  <p>
<script>
function roomSaveRequest(url, method, body) {
  return new Promise(function(resolve, reject) {
    var request = new XMLHttpRequest();
    request.open(method, url, true); request.timeout = 2500;
    if (body) request.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
    request.onload = function() {
      if (request.status < 200 || request.status >= 300) { reject(new Error('Anfrage abgelehnt')); return; }
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
      if (result.state === 2) { status.textContent = 'Wunschtemperaturen gespeichert.'; confirmed = true; break; }
      if (result.state === 3) { status.textContent = 'Speichern fehlgeschlagen. Bisherige Werte bleiben aktiv.'; confirmed = true; break; }
    }
    if (!confirmed) status.textContent = 'Speichern noch nicht bestätigt. Bitte neu laden und Werte prüfen.';
  } catch (error) {
    status.textContent = 'Speichern nicht bestätigt. Bitte neu laden und Werte prüfen.';
  } finally { button.disabled = false; }
});

var outputKeys = ['brenner','boiler','heizung','mischerauf','mischerzu'];
var statusTextKeys = ['troom','taussen','tkessel','tvorlauf','tboiler','tkachelofen','header-clock','brennersperre','regelungsart','betriebsart','gas-day','gas-total','ntp-abweichung','kachelofen-status','uptime','cpu-load'];
var sharedStatusPending = false;
var lastStatusGeneration = null;
var lastStatusChange = Date.now();
function unavailableSharedStatus() {
  outputKeys.forEach(function(key) {
    document.getElementById('led-'+key).className = 'output-led';
    document.getElementById('status-'+key).textContent = 'Nicht erreichbar';
  });
  statusTextKeys.forEach(function(key) {
    document.getElementById(key).textContent = key.charAt(0) === 't' ? '--' : 'Nicht erreichbar';
  });
}
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
      if (lastStatusGeneration !== data.generation) { lastStatusGeneration = data.generation; lastStatusChange = Date.now(); }
      if (Date.now()-lastStatusChange > 6000) { unavailableSharedStatus(); return; }
      statusTextKeys.forEach(function(key) { document.getElementById(key).textContent = data.values[key]; });
      outputKeys.forEach(function(key, index) {
        var active = data.outputs[index] === 1;
        document.getElementById('led-'+key).className = 'output-led ' + (active ? 'on' : 'off');
        document.getElementById('status-'+key).textContent = active ? 'Ein' : 'Aus';
      });
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
