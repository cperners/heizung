const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../..');
const runtime = process.env.CODEX_NODE_MODULES || '/Users/christianpernerstorfer/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules';
const {chromium} = require(path.join(runtime,'playwright'));
const html = fs.readFileSync(path.join(root,'main/web/index.html'),'utf8');
const config = {mode:'auto',holiday_room_target:15,curve_zero:45,curve_slope:1,room_gain:3,room_target:21,unsynced_room:17.5,flow_min:25,flow_max:60,summer_temp:20,boiler_target:50,boiler_hysteresis:5,kessel_max:85,kessel_margin:10,burner_hysteresis:5,dhw_lift:10,frost_outside:3,frost_water:5,frost_flow:30,mixer_band:1,burner_min_on_s:120,burner_min_off_s:180,pump_overrun_s:120,mixer_travel_s:120,mixer_dead_ms:1000,sensor_stale_s:5,dhw_priority:true,buffer_enabled:false,pv_enabled:false,periods:[{start_minute:360,room_target:21},{start_minute:510,room_target:18},{start_minute:990,room_target:21},{start_minute:1350,room_target:17.5}]};
const services={base_topic:'heizung/anlage',uri:'mqtt://broker.local:1883',username:'heizung',password_set:true,ntp_primary:'192.168.0.1',ntp_secondary:'0.at.pool.ntp.org',ntp_tertiary:'1.at.pool.ntp.org'};
const restConfig={enabled:true,stop_c:30,max_s:1800};
const gasConfig={enabled:false,topic:'',stale_s:3600};
const stoveConfig={enabled:true,topic:"haus/kachelofen/temperatur",on_c:45,off_c:32,stale_s:300};
const status={stove:{enabled:true,valid:false,active:false,temperature:null,age_s:null},room_correction_suppressed:false,csrf:'0123456789abcdef'.repeat(4),login_configured:true,faults:115,output_timeout:false,actuators_enabled:false,frost:false,dhw_demand:false,mixer_position:-1,uptime_s:112,heap_free:320000,heap_min:302000,idf:'v5.5.5',version:'1',ethernet:true,mqtt:true,time_synced:true,clock:'27.10.2026 19:30:00',ip:'192.168.0.50',mode:'auto',room_target:21,flow_target:45,kessel_target:55,boiler_target:50,sensors:Array.from({length:8},(_,i)=>({valid:false,temperature:null,failures:i<5?112:0})),outputs:{burner:false,heating_pump:false,boiler_pump:false,mixer:'STOP'}};
(async()=>{
 const browser=await chromium.launch({channel:'chrome',headless:true});
 try{
 const page=await browser.newPage({viewport:{width:1440,height:1080}});
 const errors=[];const posts=[];let offline=false;
 page.on('pageerror',e=>errors.push(e.message));
 await page.route('http://preview.local/**',async route=>{
  const req=route.request(),url=new URL(req.url());
  if(url.pathname==='/')return route.fulfill({status:200,contentType:'text/html',body:html});
  if(url.pathname==='/api/status')return offline?route.abort():route.fulfill({json:status});
  if(url.pathname==='/api/stove/history')return route.fulfill({json:{points:[],on_c:45,off_c:32,stale_s:900}});
  if(url.pathname==='/api/config'&&req.method()==='GET')return route.fulfill({json:config});
  if(url.pathname==='/api/mode'&&req.method()==='POST'){const body=req.postDataJSON();assert.deepEqual(Object.keys(body),['mode']);assert.equal(req.headers()['x-csrf-token'],status.csrf);Object.assign(config,body);return route.fulfill({json:{message:'Einstellungen gespeichert'}});}
  const expected='Basic '+Buffer.from('demo:demo_password_123').toString('base64');
  if(req.headers().authorization!==expected)return route.fulfill({status:401,json:{message:'Bitte anmelden'}});
  if(req.method()==='GET')return route.fulfill({json:url.pathname==='/api/config'?config:url.pathname==='/api/stove'?stoveConfig:url.pathname==='/api/gas'?gasConfig:url.pathname==='/api/rest-heat'?restConfig:services});
  assert.equal(req.headers()['x-csrf-token'],status.csrf);
  const body=req.postDataJSON();posts.push({url:url.pathname,body});
  if(url.pathname==='/api/config')Object.assign(config,body);
  return route.fulfill({json:{message:'Einstellungen gespeichert'}});
 });
 await page.goto('http://preview.local/');
 await page.waitForFunction(()=>document.getElementById('mqttPill').textContent==='MQTT verbunden');
 assert(await page.locator('#mode').isEnabled());
 assert.equal(await page.locator('#mode').inputValue(),status.mode);
 assert.equal(await page.locator('#curve_slope').inputValue(),'1');
 assert.equal(await page.locator('#boiler_target').inputValue(),'50');
 assert.equal(await page.locator('#temp0').textContent(),'–°C');
 assert.equal(await page.locator('#faults li').count(),5);
 await page.locator('#loginButton').click();await page.locator('#loginUser').fill('demo');await page.locator('#loginPass').fill('falsch');
 await page.locator('#submitLogin').click();await page.waitForFunction(()=>document.getElementById('loginError').textContent.length>0);
 assert(await page.locator('#mode').isEnabled());
 await page.locator('#loginPass').fill('demo_password_123');await page.locator('#submitLogin').click();
 await page.waitForFunction(()=>!document.getElementById('loginDialog').open);
 assert(await page.locator('#mode').isEnabled());assert.equal(await page.locator('#mqttPassword').inputValue(),'');
 assert.equal(await page.locator('#ntp2').inputValue(),'1.at.pool.ntp.org');
 await page.locator('#room_target').fill('22');await page.locator('#modeForm button').click();
 await page.waitForFunction(()=>document.getElementById('toast').textContent==='Einstellungen gespeichert');
 assert.equal(posts[0].body.room_target,22);
 await page.locator('#mqttUser').fill('new-user');page.once('dialog',d=>d.accept());
 await page.locator('[data-restart] button').first().click();
 await page.waitForFunction(()=>document.getElementById('loginButton').textContent==='Anmelden');
 assert.equal(posts[1].url,'/api/services');assert(!('password' in posts[1].body));
 // Frische Seite fuer Screenshots, damit der simulierte Neustart nicht dazwischenfaellt.
 const preview=await browser.newPage({viewport:{width:1440,height:1080}});await preview.route('http://preview.local/**',async route=>{
 const url=new URL(route.request().url());return route.fulfill(url.pathname==='/'?{contentType:'text/html',body:html}:{json:url.pathname==='/api/config'?config:url.pathname==='/api/stove/history'?{points:[],on_c:45,off_c:32,stale_s:900}:status});});
 await preview.goto('http://preview.local/');await preview.waitForFunction(()=>document.getElementById('mqttPill').textContent==='MQTT verbunden');
 await preview.screenshot({path:path.join(root,'output/web/desktop.png'),fullPage:true});
 await preview.setViewportSize({width:390,height:844});await preview.screenshot({path:path.join(root,'output/web/mobile.png'),fullPage:true});
 assert(await preview.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
 offline=true;await page.reload();await page.waitForFunction(()=>!document.getElementById('offline').classList.contains('hidden'));
 assert(await page.locator('#mode').isDisabled());assert.deepEqual(errors,[]);
 console.log('PASS: Anzeige, fehlende Sensoren, falscher/richtiger Login, Speichern, CSRF-Header, Passwort bleibt unveraendert, Offline-Sperre, Mobilansicht.');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
