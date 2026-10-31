// Actual login UI functions with a small DOM stub, without a browser dependency.
const fs=require('fs'), vm=require('vm'), assert=require('assert'), path=require('path');
const page=fs.readFileSync(path.join(__dirname,'../../src/web/page.cpp'),'utf8');
const elements=new Map();
const get=id=>{if(!elements.has(id))elements.set(id,{hidden:false,textContent:'',addEventListener(){}});return elements.get(id)};
const fields=Array.from({length:7},()=>({disabled:true}));
const storage=new Map();
const context={document:{getElementById:get,querySelectorAll(){return fields}},sessionStorage:{getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)},setInterval(){},roomSaveRequest:async()=>({angemeldet:false})};
vm.createContext(context);
vm.runInContext(page.slice(page.indexOf('var webToken'),page.indexOf('function roomSaveRequest')),context);
context.webToken='existing';storage.set('heizung-session','existing');context.setWebAccess(true);
assert(fields.every(f=>!f.disabled));assert(get('web-login').hidden);assert(!get('web-logout').hidden);
context.setWebAccess(false);assert(fields.every(f=>f.disabled));assert(!get('web-login').hidden);assert(get('web-logout').hidden);assert(!storage.has('heizung-session'));
// Authentication forms have no prefilled secret; all five editable forms fail closed in HTML.
for(const id of ['flow-limit','boiler-setpoint','room-setpoints','stove-settings','wifi-settings','mqtt-settings','password-settings']) {
 const form=page.match(new RegExp('<form id="'+id+'"[\\s\\S]*?</form>'))[0];assert(form.includes('<fieldset class="protected-settings" disabled>'));
}
for(const action of ['regelung','betriebsart']) {
 const form=page.match(new RegExp('<form[^>]*action="/'+action+'"[\\s\\S]*?</form>'))[0];assert(!form.includes('protected-settings'));
}
assert(!/name="passwort"[^>]*value=/.test(page));
// Network interruptions must not destroy a still-valid session token.
(async()=>{context.webToken='retained';context.roomSaveRequest=async()=>{throw Error('offline')};await context.checkWebAccess();assert.equal(context.webToken,'retained');assert(fields.every(f=>f.disabled));console.log('Web-Bedienung: gesperrter Start, Freigabe, Abmeldung, freie Umschalter und Verbindungsunterbrechung geprueft.');})().catch(e=>{console.error(e);process.exitCode=1});
