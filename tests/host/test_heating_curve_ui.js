// Test the actual chart code with a DOM stub, including narrow-card geometry.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const page=fs.readFileSync(path.join(__dirname,'../../src/web/page.cpp'),'utf8');
let width=160;
class Element {
 constructor(){this.children=[];this.attrs={};this.textContent='';this.open=true;}
 get firstChild(){return this.children[0]||null;}
 appendChild(e){this.children.push(e);}
 removeChild(e){this.children.splice(this.children.indexOf(e),1);}
 setAttribute(k,v){this.attrs[k]=String(v);}
 getBoundingClientRect(){return {width};}
 addEventListener(){}
}
const elements=new Map();const get=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
const context={document:{getElementById:get,createElementNS:()=>new Element()},window:{addEventListener(){}}};
vm.createContext(context);
vm.runInContext(page.slice(page.indexOf('var heatingCurveModel'),page.indexOf('var temperatureKeys')),context);
const m={room:22.5,maximum:40,minimumOutside:-15,exponent:1.9,outside:12.5,measured:45.38,active:true,target:31.23};
assert.strictEqual(context.heatingCurveTarget(m,22.5),22.5);
assert.strictEqual(context.heatingCurveTarget(m,-15),40);
assert.strictEqual(context.heatingCurveTarget(m,-55),40);
assert.strictEqual(context.heatingCurveTarget(m,35),22.5);
assert.strictEqual(context.heatingCurveTarget({...m,exponent:1},0),33);
assert.strictEqual(context.heatingCurveTarget(m,0),35.87);
for(const model of [{...m,maximum:10},{...m,exponent:0},{...m,minimumOutside:5},{...m,room:null}])assert.strictEqual(context.heatingCurveTarget(model,0),null);
for(const w of [160,240,400]){
 width=w;context.showHeatingCurve(m);
 assert.equal(get('heating-curve-chart').attrs.viewBox,`0 0 ${w} 260`);
 assert(get('heating-curve-state').textContent.includes('Soll 31,2'));
 const svg=get('heating-curve-chart');assert(!JSON.stringify(svg).match(/NaN|Infinity/));
 for(const item of svg.children){
  if(item.attrs.d){const coords=item.attrs.d.match(/-?\d+(?:\.\d+)?/g).map(Number);assert(coords.every(Number.isFinite));}
  if(item.attrs.cx)assert(+item.attrs.cx>=34&&+item.attrs.cx<=w-10);
 }
 // Same model and width must not allocate/redraw the curve again every second.
 const first=svg.firstChild;context.showHeatingCurve({...m});assert.strictEqual(svg.firstChild,first);
}
context.showHeatingCurve({...m,active:false,target:null});assert(get('heating-curve-state').textContent.includes('nicht aktiv'));
context.showHeatingCurve({...m,outside:null,measured:null});assert(get('heating-curve-state').textContent.includes('Kein gueltiger Aussenwert'));
context.showHeatingCurve(null);assert.equal(get('heating-curve-info').textContent,'Keine aktuellen Daten.');
assert(get('heating-curve-chart').children.length===1);
get('heating-curve-panel').open=false;context.showHeatingCurve(m);assert.equal(get('heating-curve-info').textContent,'Keine aktuellen Daten.');
get('heating-curve-panel').open=true;context.drawHeatingCurve();assert(get('heating-curve-info').textContent.startsWith('Gespeichert:'));
// Status refresh and the offline path must both update the chart.
assert(page.includes('showHeatingCurve(data.heatingCurve)'));assert(page.includes('function unavailableSharedStatus() {\n  showHeatingCurve(null);'));
console.log('Heizkurvengrafik: Grenzen, Potenzfunktion, schmale Kacheln, Live-Werte, Offline- und Sensorfehleranzeige geprueft.');

assert(page.includes('class="heating-curve-card"'));
assert(!page.slice(page.indexOf('<span class="dht-labels">Vorlauf'),page.indexOf('<span class="dht-labels">Boiler')).includes('id="heating-curve-panel"'));

assert.deepStrictEqual(Array.from(context.heatingCurveTicks(-20,30,1000)),[-20,-15,-10,-5,0,5,10,15,20,25,30]);
const mobileTicks=Array.from(context.heatingCurveTicks(-20,30,210));
assert.deepStrictEqual(mobileTicks,[-20,-10,0,10,20,30]);
assert(mobileTicks.every((v,i)=>i===0||(v-mobileTicks[i-1])*210/50>=38));
console.log('Temperaturmarken: 5-Grad-Raster auf breiten Anzeigen, lesbare Abstaende auf schmalen Anzeigen geprueft.');
