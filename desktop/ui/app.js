import {NativeSession,statusFlags,elapsed,pressureState} from './protocol.mjs';
const $=id=>document.getElementById(id), text=(id,value)=>{$(id).textContent=value;};
const invoke=window.__TAURI__?.core.invoke;
const number=new Intl.NumberFormat(undefined,{maximumFractionDigits:0});
const micronsPerCount=0.078;
const positionNumber=new Intl.NumberFormat(undefined,{minimumFractionDigits:3,maximumFractionDigits:3});
const position=counts=>positionNumber.format(counts*micronsPerCount);
let session=null,connected=false,busy=false,demo=false,streaming=true,snapshot=null,received=0,hello=null;
let pressureHistory=[],lastPressure=null;
let history=[],logs=[],epoch=null,lastSample=null,lastPong=0,lastPing=0,demoTimer=null;
function activity(message){logs.push(`${new Date().toLocaleTimeString()}  ${message}`);if(logs.length>100)logs.shift();text('log',logs.join('\n'));}
function notice(message,warn=false){text('notice',message);$('notice').classList.toggle('warn',warn);}
function controls(){
  text('connect',busy?'Please wait…':connected||demo?'Disconnect':'Connect Teensy');
  $('connect').disabled=busy||(!connected&&!demo&&!$('port').value);
  $('port').disabled=$('refresh').disabled=busy||connected||!!session;
  $('demo').disabled=busy||connected||demo||!!session;
  for(const id of ['ping','snapshot','stream'])$(id).disabled=!connected||!!session?.pending;
  for(const id of ['plunger-up','plunger-down','plunger-zero'])$(id).disabled=!connected||!hello?.motion||!!session?.pending||!snapshot?.plunger||snapshot.plunger.running||performance.now()-received>=3000;
  $('plunger-stop').disabled=!connected||!hello?.motion;
  text('stream',streaming?'Pause stream':'Resume stream');
}
function reset(){pressureHistory=[];lastPressure=null;snapshot=null;hello=null;history=[];epoch=null;lastSample=null;lastPong=0;lastPing=0;streaming=true;text('rtt','—');text('session-label','NO SESSION');render();draw();}
async function refresh(){
  if(!invoke){notice('Launch Teensy Panel using desktop/start.cmd or desktop/start.sh.',true);return;}
  const previous=$('port').value;$('refresh').disabled=true;
  try{
    const ports=await invoke('serial_ports');$('port').replaceChildren();
    if(!ports.length)$('port').add(new Option('No serial ports found',''));
    for(const port of ports)$('port').add(new Option(port.label,port.name));
    if(ports.some(p=>p.name===previous))$('port').value=previous;
    else if(ports.some(p=>p.teensy))$('port').value=ports.find(p=>p.teensy).name;
    activity(`${ports.length} native serial port(s) found`);
  }catch(e){notice(`Port discovery failed: ${e}`,true);}finally{controls();}
}
function accept(m){
  if(m.type==='log'){activity(m.message);return;}
  if(m.type==='hello'){hello=m;text('session-label','PROTOCOL 01');}
  if(m.type==='pong')lastPong=performance.now();
  if(m.type==='snapshot'){
    const now=performance.now();
    const restarted=snapshot&&elapsed(m.ms,snapshot.ms)>0x7fffffff;
    if(restarted){pressureHistory=[];lastPressure=null;}
    if(epoch!==m.epoch||restarted){history=[];lastSample=null;epoch=m.epoch;}
    snapshot=m;received=now;const e=m.values.EPOS,d=m.values.DPOS;
    if(e&&m.mounted&&m.cdc===2&&e.ms!==lastSample&&elapsed(m.ms,e.ms)<3000){
      history.push({time:now,e:e.value,d:d&&elapsed(m.ms,d.ms)<3000?d.value:null});lastSample=e.ms;
      if(history.length>600)history.shift();
    }
    const p=m.pressure;
    if(p?.valid&&elapsed(m.ms,p.ms)<1000&&p.ms!==lastPressure){
      pressureHistory.push({time:now,raw:p.raw,average:p.average});lastPressure=p.ms;
      if(pressureHistory.length>1200)pressureHistory.shift();
    }
    pressureHistory=pressureHistory.filter(p=>now-p.time<=60000);
    history=history.filter(p=>now-p.time<=60000);draw();
  }
  render();
}
async function request(command,quiet=false){
  if(!session)return;const start=performance.now(),promise=session.request(command);controls();
  try{
    const reply=await promise;
    if(command==='PING')text('rtt',`${Math.round(performance.now()-start)} ms`);
    if(reply.type==='ack')streaming=reply.stream;
    if(!quiet)activity(`${command} → ${reply.type} #${reply.id}`);
    if(command==='SNAPSHOT')activity(`Hardware ${JSON.stringify({plunger:reply.plunger,plunger_io:reply.plunger_io,pressure:reply.pressure})}`);
    if(connected&&!quiet)notice(streaming?'Connected locally. Receiving telemetry.':'Display stream paused. Hardware monitoring continues.');
    return reply;
  }catch(e){notice(String(e.message??e),true);activity(String(e.message??e));throw e;}finally{controls();}
}
async function disconnect(reason='Disconnected'){
  const old=session;session=null;connected=false;demo=false;busy=true;
  clearInterval(demoTimer);demoTimer=null;reset();controls();notice(reason);activity(reason);
  try{await old?.close();}catch(e){activity(`Close: ${e}`);}finally{busy=false;controls();render();}
}
$('connect').addEventListener('click',async()=>{
  if(connected||demo||session){await disconnect();return;}
  busy=true;reset();controls();
  try{
    const current=new NativeSession(invoke,m=>{if(session===current)accept(m);},e=>{if(session===current)void disconnect(`Connection lost: ${e.message??e}`);},e=>activity(`Ignored serial input: ${e.message??e}`));
    session=current;await current.open($('port').value);await request('HELLO');
    if(session!==current)throw Error('Connection closed during handshake');
    connected=true;await request('PING');await request('SNAPSHOT');
    activity('Native serial connection verified');
  }catch(e){await disconnect(`Could not connect: ${e.message??e}`);notice(`Could not connect: ${e.message??e}. Close other serial terminals and upload the current firmware.`,true);}
  finally{busy=false;controls();render();}
});
$('refresh').addEventListener('click',refresh);$('port').addEventListener('change',controls);
for(const[id,command]of[['ping','PING'],['snapshot','SNAPSHOT']])$(id).addEventListener('click',()=>void request(command).catch(()=>{}));
$('stream').addEventListener('click',()=>void request(streaming?'STREAM 0':'STREAM 1').catch(()=>{}));
for(const [id,direction] of [['plunger-up','U'],['plunger-down','D']])$(id).addEventListener('click',()=>{
  const speed=$('plunger-speed'),distance=$('plunger-distance');
  if(!speed.reportValidity()||!distance.reportValidity())return;
  void request(`PLUNGER_MOVE ${direction} ${speed.value} ${distance.value}`).catch(()=>{});
});
$('plunger-zero').addEventListener('click',()=>void request('PLUNGER_ZERO').catch(()=>{}));
$('plunger-stop').addEventListener('click',()=>void request('PLUNGER_STOP').catch(()=>disconnect('Stop could not be acknowledged; serial connection closed.')));
function age(s){return snapshot&&s?elapsed(snapshot.ms,s.ms)+performance.now()-received:Infinity;}
function ageText(s){const ms=age(s);return Number.isFinite(ms)?`${ms>=3000?'Stale · ':'Updated '}${(ms/1000).toFixed(1)}s ago`:'Awaiting feedback';}
function render(){
  const v=snapshot?.values??{},active=connected||demo,stalled=connected&&lastPong>0&&performance.now()-lastPong>6000;
  text('connection',demo?'Demo · simulated':stalled?'Link unresponsive':connected?'Connected':'Disconnected');
  $('connection').className=`pill ${demo||stalled?'warn':connected?'live':''}`;
  for(const[id,tag]of[['epos','EPOS'],['dpos','DPOS']]){text(id,v[tag]?position(v[tag].value):'—');text(`${id}-age`,ageText(v[tag]));$(id).closest('.metric').classList.toggle('stale',!!v[tag]&&age(v[tag])>=3000);}
  const stat=v.STAT,key=stat?String(stat.value):'none';
  text('stat-word',stat?`STAT ${stat.value} / 0x${stat.value.toString(16).padStart(6,'0')}`:'Waiting for STAT');
  text('stat-age',stat?`${ageText(stat)} · listed flags are active`:'No status received');$('flags').classList.toggle('stale-flags',!!stat&&age(stat)>=3000);
  if($('flags').dataset.word!==key){
    $('flags').dataset.word=key;$('flags').replaceChildren();const flags=stat?statusFlags(stat.value):[];
    if(!flags.length){const p=document.createElement('p');p.className='empty-copy';p.textContent=stat?'No status bits set. This is not a general health verdict.':'Active status flags appear with XD-C feedback.';$('flags').append(p);}
    for(const flag of flags){const span=document.createElement('span');span.className=`flag ${flag.kind}`;span.textContent=flag.name;span.title=`STAT bit ${flag.bit}`;$('flags').append(span);}
  }
  const pressure=snapshot?.pressure,pressureAge=pressure?elapsed(snapshot.ms,pressure.ms)+performance.now()-received:Infinity;
  const pressureValid=pressure?.valid,pressureFresh=pressureValid&&pressureAge<1000;
  text('pressure-raw',pressureValid?number.format(pressure.raw):'—');
  text('pressure-average',pressureValid?number.format(pressure.average):'—');
  text('pressure-samples',number.format(pressure?.samples??0));
  text('pressure-state',demo?'Simulated':!connected?'Offline':!hello?.pressure?'Unavailable':!pressureValid?pressureState(pressure):!pressureFresh?'Stale':pressure.saturated?'Saturated':'Receiving');
  text('pressure-age',pressureValid?`${pressureFresh?'Updated':'Stale'} · ${(pressureAge/1000).toFixed(1)}s ago`:'NO DATA');
  text('pressure-sensor',pressure?.sensor==='i2c'||!pressure?'I²C · address 0x5C':'HX711 · legacy firmware');
  text('pressure-detail',!active?'Raw readings appear when the I²C sensor responds.':!pressure?'Waiting for pressure telemetry.':`${pressureState(pressure)} · transfer errors ${pressure.errors??0}${pressure.sensor==='i2c'?` · WHO_AM_I ${pressure.who===null?'unavailable':`0x${pressure.who.toString(16).padStart(2,'0')}`} · temperature (raw) ${pressureValid?pressure.temperature:'—'}`:''}${pressure.pins?` · DT ${pressure.pins.dout_high?'HIGH':'LOW'} · SCK ${pressure.pins.sck_high?'HIGH (should be LOW)':'LOW'}`:''}`);
  const plunger=snapshot?.plunger;
  text('plunger-position',plunger?positionNumber.format(plunger.counts*1000/plunger.counts_per_mm):'—');
  text('plunger-state',!connected?'Waiting for connection':!hello?.motion?'Firmware does not support plunger control':!plunger?'Waiting for feedback':performance.now()-received>=3000?'Stale feedback':plunger.running?`Drive ${plunger.up?'up':'down'} enabled · PWM ${plunger.speed}`:'Stopped');
  text('plunger-pot',plunger?.pot!=null?`Potentiometer ${plunger.pot} / 4095 (${(100*plunger.pot/4095).toFixed(1)}%)`:'Potentiometer —');
  controls();
  text('board-state',demo?'Demo':connected?stalled?'Stalled':'Online':'Offline');
  text('firmware',hello?`Firmware ${hello.firmware} · protocol ${hello.protocol}`:'USB1 · waiting for connection');
  text('uptime',snapshot?`Uptime ${(snapshot.ms/1000).toFixed(1)}s · 10 Hz snapshots`:'Bare-metal firmware');
  const xd=snapshot?.vid===0x04d8&&snapshot?.pid===0xef4a,fresh=Object.values(v).some(s=>age(s)<3000);
  text('actuator-state',!active||!snapshot?'Unknown':!snapshot.vid?'Detached':!xd?'Other device':['Closed','Opening',fresh?'Receiving':'No feedback','Setup failed','Timed out'][snapshot.cdc]);
  text('actuator-detail',snapshot?.vid?`USB2 · ${snapshot.vid.toString(16).padStart(4,'0')}:${snapshot.pid.toString(16).padStart(4,'0')} · ${snapshot.mounted?'configured':'enumerating'}`:'USB2 host · 04d8:ef4a');
  text('serial',`Serial number ${v.SRNO?.value??'—'}`);
  for(const id of ['records','malformed','skipped'])text(id,number.format(snapshot?.[id]??0));
  text('coalesced',session?.coalesced??0);text('samples',`${history.length} samples`);
  text('stream-state',demo?'SIMULATED':!active?'NO DATA':!streaming?'PAUSED':!snapshot||performance.now()-received>=3000?'WAITING':'LIVE / 10 HZ');
  $('empty').hidden=history.length>0;
  if(stalled)notice('The Teensy is not answering connection checks. Displayed values may be stale; reconnect to retry.',true);
}
function drawSeries(canvasId,points,series,format,range){
  const canvas=$(canvasId),ctx=canvas.getContext('2d'),r=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
  canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);ctx.scale(dpr,dpr);
  const left=57,right=r.width-14,top=12,bottom=r.height-29,now=performance.now();
  let lo=range?.[0]??Infinity,hi=range?.[1]??-Infinity;for(const p of points)for(const[key]of series){const value=p[key];if(value!==null){lo=Math.min(lo,value);hi=Math.max(hi,value);}}
  if(!Number.isFinite(lo)){lo=-100;hi=100;}const pad=Math.max((hi-lo)*.15,1);lo-=pad;hi+=pad;
  ctx.font='9px system-ui';ctx.fillStyle='#929f88';ctx.textAlign='right';
  for(let i=0;i<5;i++){const y=top+(bottom-top)*i/4;ctx.strokeStyle='#edf1e7';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillText(format(hi-(hi-lo)*i/4),left-8,y+3);}
  ctx.textAlign='center';for(let i=0;i<=4;i++)ctx.fillText(i===4?'now':`−${60-i*15}s`,left+(right-left)*i/4,r.height-8);
  const x=t=>left+(1-(now-t)/60000)*(right-left),y=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
  ctx.save();ctx.beginPath();ctx.rect(left,top,right-left,bottom-top);ctx.clip();
  for(const[key,color,dash]of series){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dash);ctx.beginPath();let previous=null;for(const p of points){if(p[key]===null){previous=null;continue;}if(!previous||p.time-previous.time>3000)ctx.moveTo(x(p.time),y(p[key]));else ctx.lineTo(x(p.time),y(p[key]));previous=p;}ctx.stroke();}ctx.restore();
}
function draw(){
  const now=performance.now();
  drawSeries('plot',history.filter(p=>now-p.time<=60000),[['d','#b3bda5',[5,4]],['e','#4a8962',[]]],position);
  drawSeries('pressure-plot',pressureHistory.filter(p=>now-p.time<=60000),[['raw','#b3bda5',[5,4]],['average','#4a8962',[]]],number.format);
}
const plotResize=new ResizeObserver(draw);
for(const id of ['plot','pressure-plot'])plotResize.observe($(id).parentElement);
$('demo').addEventListener('click',()=>{
  if(session||busy)return;reset();demo=true;const start=performance.now();hello={firmware:'preview',protocol:1,pressure:true,motion:false};epoch=1;
  const make=ms=>({type:'snapshot',id:0,ms,epoch:1,vid:0x04d8,pid:0xef4a,mounted:true,cdc:2,records:Math.floor(ms/10),malformed:0,skipped:0,pressure:{sensor:'i2c',address:92,who:177,temperature:2500,state:2,errors:0,raw:Math.round(100000+15000*Math.sin(ms/1800)),average:Math.round(100000+12000*Math.sin(ms/1800-.4)),ms,samples:Math.floor(ms/100),valid:true,saturated:false},values:Object.fromEntries(Object.entries({EPOS:Math.round(1200+180*Math.sin(ms/6000)),DPOS:1200,STAT:352,SRNO:12345,SOFT:20103}).map(([tag,value])=>[tag,{value,ms}]))});
  for(let i=0;i<600;i++)history.push({time:start-60000+i*100,e:make(i*100).values.EPOS.value,d:1200});
  accept(make(60000));text('session-label','SIMULATED SESSION');notice('Demo mode — all readings are simulated. No serial port is open.');controls();
  demoTimer=setInterval(()=>accept(make(Math.floor(performance.now()-start)+60000)),100);
});
setInterval(()=>{if(connected&&session&&!session.pending&&performance.now()-lastPing>2000){lastPing=performance.now();void request('PING',true).catch(()=>{});}render();if(history.length||pressureHistory.length)draw();},500);
render();controls();void refresh();
