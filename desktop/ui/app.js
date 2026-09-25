import {setupExperiments,experimentControls,experimentMessage} from './experiment.js';
import {NativeSession,statusFlags,elapsed,pressureState,xeryonMoveCommand} from './protocol.mjs';
const $=id=>document.getElementById(id), text=(id,value)=>{$(id).textContent=value;};
const invoke=window.__TAURI__?.core.invoke;
const number=new Intl.NumberFormat(undefined,{maximumFractionDigits:0});
const micronsPerCount=0.078;
const positionNumber=new Intl.NumberFormat(undefined,{minimumFractionDigits:3,maximumFractionDigits:3});
const position=counts=>positionNumber.format(counts*micronsPerCount);
let session=null,connected=false,busy=false,demo=false,streaming=true,snapshot=null,received=0,hello=null;
let limitsInitialized=false;
let pressureHistory=[],lastPressure=null,experimentUnresolved=false;
let history=[],logs=[],epoch=null,lastSample=null,lastPong=0,lastPing=0,demoTimer=null;
function activity(message){logs.push(`${new Date().toLocaleTimeString()}  ${message}`);if(logs.length>100)logs.shift();text('log',logs.join('\n'));}
function notice(message,warn=false){text('notice',message);$('notice').classList.toggle('warn',warn);}
function controls(){
  for(const id of ['xeryon-distance','plunger-distance','plunger-speed'])$(id).disabled=!!session?.experimentOwned;
  text('connect',busy?'Please wait…':connected||demo?'Disconnect':'Connect Teensy');
  $('connect').disabled=busy||(!connected&&!demo&&!$('port').value);
  $('port').disabled=$('refresh').disabled=busy||connected||!!session;
  $('demo').disabled=busy||connected||demo||!!session;
  for(const id of ['ping','snapshot','stream'])$(id).disabled=!connected||!!session?.pending;
  for(const id of ['plunger-up','plunger-down','plunger-zero'])$(id).disabled=!connected||!hello?.motion||!!session?.pending||!snapshot?.plunger||snapshot.plunger.running||performance.now()-received>=3000;
  for(const id of ['plunger-min','plunger-max','plunger-apply-limits'])$(id).disabled=!connected||!hello?.plunger_limits||!!session?.pending||!!session?.experimentOwned||!snapshot?.plunger||snapshot.plunger.running||performance.now()-received>=3000;
  if(hello?.plunger_limits){
    const p=snapshot?.plunger,limits=p?.limits;
    $('plunger-up').disabled ||= !limits||p.pot==null||p.pot>=Math.floor(limits.max*4095/100);
    $('plunger-down').disabled ||= !limits||p.pot==null||p.pot<=Math.ceil(limits.min*4095/100);
  }
  $('plunger-stop').disabled=!connected||!hello?.motion;
  for(const id of ['xeryon-up','xeryon-down'])$(id).disabled=!!xeryonBlocked()||!!session?.pending;
  $('xeryon-stop').disabled=!connected||!hello?.xeryon||!snapshot?.mounted||snapshot.cdc!==2||snapshot.vid!==0x04d8||snapshot.pid!==0xef4a;
  text('stream',streaming?'Pause stream':'Resume stream');
  if(session?.experimentOwned)for(const id of ['ping','snapshot','stream','plunger-up','plunger-down','plunger-zero','plunger-stop','xeryon-up','xeryon-down','xeryon-stop','xeryon-speed','xeryon-distance','plunger-speed','plunger-distance'])$(id).disabled=true;
  experimentControls();
}
function xeryonBlocked(){
  if(!connected)return 'Connect Teensy to move the actuator.';
  if(!hello?.xeryon)return 'Upload current firmware to enable Xeryon movement.';
  if(hello.xeryon_speed&&!hello.xeryon_optional_speed&&$('xeryon-speed').value.trim()==='')return 'This firmware requires a speed. Enter one, or flash current firmware to use the controller setting.';
  if(!snapshot?.mounted||snapshot.cdc!==2||snapshot.vid!==0x04d8||snapshot.pid!==0xef4a)return 'Connect the Xeryon to the USB2 host port.';
  const v=snapshot.values;
  if(age(v.EPOS)>=3000||age(v.STAT)>=3000)return 'Waiting for fresh position and status.';
  const stat=v.STAT.value;
  if(stat&((1<<2)|(1<<3)|(1<<12)|(1<<16)|(1<<18)|(1<<21)))return 'Controller fault — check Controller status.';
  if(snapshot.xeryon_motion?.state===1)return 'Moving — waiting for target feedback.';
  if(stat&((1<<9)|(1<<13))||(stat&(1<<5))&&!(stat&(1<<10)))return 'Actuator is already moving.';
  return '';
}
function reset(){limitsInitialized=false;pressureHistory=[];lastPressure=null;snapshot=null;hello=null;history=[];epoch=null;lastSample=null;lastPong=0;lastPing=0;streaming=true;text('rtt','—');text('session-label','NO SESSION');render();draw();}
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
  if(m.type.startsWith('experiment_')){
    if(m.type==='experiment_state')experimentUnresolved=m.state!=='dashboard';
    if(m.type==='experiment_ack'&&m.action==='data_received')experimentUnresolved=false;
    experimentMessage(m);
    if(m.type==='experiment_ack'&&m.action==='data_received'){
      lastPong=performance.now();lastPing=0;
      if(connected&&!hello)void request('HELLO',true).catch(()=>{});
    }
    render();return;
  }
  if(m.type==='log'){activity(m.message);return;}
  if(m.type==='hello'){hello=m;text('session-label','PROTOCOL 01');}
  if(m.type==='pong')lastPong=performance.now();
  if(m.type==='snapshot'){
    const now=performance.now();
    const restarted=snapshot&&elapsed(m.ms,snapshot.ms)>0x7fffffff;
    if(restarted){pressureHistory=[];lastPressure=null;}
    if(epoch!==m.epoch||restarted){history=[];lastSample=null;epoch=m.epoch;}
    if(restarted)limitsInitialized=false;
    if(m.plunger?.limits&&!limitsInitialized){
      $('plunger-min').value=m.plunger.limits.min;$('plunger-max').value=m.plunger.limits.max;limitsInitialized=true;
    }
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
    if(connected&&!quiet)notice(streaming?'Connected locally. Receiving telemetry.':'Display stream paused. Hardware monitoring continues.');
    if(reply.type==='xeryon_ack')notice(reply.action==='move'?`Move queued · target ${position(reply.target)} µm. Waiting for controller feedback.`:'Stop queued. Check actuator feedback.');
    return reply;
  }catch(e){notice(String(e.message??e),true);activity(String(e.message??e));throw e;}finally{controls();}
}
async function disconnect(reason='Disconnected'){
  if(session?.experimentOwned)experimentUnresolved=true;
  if(experimentUnresolved)reason+=' · Autonomous operation may continue; reconnect to recover or ABORT.';
  const old=session;session=null;connected=false;demo=false;busy=true;
  clearInterval(demoTimer);demoTimer=null;reset();controls();notice(reason);activity(reason);
  try{await old?.close();}catch(e){activity(`Close: ${e}`);}finally{busy=false;controls();render();}
}
$('connect').addEventListener('click',async()=>{
  if(connected||demo||session){await disconnect();return;}
  busy=true;reset();controls();
  try{
    const current=new NativeSession(invoke,m=>{if(session===current)accept(m);},e=>{if(session===current)void disconnect(`Connection lost: ${e.message??e}`);},e=>activity(`Ignored serial input: ${e.message??e}`));
    session=current;await current.open($('port').value);await current.waitForStartup();
    if(current.experimentOwned){connected=true;notice('Autonomous experiment recovered. Normal desktop traffic is suspended.');return;}
    await request('HELLO');
    if(session!==current)throw Error('Connection closed during handshake');
    connected=true;await request('PING');await request('SNAPSHOT');
    activity('Native serial connection verified');
  }catch(e){await disconnect(`Could not connect: ${e.message??e}`);notice(`Could not connect: ${e.message??e}. Close other serial terminals and upload the current firmware.`,true);}
  finally{busy=false;controls();render();}
});
$('refresh').addEventListener('click',refresh);$('port').addEventListener('change',controls);
$('xeryon-speed').addEventListener('input',render);
for(const[id,command]of[['ping','PING'],['snapshot','SNAPSHOT']])$(id).addEventListener('click',()=>void request(command).catch(()=>{}));
$('stream').addEventListener('click',()=>void request(streaming?'STREAM 0':'STREAM 1').catch(()=>{}));
$('plunger-limits-form').addEventListener('submit',async event=>{
  event.preventDefault();
  const min=$('plunger-min'),max=$('plunger-max');
  if(!min.reportValidity()||!max.reportValidity())return;
  if(+min.value>=+max.value){notice('Minimum position must be below maximum position.',true);return;}
  try{
    await request(`PLUNGER_LIMITS ${min.value} ${max.value}`);
    await request('SNAPSHOT',true);
  }catch{}
});
for(const [id,direction] of [['plunger-up','U'],['plunger-down','D']])$(id).addEventListener('click',()=>{
  const speed=$('plunger-speed'),distance=$('plunger-distance');
  if(!speed.reportValidity()||!distance.reportValidity())return;
  void request(`PLUNGER_MOVE ${direction} ${speed.value} ${distance.value}`).catch(()=>{});
});
for(const [id,direction] of [['xeryon-up','U'],['xeryon-down','D']])$(id).addEventListener('click',()=>{
  const distance=$('xeryon-distance'),speed=$('xeryon-speed');
  if(!distance.reportValidity()||(hello?.xeryon_speed&&speed.value.trim()!==''&&!speed.reportValidity())||xeryonBlocked())return;
  try{
    void request(xeryonMoveCommand(direction,distance.value,speed.value,hello?.xeryon_speed)).catch(()=>{});
  }
  catch(e){notice(e.message,true);}
});
$('xeryon-stop').addEventListener('click',()=>void request('XERYON_STOP').catch(()=>disconnect('Xeryon Stop was not acknowledged; check the actuator.')));
$('plunger-zero').addEventListener('click',()=>void request('PLUNGER_ZERO').catch(()=>{}));
$('plunger-stop').addEventListener('click',()=>void request('PLUNGER_STOP').catch(()=>disconnect('Stop could not be acknowledged; serial connection closed.')));
function age(s){return snapshot&&s?elapsed(snapshot.ms,s.ms)+performance.now()-received:Infinity;}
function ageText(s){const ms=age(s);return Number.isFinite(ms)?`${ms>=3000?'Stale · ':'Updated '}${(ms/1000).toFixed(1)}s ago`:'Awaiting feedback';}
function render(){
  const v=snapshot?.values??{},active=connected||demo,stalled=connected&&!session?.experimentOwned&&lastPong>0&&performance.now()-lastPong>6000;
  text('connection',session?.experimentOwned?'Autonomous experiment':demo?'Demo · simulated':stalled?'Link unresponsive':connected?'Connected':'Disconnected');
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
  text('pressure-state',demo?'Simulated':!connected?'Offline':!hello?.pressure?'Unavailable':!pressureValid?pressureState(pressure):!pressureFresh?'Stale':'Receiving');
  text('pressure-age',pressureValid?`${pressureFresh?'Updated':'Stale'} · ${(pressureAge/1000).toFixed(1)}s ago`:'NO DATA');
  text('pressure-sensor','LPS22HHTR · I²C 0x5C · 200 Hz');
  text('pressure-detail',!active?'LPS22HHTR readings appear when the sensor responds.':!pressure?'Waiting for pressure telemetry.':`${pressureState(pressure)} · errors ${pressure.errors} · WHO_AM_I ${pressure.who===null?'unavailable':`0x${pressure.who.toString(16).padStart(2,'0')}`} · temperature ${pressureValid?`${(pressure.temperature/100).toFixed(2)} °C`:'—'}`);
  const xm=snapshot?.xeryon_motion;
  $('xeryon-speed').disabled=!connected||!hello?.xeryon_speed;
  text('xeryon-motion-state',demo?'Demo — motion controls are disabled.':xeryonBlocked()||(xm?.state===2?'Target reached.':xm?.state===3?'Stop requested.':xm?.state===4?'Move interrupted — check feedback and the activity log.':hello?.xeryon_speed?'Ready. Enter travel; leave speed blank to use the controller setting.':'Ready. Speed uses the controller setting with this firmware. Enter travel, then choose Up or Down.'));
  text('xeryon-target',xm?.target!=null?`Requested target ${position(xm.target)} µm`:'Travel rounds to the nearest 0.078 µm.');
  const plunger=snapshot?.plunger;
  text('plunger-position',plunger?positionNumber.format(plunger.counts*1000/plunger.counts_per_mm):'—');
  text('plunger-state',!connected?'Waiting for connection':!hello?.motion?'Firmware does not support plunger control':!plunger?'Waiting for feedback':performance.now()-received>=3000?'Stale feedback':plunger.running?`Drive ${plunger.up?'up':'down'} enabled · PWM ${plunger.speed}`:'Stopped');
  const limits=plunger?.limits;
  text('plunger-limits-state',!connected?'Connect to read active position limits.':!hello?.plunger_limits?'Upload current firmware to enable position limits.':limits?`Active limits: ${limits.min}%–${limits.max}% · changes apply while stopped.`:'Waiting for active position limits.');
  const gauge=$('plunger-gauge'),potPercent=plunger?.pot==null?null:100*plunger.pot/4095;
  const potFresh=connected&&performance.now()-received<3000;
  const outside=potPercent!=null&&limits&&(potPercent<limits.min||potPercent>limits.max);
  gauge.classList.toggle('outside',!!outside);gauge.classList.toggle('stale',!potFresh);
  for(const id of ['plunger-gauge-window','plunger-gauge-min','plunger-gauge-max'])$(id).hidden=!limits;
  if(limits){
    $('plunger-gauge-window').style.bottom=`${limits.min}%`;
    $('plunger-gauge-window').style.height=`${limits.max-limits.min}%`;
    for(const key of ['min','max']){
      $(`plunger-gauge-${key}`).style.bottom=`${limits[key]}%`;
      text(`plunger-gauge-${key}`,`${key==='min'?'Min':'Max'} ${limits[key]}%`);
    }
  }
  $('plunger-gauge-marker').hidden=potPercent==null;
  if(potPercent!=null)$('plunger-gauge-marker').style.bottom=`${potPercent}%`;
  const gaugeStatus=potPercent==null?'No feedback':!potFresh?'Stale feedback':!limits?'Limits unavailable':outside?'Outside limits':'Within limits';
  text('plunger-gauge-value',potPercent==null?'—':`${potPercent.toFixed(1)}%`);
  text('plunger-gauge-status',gaugeStatus);
  gauge.setAttribute('aria-label',`Potentiometer ${potPercent==null?'unavailable':`${potPercent.toFixed(1)} percent`}. ${gaugeStatus}.${limits?` Active limits ${limits.min} to ${limits.max} percent.`:''}`);
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
function drawSeries(canvasId,points,series,format){
  const canvas=$(canvasId),ctx=canvas.getContext('2d'),r=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
  canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);ctx.scale(dpr,dpr);
  const left=57,right=r.width-14,top=12,bottom=r.height-29,now=performance.now();
  let lo=Infinity,hi=-Infinity;for(const p of points)for(const[key]of series){const value=p[key];if(value!==null){lo=Math.min(lo,value);hi=Math.max(hi,value);}}
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
  const make=ms=>({type:'snapshot',id:0,ms,epoch:1,vid:0x04d8,pid:0xef4a,mounted:true,cdc:2,records:Math.floor(ms/10),malformed:0,skipped:0,pressure:{sensor:'lps22hh',address:92,odr_hz:200,who:179,temperature:2500,state:2,errors:0,raw:Math.round(100000+15000*Math.sin(ms/1800)),average:Math.round(100000+12000*Math.sin(ms/1800-.4)),ms,samples:Math.floor(ms/5),valid:true},values:Object.fromEntries(Object.entries({EPOS:Math.round(1200+180*Math.sin(ms/6000)),DPOS:1200,STAT:352,SRNO:12345,SOFT:20103}).map(([tag,value])=>[tag,{value,ms}]))});
  for(let i=0;i<600;i++)history.push({time:start-60000+i*100,e:make(i*100).values.EPOS.value,d:1200});
  accept(make(60000));text('session-label','SIMULATED SESSION');notice('Demo mode — all readings are simulated. No serial port is open.');controls();
  demoTimer=setInterval(()=>accept(make(Math.floor(performance.now()-start)+60000)),100);
});
setInterval(()=>{if(connected&&session&&!session.experimentOwned&&!session.pending&&performance.now()-lastPing>2000){lastPing=performance.now();void request('PING',true).catch(()=>{});}render();if(history.length||pressureHistory.length)draw();},500);
$('xeryon-speed').value='';
setupExperiments({state:()=>({session,connected,hello,snapshot,experimentUnresolved}),positionAge:()=>age(snapshot?.values.EPOS),invoke,controls,activity});
render();controls();void refresh();
