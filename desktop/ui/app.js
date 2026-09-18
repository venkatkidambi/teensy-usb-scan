import {NativeSession,statusFlags,elapsed} from './protocol.mjs';
const $=id=>document.getElementById(id), text=(id,value)=>{$(id).textContent=value;};
const invoke=window.__TAURI__?.core.invoke;
const number=new Intl.NumberFormat(undefined,{maximumFractionDigits:0});
let session=null,connected=false,busy=false,demo=false,streaming=true,snapshot=null,received=0,hello=null;
let history=[],logs=[],epoch=null,lastSample=null,lastPong=0,lastPing=0,demoTimer=null;
function activity(message){logs.push(`${new Date().toLocaleTimeString()}  ${message}`);if(logs.length>100)logs.shift();text('log',logs.join('\n'));}
function notice(message,warn=false){text('notice',message);$('notice').classList.toggle('warn',warn);}
function controls(){
  text('connect',busy?'Please wait…':connected||demo?'Disconnect':'Connect Teensy');
  $('connect').disabled=busy||(!connected&&!demo&&!$('port').value);
  $('port').disabled=$('refresh').disabled=busy||connected||!!session;
  $('demo').disabled=busy||connected||demo||!!session;
  for(const id of ['ping','snapshot','stream'])$(id).disabled=!connected||!!session?.pending;
  text('stream',streaming?'Pause stream':'Resume stream');
}
function reset(){snapshot=null;hello=null;history=[];epoch=null;lastSample=null;lastPong=0;lastPing=0;streaming=true;text('rtt','—');text('session-label','NO SESSION');render();draw();}
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
    if(epoch!==m.epoch||snapshot&&elapsed(m.ms,snapshot.ms)>0x7fffffff){history=[];lastSample=null;epoch=m.epoch;}
    snapshot=m;received=now;const e=m.values.EPOS,d=m.values.DPOS;
    if(e&&m.mounted&&m.cdc===2&&e.ms!==lastSample&&elapsed(m.ms,e.ms)<3000){
      history.push({time:now,e:e.value,d:d&&elapsed(m.ms,d.ms)<3000?d.value:null});lastSample=e.ms;
      if(history.length>600)history.shift();
    }
    history=history.filter(p=>now-p.time<=60000);draw();
  }
  render();
}
async function request(command){
  if(!session)return;const start=performance.now(),promise=session.request(command);controls();
  try{
    const reply=await promise;
    if(command==='PING')text('rtt',`${Math.round(performance.now()-start)} ms`);
    if(reply.type==='ack')streaming=reply.stream;
    activity(`${command} → ${reply.type} #${reply.id}`);
    if(connected)notice(streaming?'Connected locally. Receiving XD-C feedback; no actuator commands are sent.':'Display stream paused. The Teensy continues receiving actuator feedback.');
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
function age(s){return snapshot&&s?elapsed(snapshot.ms,s.ms)+performance.now()-received:Infinity;}
function ageText(s){const ms=age(s);return Number.isFinite(ms)?`${ms>=3000?'Stale · ':'Updated '}${(ms/1000).toFixed(1)}s ago`:'Awaiting feedback';}
function render(){
  const v=snapshot?.values??{},active=connected||demo,stalled=connected&&lastPong>0&&performance.now()-lastPong>6000;
  text('connection',demo?'Demo · simulated':stalled?'Link unresponsive':connected?'Connected':'Disconnected');
  $('connection').className=`pill ${demo||stalled?'warn':connected?'live':''}`;
  for(const[id,tag]of[['epos','EPOS'],['dpos','DPOS'],['freq','FREQ']]){text(id,v[tag]?number.format(v[tag].value):'—');text(`${id}-age`,ageText(v[tag]));$(id).closest('.metric').classList.toggle('stale',!!v[tag]&&age(v[tag])>=3000);}
  const stat=v.STAT,key=stat?String(stat.value):'none';
  text('stat-word',stat?`STAT ${stat.value} / 0x${stat.value.toString(16).padStart(6,'0')}`:'Waiting for STAT');
  text('stat-age',stat?`${ageText(stat)} · listed flags are active`:'No status received');$('flags').classList.toggle('stale-flags',!!stat&&age(stat)>=3000);
  if($('flags').dataset.word!==key){
    $('flags').dataset.word=key;$('flags').replaceChildren();const flags=stat?statusFlags(stat.value):[];
    if(!flags.length){const p=document.createElement('p');p.className='empty-copy';p.textContent=stat?'No status bits set. This is not a general health verdict.':'Active status flags appear with XD-C feedback.';$('flags').append(p);}
    for(const flag of flags){const span=document.createElement('span');span.className=`flag ${flag.kind}`;span.textContent=flag.name;span.title=`STAT bit ${flag.bit}`;$('flags').append(span);}
  }
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
function draw(){
  const canvas=$('plot'),ctx=canvas.getContext('2d'),r=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
  canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);ctx.scale(dpr,dpr);
  const left=57,right=r.width-14,top=12,bottom=r.height-29,now=performance.now(),points=history.filter(p=>now-p.time<=60000);
  let lo=Infinity,hi=-Infinity;for(const p of points)for(const value of[p.e,p.d])if(value!==null){lo=Math.min(lo,value);hi=Math.max(hi,value);}
  if(!Number.isFinite(lo)){lo=-100;hi=100;}const pad=Math.max((hi-lo)*.15,1);lo-=pad;hi+=pad;
  ctx.font='9px system-ui';ctx.fillStyle='#929f88';ctx.textAlign='right';
  for(let i=0;i<5;i++){const y=top+(bottom-top)*i/4;ctx.strokeStyle='#edf1e7';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillText(number.format(hi-(hi-lo)*i/4),left-8,y+3);}
  ctx.textAlign='center';for(let i=0;i<=4;i++)ctx.fillText(i===4?'now':`−${60-i*15}s`,left+(right-left)*i/4,r.height-8);
  const x=t=>left+(1-(now-t)/60000)*(right-left),y=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
  ctx.save();ctx.beginPath();ctx.rect(left,top,right-left,bottom-top);ctx.clip();
  for(const[key,color,dash]of[['d','#b3bda5',[5,4]],['e','#4a8962',[]]]){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dash);ctx.beginPath();let previous=null;for(const p of points){if(p[key]===null){previous=null;continue;}if(!previous||p.time-previous.time>3000)ctx.moveTo(x(p.time),y(p[key]));else ctx.lineTo(x(p.time),y(p[key]));previous=p;}ctx.stroke();}ctx.restore();
}
new ResizeObserver(draw).observe($('plot').parentElement);
$('demo').addEventListener('click',()=>{
  if(session||busy)return;reset();demo=true;const start=performance.now();hello={firmware:'preview',protocol:1};epoch=1;
  const make=ms=>({type:'snapshot',id:0,ms,epoch:1,vid:0x04d8,pid:0xef4a,mounted:true,cdc:2,records:Math.floor(ms/10),malformed:0,skipped:0,values:Object.fromEntries(Object.entries({EPOS:Math.round(1200+180*Math.sin(ms/6000)),DPOS:1200,FREQ:84000,STAT:352,SRNO:12345,SOFT:20103}).map(([tag,value])=>[tag,{value,ms}]))});
  for(let i=0;i<600;i++)history.push({time:start-60000+i*100,e:make(i*100).values.EPOS.value,d:1200});
  accept(make(60000));text('session-label','SIMULATED SESSION');notice('Demo mode — all readings are simulated. No serial port is open.');controls();
  demoTimer=setInterval(()=>accept(make(Math.floor(performance.now()-start)+60000)),100);
});
setInterval(()=>{if(connected&&session&&!session.pending&&performance.now()-lastPing>2000){lastPing=performance.now();void request('PING').catch(()=>{});}render();if(history.length)draw();},500);
render();controls();void refresh();
