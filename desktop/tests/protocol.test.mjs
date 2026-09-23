import {test} from 'node:test';
import assert from 'node:assert/strict';
import {NativeSession,parseMessage,statusFlags,elapsed,pressureState,xeryonTravelNm,xeryonMoveCommand} from '../ui/protocol.mjs';

test('telemetry schema and status mapping',()=>{
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:1240,pid:61258,mounted:true,cdc:2,records:3,malformed:0,skipped:0,values:{STAT:{value:1344,ms:99}}};
  assert.deepEqual(parseMessage(JSON.stringify(m)),m);
  assert.deepEqual(statusFlags(1344).map(f=>f.name),['Closed loop','Encoder valid','Position reached']);
  assert.equal(statusFlags(1<<20)[0].name,'Unused bit 20');
  assert.equal(statusFlags(1<<23)[0].name,'Unknown bit 23');
  m.values.STAT.value=-1;assert.throws(()=>parseMessage(JSON.stringify(m)));
  assert.throws(()=>parseMessage('{"type":"hello","id":1,"protocol":2}'));
  assert.equal(elapsed(5,0xfffffffe),7);
});
function backend(reply=true){
  const queue=[],sent=[];let closed=false;
  return {sent,get closed(){return closed;},async invoke(name,args){
    if(name==='serial_connect')return 7;
    assert.equal(args.session,7);
    if(name==='serial_poll')return {session:7,messages:queue.splice(0),coalesced:0,closed:false,error:null};
    if(name==='serial_disconnect'){closed=true;return;}
    assert.equal(name,'serial_send');sent.push(args.command);
    if(reply){const[name,id,arg]=args.command.split(' ');queue.push(JSON.stringify(name==='PING'?{type:'pong',id:+id,ms:99}:name==='HELLO'?{type:'hello',id:+id,protocol:1,firmware:'0.1.0',board:'Teensy 4.1',motion:false,pressure:false}:{type:'ack',id:+id,stream:arg==='1'}));}
  }};
}
test('native bridge handshake, acknowledgements and close',async()=>{
  const b=backend(),messages=[];
  const s=new NativeSession(b.invoke,m=>messages.push(m),assert.fail,assert.fail,500);
  try{
    await s.open('COM6');assert.equal((await s.request('HELLO')).protocol,1);
    assert.equal((await s.request('PING')).ms,99);
    assert.equal((await s.request('STREAM 0')).stream,false);
    await assert.rejects(s.request('MOVE 1'),/Unsupported/);
    assert.deepEqual(b.sent,['HELLO 1','PING 2','STREAM 3 0']);assert.equal(messages.length,3);
  }finally{await s.close();}
  assert(b.closed);
});
test('timeouts, pending requests and disconnect cleanup',async()=>{
  const b=backend(false),s=new NativeSession(b.invoke,()=>{},assert.fail,assert.fail,15);
  await s.open('COM6');const p=s.request('PING');
  await assert.rejects(s.request('HELLO'),/current request/);await assert.rejects(p,/timed out/);
  const p2=s.request('PING'),rejection=assert.rejects(p2,/Connection closed/);
  await s.close();await rejection;assert(b.closed);assert.equal(s.pending,null);
});
test('unplug invalidates pending request',async()=>{
  let lost;const b=backend(false),s=new NativeSession(async(name,args)=>name==='serial_poll'?{session:7,messages:[],closed:true,error:'unplugged',coalesced:0}:b.invoke(name,args),()=>{},e=>{lost=e.message;},assert.fail);
  await s.open('COM6');await new Promise(resolve=>setTimeout(resolve,1));assert.equal(lost,'unplugged');await s.close();
});

test('plunger schema, move framing and Stop preemption',async()=>{
  assert.equal(parseMessage(JSON.stringify({type:'hello',id:1,protocol:1,board:'Teensy 4.1',firmware:'test',motion:true,pressure:false})).motion,true);
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:0,pid:0,mounted:false,cdc:0,records:0,malformed:0,skipped:0,values:{},plunger:{counts:-16688,counts_per_mm:16688,running:false,speed:0,up:true,pot:4095}};
  assert.equal(parseMessage(JSON.stringify(m)).plunger.counts,-16688);
  m.plunger.pot=4096;assert.throws(()=>parseMessage(JSON.stringify(m)));
  const b=backend(false),s=new NativeSession(b.invoke,()=>{},assert.fail,assert.fail,500);
  await s.open('COM6');
  try {
  for(const command of ['PLUNGER_MOVE U 0 1','PLUNGER_MOVE U 256 1','PLUNGER_MOVE U 1 0','PLUNGER_MOVE U 1 128684303','PLUNGER_MOVE X 1 1'])await assert.rejects(s.request(command),/Unsupported/);
  const moving=s.request('PLUNGER_MOVE U 64 10');
  const interrupted=assert.rejects(moving,/Interrupted by Stop/);
  const stopping=s.request('PLUNGER_STOP');
  const closed=assert.rejects(stopping,/Connection closed/);
  assert.deepEqual(b.sent,['PLUNGER_MOVE 1 U 64 10','PLUNGER_STOP 2']);
  await s.close();await interrupted;await closed;
  } finally { await s.close(); }
});

test('HX711 pressure schema preserves inverted signed range and freshness',()=>{
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:0,pid:0,mounted:false,cdc:0,records:0,malformed:0,skipped:0,values:{},pressure:{raw:8388608,average:-8388607,ms:99,samples:1,valid:true,saturated:true}};
  assert.deepEqual(parseMessage(JSON.stringify(m)).pressure,m.pressure);
  m.pressure.valid=false;assert.equal(parseMessage(JSON.stringify(m)).pressure.valid,false);
  for(const [key,value] of [['raw',8388609],['raw',-8388608],['average',1.5],['ms',-1],['samples',-1],['valid',1],['saturated','yes']]){
    const bad={...m,pressure:{...m.pressure,[key]:value}};
    assert.throws(()=>parseMessage(JSON.stringify(bad)),/pressure/);
  }
  delete m.pressure;assert.doesNotThrow(()=>parseMessage(JSON.stringify(m)));
});

test('hardware diagnostics distinguish missing data, settling and transfer failure',()=>{
  assert.equal(pressureState({state:0,valid:false}),'Waiting for HX711 data');
  assert.equal(pressureState({state:1,valid:false}),'Settling channel B');
  assert.equal(pressureState({state:3,valid:false}),'No conversions for 1s');
  assert.equal(pressureState({state:4,valid:false}),'DOUT stayed low after read');
  assert.equal(pressureState({valid:true}),'Receiving');
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:0,pid:0,mounted:false,cdc:0,records:0,malformed:0,skipped:0,values:{},plunger_io:{standby:true,ain1:true,ain2:false,encoder:3,pwm_count:123,pwm_compare:16447,pwm_running:true,pwm_enabled:true}};
  assert.deepEqual(parseMessage(JSON.stringify(m)).plunger_io,m.plunger_io);
  m.plunger_io.standby=1;assert.throws(()=>parseMessage(JSON.stringify(m)),/plunger IO/);
});

test('HX711 pin readback validates booleans',()=>{
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:0,pid:0,mounted:false,cdc:0,records:0,malformed:0,skipped:0,values:{},pressure:{raw:0,average:0,ms:0,samples:0,valid:false,saturated:false,state:0,errors:0,pins:{dout_high:true,sck_high:false,sck_requested_high:false}}};
  assert.deepEqual(parseMessage(JSON.stringify(m)).pressure.pins,m.pressure.pins);
  m.pressure.pins.sck_high=0;assert.throws(()=>parseMessage(JSON.stringify(m)),/pin levels/);
});


test('I²C pressure accepts unsigned 24-bit values and validates sensor diagnostics',()=>{
  const m={type:'snapshot',id:0,ms:100,epoch:1,vid:0,pid:0,mounted:false,cdc:0,records:0,malformed:0,skipped:0,values:{},pressure:{sensor:'i2c',address:92,who:177,temperature:65535,raw:0xffffff,average:0xffffff,ms:99,samples:16,valid:true,saturated:false,state:2,errors:0}};
  assert.deepEqual(parseMessage(JSON.stringify(m)).pressure,m.pressure);
  for(const [key,value] of [['raw',-1],['average',0x1000000],['who',256],['temperature',65536],['address',93],['sensor','unknown']]) {
    assert.throws(()=>parseMessage(JSON.stringify({...m,pressure:{...m.pressure,[key]:value}})),/pressure/);
  }
  m.pressure.who=null;assert.doesNotThrow(()=>parseMessage(JSON.stringify(m)));
  assert.equal(pressureState({...m.pressure,state:0}),'Initializing I²C sensor');
  assert.equal(pressureState({...m.pressure,state:3}),'I²C transfer timed out');
  assert.equal(pressureState({...m.pressure,state:4}),'I²C bus error / no acknowledgement');
});


test('Xeryon travel units, schema, framing and Stop preemption',async()=>{
  assert.equal(xeryonTravelNm('10'),10000);
  assert.equal(xeryonTravelNm('0.078'),78);
  assert.equal(xeryonTravelNm('1.234'),1234);
  assert.equal(xeryonMoveCommand('U','10','10000',true),'XERYON_MOVE U 10000 10000');
  assert.equal(xeryonMoveCommand('U','10','',true),'XERYON_MOVE U 10000');
  assert.equal(xeryonMoveCommand('U','10','10000',false),'XERYON_MOVE U 10000');
  for(const value of ['0','-1','0.077','1.2345','Infinity','1e3','2617245.619'])assert.throws(()=>xeryonTravelNm(value));
  const ack={type:'xeryon_ack',id:1,action:'move',target:-28};
  assert.deepEqual(parseMessage(JSON.stringify(ack)),ack);
  assert.throws(()=>parseMessage(JSON.stringify({...ack,target:33554432})));
  assert.throws(()=>parseMessage(JSON.stringify({...ack,action:'reset'})));
  const b=backend(false),s=new NativeSession(b.invoke,()=>{},assert.fail,assert.fail,500);
  await s.open('COM6');
  try{
    for(const cmd of ['XERYON_MOVE U 10000 0','XERYON_MOVE U 10000 16777216','XERYON_MOVE U 10000 -1','XERYON_MOVE U 10000 1.5','XERYON_MOVE U 0 10000','XERYON_MOVE U 77 10000','XERYON_MOVE D -100 10000','XERYON_MOVE X 100 10000','XERYON_MOVE U 2617245619 10000','XERYON_MOVE U 1.5 10000','DPOS=100'])await assert.rejects(s.request(cmd),/Unsupported/);
    const moving=s.request('XERYON_MOVE D 10000 10000');
    assert.equal(s.pending.expected,'xeryon_ack');
    const interrupted=assert.rejects(moving,/Interrupted by Stop/);
    const stopping=s.request('XERYON_STOP');
    const closed=assert.rejects(stopping,/Connection closed/);
    assert.deepEqual(b.sent,['XERYON_MOVE 1 D 10000 10000','XERYON_STOP 2']);
    await s.close();await interrupted;await closed;
  }finally{await s.close();}
});

test('legacy Xeryon firmware can receive a move without a speed argument',async()=>{
  const b=backend(false),s=new NativeSession(b.invoke,()=>{},assert.fail,assert.fail,500);
  await s.open('COM6');
  try {
    const moving=s.request(xeryonMoveCommand('D','10','10000',false));
    assert.deepEqual(b.sent,['XERYON_MOVE 1 D 10000']);
    await s.close();
    await assert.rejects(moving,/Connection closed/);
  } finally { await s.close(); }
});


test('Xeryon reply resolves the move and motion telemetry remains optional',async()=>{
  const snapshot={type:'snapshot',id:0,ms:100,epoch:1,vid:1240,pid:61258,mounted:true,cdc:2,records:3,malformed:0,skipped:0,values:{},xeryon_motion:{state:1,target:228}};
  assert.deepEqual(parseMessage(JSON.stringify(snapshot)).xeryon_motion,{state:1,target:228});
  for(const motion of [{state:5,target:0},{state:1,target:33554432},{state:1,target:1.5},null])assert.throws(()=>parseMessage(JSON.stringify({...snapshot,xeryon_motion:motion})),/Xeryon/);
  const hello={type:'hello',id:1,protocol:1,board:'Teensy 4.1',firmware:'0.1.7',motion:true,pressure:true,xeryon:true,xeryon_speed:true,xeryon_optional_speed:true};
  assert.equal(parseMessage(JSON.stringify(hello)).xeryon,true);
  assert.equal(parseMessage(JSON.stringify(hello)).xeryon_speed,true);
  assert.equal(parseMessage(JSON.stringify(hello)).xeryon_optional_speed,true);
  assert.throws(()=>parseMessage(JSON.stringify({...hello,xeryon_speed:1})),/speed capability/);
  assert.throws(()=>parseMessage(JSON.stringify({...hello,xeryon_optional_speed:1})),/optional speed capability/);
  delete hello.xeryon_optional_speed;assert.equal(parseMessage(JSON.stringify(hello)).xeryon_optional_speed,undefined);
  delete hello.xeryon_speed;assert.equal(parseMessage(JSON.stringify(hello)).xeryon_speed,undefined);
  delete hello.xeryon;assert.equal(parseMessage(JSON.stringify(hello)).xeryon,undefined);
  const queue=[];
  const s=new NativeSession(async(name,args)=>{
    if(name==='serial_connect')return 7;
    if(name==='serial_poll')return {session:7,messages:queue.splice(0),coalesced:0,closed:false};
    if(name==='serial_send'){
      const [cmd,id]=args.command.split(' ');
      queue.push(JSON.stringify({type:'xeryon_ack',id:+id,action:cmd==='XERYON_MOVE'?'move':'stop',...(cmd==='XERYON_MOVE'?{target:228}:{})}));
    }
  },()=>{},assert.fail,assert.fail,500);
  try{
    await s.open('COM6');
    assert.equal((await s.request('XERYON_MOVE U 10000 10000')).target,228);
    assert.equal((await s.request('XERYON_STOP')).action,'stop');
  }finally{await s.close();}
});


test('experiment session suppresses normal requests and ABORT preempts START',async()=>{
  const sent=[],queue=[];
  const s=new NativeSession(async(name,args)=>{
    if(name==='serial_connect')return 7;
    if(name==='serial_poll')return {session:7,messages:queue.splice(0),closed:false,coalesced:0};
    if(name==='experiment_start'){sent.push({name,...args});return;}
    if(name==='serial_send'){sent.push(args.command);return;}
  },()=>{},assert.fail,assert.fail,1000);
  await s.open('COM6');
  const start=s.startExperiment({kind:'sinusoidal',start_um:0,increment_um:1,timing:1,repeats:1});
  const interrupted=assert.rejects(start,/Interrupted by Stop/);
  assert(s.experimentOwned);
  for(const command of ['PING','SNAPSHOT','STREAM 1','XERYON_MOVE U 78','PLUNGER_STOP'])await assert.rejects(s.request(command),/only ABORT/);
  const abort=s.request('ABORT');
  queue.push(JSON.stringify({type:'experiment_ack',id:2,run:1,action:'abort'}));
  await abort;await interrupted;
  assert.equal(sent.length,2);assert.equal(sent[0].name,'experiment_start');assert.equal(sent[1],'ABORT 2');
  assert(s.experimentOwned); // abort does not release retained data
  queue.push(JSON.stringify({type:'experiment_ack',id:0xf0000000,run:1,action:'data_received'}));
  await new Promise(resolve=>setTimeout(resolve,60));assert(!s.experimentOwned);
  await s.close();
});

test('reconnecting to an autonomous run uses passive state reception',async()=>{
  const queue=[JSON.stringify({type:'experiment_state',id:0,run:9,state:'running'})],sent=[];
  const s=new NativeSession(async(name,args)=>{
    if(name==='serial_connect')return 7;
    if(name==='serial_poll')return {session:7,messages:queue.splice(0),closed:false,coalesced:0,experiment_owned:true};
    if(name==='serial_send')sent.push(args.command);
  },()=>{},assert.fail,assert.fail,500);
  try{await s.open('COM6');await s.waitForStartup();assert(s.experimentOwned);await assert.rejects(s.request('HELLO'),/only ABORT/);assert.deepEqual(sent,[]);}
  finally{await s.close();}
});


test('ABORT cannot overtake a START still being submitted through native IPC',async()=>{
  const sent=[];let acceptStart;
  const s=new NativeSession(async(name,args)=>{
    if(name==='serial_connect')return 7;
    if(name==='serial_poll')return {session:7,messages:[],closed:false,coalesced:0};
    if(name==='experiment_start'){await new Promise(resolve=>{acceptStart=resolve;});sent.push('START');}
    if(name==='serial_send')sent.push(args.command);
  },()=>{},assert.fail,assert.fail,500);
  await s.open('COM6');
  const starting=s.startExperiment({});const interrupted=assert.rejects(starting,/Interrupted by Stop/);
  const abort=s.request('ABORT');const aborted=assert.rejects(abort,/Connection closed/);
  assert.deepEqual(sent,[]);acceptStart();await new Promise(resolve=>setTimeout(resolve,1));
  assert.deepEqual(sent,['START','ABORT 2']);await s.close();await interrupted;await aborted;
});


test('plunger limit configuration validates and frames requests',async()=>{
  const b=backend(false),s=new NativeSession(b.invoke,()=>{},assert.fail,assert.fail,500);
  await s.open('COM6');
  try {
    for(const cmd of ['PLUNGER_LIMITS 75 30','PLUNGER_LIMITS 30 30','PLUNGER_LIMITS 0 101','PLUNGER_LIMITS -1 75','PLUNGER_LIMITS 30.5 75'])await assert.rejects(s.request(cmd),/Unsupported/);
    const pending=s.request('PLUNGER_LIMITS 30 75');
    const rejection=assert.rejects(pending,/Connection closed/);
    assert.deepEqual(b.sent,['PLUNGER_LIMITS 1 30 75']);
    assert.equal(s.pending.expected,'plunger_ack');
    assert.doesNotThrow(()=>parseMessage(JSON.stringify({type:'plunger_ack',id:1,action:'limits'})));
    await s.close();await rejection;
  }finally{await s.close();}
});
