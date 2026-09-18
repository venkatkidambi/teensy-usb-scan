import {test} from 'node:test';
import assert from 'node:assert/strict';
import {NativeSession,parseMessage,statusFlags,elapsed} from '../ui/protocol.mjs';

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
