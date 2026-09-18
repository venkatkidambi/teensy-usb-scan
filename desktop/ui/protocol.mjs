export const FLAGS = ['Amplifiers enabled (XRTA only)','End stop','Thermal protection 1','Thermal protection 2','Force zero','Motor on','Closed loop','Encoder at index','Encoder valid','Searching index','Position reached','Error compensation','Encoder error','Scanning','Left end stop','Right end stop','Error limit','Searching optimal frequency','Safety timeout (TOU2)','EtherCAT acknowledge','Unused bit 20','Position fail (TOU3)','Unknown bit 22','Unknown bit 23'];
const faults = new Set([2,3,12,16,18,21]);
export const statusFlags = word => FLAGS.flatMap((name,bit) => word & (1<<bit) ? [{name,bit,kind:faults.has(bit)?'fault':bit>=19?'special':''}] : []);
export const elapsed = (now,before) => (now-before)>>>0;
const u32 = n => Number.isInteger(n) && n>=0 && n<=0xffffffff;
const object = v => v!==null && typeof v==='object' && !Array.isArray(v);
const tags = new Set(['EPOS','DPOS','STAT','SRNO','SOFT','FREQ','TIME','SYNC']);
export function parseMessage(line) {
  if(line.startsWith('#')) return {type:'log',message:line.slice(1).trim()};
  const m=JSON.parse(line);
  if(!object(m)||!u32(m.id)) throw Error('Invalid message envelope');
  if(m.type==='snapshot') {
    for(const key of ['ms','epoch','vid','pid','cdc','records','malformed','skipped']) if(!u32(m[key])) throw Error(`Invalid ${key}`);
    if(m.vid>65535||m.pid>65535||m.cdc>4||typeof m.mounted!=='boolean'||!object(m.values)) throw Error('Invalid device state');
    for(const [tag,s] of Object.entries(m.values)) {
      if(!tags.has(tag)||!object(s)||!Number.isSafeInteger(s.value)||!u32(s.ms)) throw Error('Invalid sample');
      if(tag==='STAT'&&(s.value<0||s.value>0xffffff)) throw Error('Invalid STAT');
    }
  } else if(m.type==='hello') {
    if(m.protocol!==1||m.board!=='Teensy 4.1'||typeof m.firmware!=='string'||m.motion!==false||m.pressure!==false) throw Error('Incompatible firmware; upload this project’s telemetry build');
  } else if(m.type==='pong') { if(!u32(m.ms)) throw Error('Invalid pong'); }
  else if(m.type==='ack') { if(typeof m.stream!=='boolean') throw Error('Invalid acknowledgement'); }
  else if(m.type==='error') { if(typeof m.message!=='string') throw Error('Invalid error'); }
  else throw Error('Unknown message type');
  return m;
}

// Native Rust owns the serial port. This class only invokes local Tauri IPC.
export class NativeSession {
  constructor(invoke,onMessage,onLost,onError,timeout=2500) {
    Object.assign(this,{invoke,onMessage,onLost,onError,timeout});
    this.id=0;this.session=null;this.pending=null;this.closed=false;this.coalesced=0;
  }
  async open(port) {
    this.session=await this.invoke('serial_connect',{port});
    this.poll();
  }
  async poll() {
    if(this.closed) return;
    try {
      const batch=await this.invoke('serial_poll',{session:this.session});
      if(this.closed||batch.session!==this.session) return;
      this.coalesced=batch.coalesced;
      for(const line of batch.messages) {
        try {
          const m=parseMessage(line);
          if(this.pending&&(m.id===this.pending.id||m.type==='error'&&m.id===0)) {
            const p=this.pending;this.pending=null;clearTimeout(p.timer);
            if(m.type==='error') p.reject(Error(m.message));
            else if(m.type!==p.expected) p.reject(Error('Unexpected reply type'));
            else p.resolve(m);
          }
          this.onMessage(m);
        } catch(e) { this.onError(e); }
      }
      if(batch.closed||batch.error) throw Error(batch.error||'Serial device disconnected');
      this.timer=setTimeout(()=>this.poll(),50);
    } catch(e) {
      if(!this.closed) { this.rejectPending(e);this.onLost(e); }
    }
  }
  rejectPending(error) {
    if(!this.pending) return;
    const p=this.pending;this.pending=null;clearTimeout(p.timer);p.reject(error);
  }
  request(command) {
    if(this.closed||this.session===null) return Promise.reject(Error('Not connected'));
    if(this.pending) return Promise.reject(Error('Wait for the current request'));
    if(!['HELLO','PING','SNAPSHOT','STREAM 0','STREAM 1'].includes(command)) return Promise.reject(Error('Unsupported command'));
    const [name,arg]=command.split(' ');const id=this.id=this.id===0xffffffff?1:this.id+1;
    return new Promise((resolve,reject)=>{
      this.pending={id,resolve,reject,expected:{HELLO:'hello',PING:'pong',SNAPSHOT:'snapshot',STREAM:'ack'}[name],timer:setTimeout(()=>this.rejectPending(Error(`${name} timed out`)),this.timeout)};
      this.invoke('serial_send',{session:this.session,command:`${name} ${id}${arg===undefined?'':` ${arg}`}`}).catch(e=>{if(this.pending?.id===id)this.rejectPending(Error(String(e)));});
    });
  }
  async close() {
    if(this.closed) return;
    this.closed=true;clearTimeout(this.timer);this.rejectPending(Error('Connection closed'));
    if(this.session!==null) await this.invoke('serial_disconnect',{session:this.session});
  }
}
