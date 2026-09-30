export const FLAGS = ['Amplifiers enabled (XRTA only)','End stop','Thermal protection 1','Thermal protection 2','Force zero','Motor on','Closed loop','Encoder at index','Encoder valid','Searching index','Position reached','Error compensation','Encoder error','Scanning','Left end stop','Right end stop','Error limit','Searching optimal frequency','Safety timeout (TOU2)','EtherCAT acknowledge','Unused bit 20','Position fail (TOU3)','Unknown bit 22','Unknown bit 23'];
const faults = new Set([2,3,12,16,18,21]);
export const statusFlags = word => FLAGS.flatMap((name,bit) => word & (1<<bit) ? [{name,bit,kind:faults.has(bit)?'fault':bit>=19?'special':''}] : []);
export const elapsed = (now,before) => (now-before)>>>0;
const u32 = n => Number.isInteger(n) && n>=0 && n<=0xffffffff;
const object = v => v!==null && typeof v==='object' && !Array.isArray(v);
const xeryonPosition = n => Number.isInteger(n) && n>=-33554432 && n<=33554431;
export function xeryonTravelNm(value) {
  // Decimal µm input, transmitted as integer nm to avoid floating-point firmware.
  if(!/^[0-9]+(?:\.[0-9]{1,3})?$/.test(String(value))) throw Error('Enter travel in µm with up to three decimal places');
  const nm=Math.round(Number(value)*1000);
  if(!Number.isSafeInteger(nm)||nm<78||nm>2617245618) throw Error('Travel must be between 0.078 and 2617245.618 µm');
  return nm;
}
export function xeryonMoveCommand(direction, travel, speed, speedSupported) {
  const move=`XERYON_MOVE ${direction} ${xeryonTravelNm(travel)}`;
  return speedSupported&&String(speed).trim()!==''?`${move} ${Number(speed)}`:move;
}
const tags = new Set(['EPOS','DPOS','STAT','SRNO','SOFT','FREQ','TIME','SYNC']);
export function pressureState(p) {
  return ['Initializing LPS22HHTR','Waiting for first sample','Receiving','I²C transfer timed out','I²C bus error / no acknowledgement','Unexpected sensor ID (expected 0xB3)','No fresh pressure conversion','Sensor configuration mismatch'][p?.state]??'Waiting for LPS22HHTR';
}
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
    if(m.xeryon_motion!==undefined) {
      const x=m.xeryon_motion;
      if(!object(x)||!u32(x.state)||x.state>4||(x.target!==null&&!xeryonPosition(x.target))) throw Error('Invalid Xeryon motion telemetry');
    }
    if(m.pressure!==undefined) {
      const p=m.pressure;
      const counts=n=>Number.isInteger(n)&&n>=-8388608&&n<=8388607;
      if(!object(p)||p.sensor!=='lps22hh'||!counts(p.raw)||!counts(p.average)||!u32(p.ms)||!u32(p.samples)||typeof p.valid!=='boolean') throw Error('Invalid pressure telemetry');
      if(p.address!==92||p.odr_hz!==200||(p.who!==null&&(!u32(p.who)||p.who>255))||!Number.isInteger(p.temperature)||p.temperature< -32768||p.temperature>32767) throw Error('Invalid pressure sensor diagnostics');
      if(!u32(p.state)||p.state>7||!u32(p.errors)) throw Error('Invalid pressure state');
      if(p.valid&&(p.who!==0xb3||p.state!==2)) throw Error('Invalid pressure sensor identity or state');
    }
    if(m.plunger_io!==undefined) {
      const io=m.plunger_io;
      if(!object(io)||!['standby','ain1','ain2','pwm_running','pwm_enabled'].every(k=>typeof io[k]==='boolean')||!u32(io.encoder)||io.encoder>3||!['pwm_count','pwm_compare'].every(k=>Number.isInteger(io[k])&&io[k]>=-32768&&io[k]<=32767)) throw Error('Invalid plunger IO');
    }
    if(m.plunger!==undefined) {
      const p=m.plunger;
      if(p?.limits!==undefined&&(!object(p.limits)||!u32(p.limits.min)||!u32(p.limits.max)||p.limits.min>=p.limits.max||p.limits.max>100)) throw Error('Invalid plunger limits');
      if(!object(p)||!Number.isInteger(p.counts)||p.counts< -2147483648||p.counts>2147483647||p.counts_per_mm!==16688||typeof p.running!=='boolean'||typeof p.up!=='boolean'||!u32(p.speed)||p.speed>255||(p.pot!==null&&(!u32(p.pot)||p.pot>4095))) throw Error('Invalid plunger telemetry');
    }
  } else if(m.type==='hello') {
    if(m.protocol!==1||m.board!=='Teensy 4.1'||typeof m.firmware!=='string'||typeof m.motion!=='boolean'||typeof m.pressure!=='boolean') throw Error('Incompatible firmware; upload this project’s telemetry build');
    if(m.sine_velocity!==undefined&&typeof m.sine_velocity!=='boolean')throw Error('Invalid sine_velocity capability');
    if(m.sine_position!==undefined&&typeof m.sine_position!=='boolean')throw Error('Invalid sine_position capability');
    if(m.quasi_static_speed!==undefined&&typeof m.quasi_static_speed!=='boolean')throw Error('Invalid quasi_static_speed capability');
    if(m.experiments!==undefined&&typeof m.experiments!=='boolean')throw Error('Invalid experiment capability');
    if(m.plunger_limits!==undefined&&typeof m.plunger_limits!=='boolean') throw Error('Invalid plunger limits capability');
    if(m.xeryon_speed!==undefined&&typeof m.xeryon_speed!=='boolean') throw Error('Invalid Xeryon speed capability');
    if(m.xeryon_optional_speed!==undefined&&typeof m.xeryon_optional_speed!=='boolean') throw Error('Invalid Xeryon optional speed capability');
    if(m.xeryon!==undefined&&typeof m.xeryon!=='boolean') throw Error('Invalid Xeryon capability');
  } else if(m.type==='pong') { if(!u32(m.ms)) throw Error('Invalid pong'); }
  else if(m.type==='ack') { if(typeof m.stream!=='boolean') throw Error('Invalid acknowledgement'); }
  else if(m.type==='xeryon_ack') { if(!['move','stop'].includes(m.action)||(m.action==='move'&&!xeryonPosition(m.target))) throw Error('Invalid Xeryon acknowledgement'); }
  else if(m.type==='plunger_ack') { if(!['move','stop','zero','limits'].includes(m.action)) throw Error('Invalid plunger acknowledgement'); }
  else if(m.type==='experiment_state') { if(!['dashboard','running','data_ready'].includes(m.state)||!u32(m.run)) throw Error('Invalid experiment state'); }
  else if(m.type==='experiment_ack') { if(!['start','abort','data_received'].includes(m.action)||!u32(m.run)) throw Error('Invalid experiment acknowledgement'); }
  else if(m.type==='error') { if(typeof m.message!=='string') throw Error('Invalid error'); }
  else throw Error('Unknown message type');
  return m;
}

// Native Rust owns the serial port. This class only invokes local Tauri IPC.
export class NativeSession {
  constructor(invoke,onMessage,onLost,onError,timeout=2500) {
    Object.assign(this,{invoke,onMessage,onLost,onError,timeout});
    this.startupSeen=false;this.experimentOwned=false;this.id=0;this.session=null;this.pending=null;this.closed=false;this.coalesced=0;
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
      if(typeof batch.experiment_owned==='boolean') this.experimentOwned=batch.experiment_owned;
      for(const line of batch.messages) {
        try {
          const m=parseMessage(line);
          if(m.type==='experiment_state'||m.type==='snapshot')this.startupSeen=true;
          if(m.type==='experiment_state') this.experimentOwned=m.state!=='dashboard';
          if(m.type==='experiment_ack'&&m.action==='start') this.experimentOwned=true;
          if(m.type==='experiment_ack'&&m.action==='data_received') this.experimentOwned=false;
          if(this.pending&&(m.id===this.pending.id||m.type==='error'&&m.id===0)) {
            const p=this.pending;this.pending=null;clearTimeout(p.timer);
            if(m.type==='error') p.reject(Error(m.message));
            else if(m.type!==p.expected) p.reject(Error('Unexpected reply type'));
            else p.resolve(m);
          }
          this.onMessage(m);
        } catch(e) { this.onError(e); }
      }
      if(batch.dataset) this.onMessage({type:'experiment_dataset',dataset:batch.dataset});
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
    if(this.experimentOwned&&command!=='ABORT') return Promise.reject(Error('Experiment owns hardware; only ABORT is allowed'));
    if(command==='ABORT'||command==='PLUNGER_STOP'||command==='XERYON_STOP') this.rejectPending(Error('Interrupted by Stop'));
    if(this.pending) return Promise.reject(Error('Wait for the current request'));
    const move=/^PLUNGER_MOVE [UD] ([0-9]+) ([0-9]+)$/.exec(command);
    const validMove=move&&+move[1]>0&&+move[1]<=255&&Number.isSafeInteger(+move[2])&&+move[2]>0&&Math.floor((+move[2]*16688+500)/1000)<=2147483647;
    const xeryonMove=/^XERYON_MOVE [UD] ([0-9]+)(?: ([0-9]+))?$/.exec(command);
    const validXeryon=xeryonMove&&Number.isSafeInteger(+xeryonMove[1])&&+xeryonMove[1]>=78&&+xeryonMove[1]<=2617245618&&(xeryonMove[2]===undefined||Number.isInteger(+xeryonMove[2])&&+xeryonMove[2]>=1&&+xeryonMove[2]<=16777215);
    const limits=/^PLUNGER_LIMITS ([0-9]+) ([0-9]+)$/.exec(command);
    const validLimits=limits&&+limits[1]<+limits[2]&&+limits[2]<=100;
    if(!validLimits&&!validMove&&!validXeryon&&!['HELLO','PING','SNAPSHOT','STREAM 0','STREAM 1','PLUNGER_STOP','PLUNGER_ZERO','XERYON_STOP','ABORT'].includes(command)) return Promise.reject(Error('Unsupported command'));
    const [name,...args]=command.split(' ');const arg=args.length?args.join(' '):undefined;const id=this.id=this.id===0xefffffff?1:this.id+1;
    return new Promise((resolve,reject)=>{
      this.pending={id,resolve,reject,expected:{ABORT:'experiment_ack',XERYON_MOVE:'xeryon_ack',XERYON_STOP:'xeryon_ack',HELLO:'hello',PING:'pong',SNAPSHOT:'snapshot',STREAM:'ack',PLUNGER_LIMITS:'plunger_ack',PLUNGER_MOVE:'plunger_ack',PLUNGER_STOP:'plunger_ack',PLUNGER_ZERO:'plunger_ack'}[name],timer:setTimeout(()=>this.rejectPending(Error(`${name} timed out`)),(command==='ABORT'||command==='PLUNGER_STOP'||command==='XERYON_STOP')?500:this.timeout)};
      const send=()=>this.invoke('serial_send',{session:this.session,command:`${name} ${id}${arg===undefined?'':` ${arg}`}`});
      // Preserve native queue order if ABORT preempts an in-flight START IPC.
      const sending=command==='ABORT'&&this.startSubmission?this.startSubmission.then(send,send):send();
      sending.catch(e=>{if(this.pending?.id===id)this.rejectPending(Error(String(e)));});
    });
  }
  async waitForStartup() {
    const end=Date.now()+this.timeout;
    while(!this.startupSeen&&!this.closed&&Date.now()<end)await new Promise(resolve=>setTimeout(resolve,20));
    if(!this.startupSeen)throw Error('No startup state received; reconnect to recover safely');
  }
  startExperiment(config,appendResults=false) {
    if(this.closed||this.session===null||this.pending||this.experimentOwned) return Promise.reject(Error('Experiment or request already active'));
    const id=this.id=this.id===0xefffffff?1:this.id+1;
    this.experimentOwned=true;
    return new Promise((resolve,reject)=>{
      this.pending={id,resolve,reject,expected:'experiment_ack',timer:setTimeout(()=>this.rejectPending(Error('START timed out; reconnect to recover state or send ABORT')),this.timeout)};
      const submission=this.invoke('experiment_start',{session:this.session,id,config,appendResults});
      this.startSubmission=submission;
      submission.then(()=>{if(this.startSubmission===submission)this.startSubmission=null;},e=>{
        if(this.startSubmission===submission)this.startSubmission=null;
        this.experimentOwned=false;
        if(this.pending?.id===id)this.rejectPending(Error(String(e)));
      });
    });
  }
  async close() {
    if(this.closed) return;
    this.closed=true;clearTimeout(this.timer);this.rejectPending(Error('Connection closed'));
    if(this.session!==null) await this.invoke('serial_disconnect',{session:this.session});
  }
}
