import test from 'node:test';
import assert from 'node:assert/strict';
import {setupExperiments,experimentControls,experimentMessage} from '../ui/experiment.js';

const tick=()=>new Promise(resolve=>setTimeout(resolve,130));
const flush=async()=>{for(let i=0;i<8;i++)await Promise.resolve();};
const deferred=()=>{let resolve;const promise=new Promise(ok=>{resolve=ok;});return {promise,resolve};};
function harness(mode='quasi_static') {
  const nodes=new Map(),calls=[],ipc=[];
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{value:'1',checked:false,elements:[],setAttribute(){},
      reportValidity:()=>true,getBoundingClientRect:()=>({width:0})});
    return nodes.get(id);
  };
  const previousDocument=globalThis.document,previousObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};globalThis.ResizeObserver=class{observe(){}};
  let files=['existing.czi'],scan=async()=>[...files],validate=async()=>{},choose=async()=>'/ZEN images';
  const state={connected:true,hello:{experiments:true,quasi_static_speed:true,sine_serial_displacement:true},session:{
    async startExperiment(config,append=false){calls.push({config,append});this.experimentOwned=true;return {run:calls.length};},
    async request(command){calls.push(command);},
  }};
  setupExperiments({state:()=>state,positionAge:()=>Infinity,controls:experimentControls,
    async invoke(command,args){
      ipc.push([command,args]);
      if(command==='experiment_validate')return validate();
      if(command==='zeiss_czi_files')return scan();
      if(command==='zeiss_choose_folder')return choose();
      assert.equal(command,'experiment_results');return [];
    },
  });
  node(`tab-${mode}`).onclick();
  node('experiment-zen-folder').value='/ZEN images';
  const enable=()=>{node('experiment-zen-enabled').checked=true;node('experiment-zen-enabled').onchange();};
  const submit=async()=>{await flush();await node('experiment-form').onsubmit({preventDefault(){}});};
  const abort=()=>node('experiment-abort').onclick();
  return {node,state,calls,ipc,enable,submit,abort,
    files(value){files=value;},scan(fn){scan=fn;},validate(fn){validate=fn;},choose(fn){choose=fn;},
    restore(){state.connected=false;experimentControls();globalThis.document=previousDocument;globalThis.ResizeObserver=previousObserver;},
  };
}

test('ZEN is opt-in; unchecked starts both modes immediately without folder access',async()=>{
  for(const mode of ['quasi_static','sinusoidal']){
    const h=harness(mode);
    try{
      assert.equal(h.node('experiment-zen-enabled').checked,false);
      assert.equal(h.node('experiment-zen-options').hidden,true);
      assert.equal(h.node('experiment-start-button').textContent,'Start experiment');
      await h.submit();
      assert.equal(h.calls.length,1);assert.equal(h.calls[0].config.kind,mode);
      assert.deepEqual(h.ipc.map(c=>c[0]),['experiment_results']);
    }finally{h.restore();}
  }
});

test('both modes wait for a new file, ignore existing files, and start exactly once',async()=>{
  for(const mode of ['quasi_static','sinusoidal']){
    const h=harness(mode);
    try{
      h.enable();await h.submit();
      assert.equal(h.calls.length,0);
      assert.equal(h.node('experiment-start-button').textContent,'Wait for ZEN image');
      assert.equal(h.node('experiment-start-button').disabled,true);
      assert.equal(h.node('experiment-abort').disabled,false);
      assert.equal(h.node('tab-dashboard').disabled,true);
      assert.equal(h.node('experiment-zen-folder').disabled,true);
      assert.equal(h.node('dashboard-panel').inert,true);
      assert.match(h.node('experiment-state').textContent,/Waiting for a new ZEN/);
      await tick();assert.equal(h.calls.length,0);
      await h.submit();assert.equal(h.calls.length,0);
      assert.equal(h.ipc.filter(c=>c[0]==='experiment_validate').length,1);
      h.files(['existing.czi','first.CZI','second.czi']);
      await tick();assert.equal(h.calls.length,1);
      assert.equal(h.calls[0].config.kind,mode);
      assert.equal(h.calls[0].append,false);
      assert.match(h.node('experiment-state').textContent,/ZEN image detected: first.CZI/);
      h.state.session.experimentOwned=false;experimentControls();
      h.files(['existing.czi','first.CZI','second.czi','third.czi']);
      await tick();assert.equal(h.calls.length,1);
      assert.equal(h.node('experiment-start-button').disabled,false);
      // Re-arming takes a fresh baseline, so images saved between runs are ignored.
      await h.submit();await tick();assert.equal(h.calls.length,1);
      h.files(['existing.czi','first.CZI','second.czi','third.czi','fourth.czi']);
      await tick();assert.equal(h.calls.length,2);
    }finally{h.restore();}
  }
});

test('ABORT and disconnect discard a pending scan and never restart after reconnect',async()=>{
  for(const stop of ['abort','disconnect','session','ownership']){
    const h=harness();
    try{
      h.enable();await h.submit();
      const pending=deferred();h.scan(()=>pending.promise);await tick();
      if(stop==='abort')await h.abort();
      if(stop==='disconnect')h.state.connected=false;
      if(stop==='session')h.state.session={...h.state.session};
      if(stop==='ownership')h.state.experimentUnresolved=true;
      experimentControls();
      pending.resolve(['existing.czi','new.czi']);await flush();
      h.state.connected=true;h.state.experimentUnresolved=false;experimentControls();await tick();
      assert.equal(h.calls.length,0);
      assert.match(h.node('experiment-state').textContent,/ZEN wait cancelled/);
      assert.equal(h.node('experiment-start-button').disabled,false);
    }finally{h.restore();}
  }
});

test('ABORT during validation or baseline scanning prevents delayed arming',async()=>{
  for(const phase of ['validation','baseline']){
    const h=harness();
    try{
      const pending=deferred(),entered=deferred();
      const wait=()=>{entered.resolve();return pending.promise;};
      if(phase==='validation')h.validate(wait);else h.scan(wait);
      h.enable();const arming=h.submit();await entered.promise;
      await h.abort();pending.resolve(['existing.czi']);await arming;await tick();
      assert.equal(h.calls.length,0);
      assert.equal(h.node('experiment-start-button').disabled,false);
      assert.match(h.node('experiment-state').textContent,/ZEN wait cancelled/);
      assert.equal(h.ipc.filter(c=>c[0]==='zeiss_czi_files').length,phase==='validation'?0:1);
    }finally{h.restore();}
  }
});

test('invalid folders, validation errors and read failures stop the wait without motion',async()=>{
  for(const failure of ['blank','validation','baseline','poll']){
    const h=harness();
    try{
      h.enable();
      if(failure==='blank')h.node('experiment-zen-folder').value='  ';
      if(failure==='validation')h.validate(async()=>{throw Error('Invalid amplitude');});
      if(failure==='baseline')h.scan(async()=>{throw Error('Folder missing');});
      await h.submit();
      if(failure==='poll'){h.scan(async()=>{throw Error('Permission denied');});await tick();}
      assert.equal(h.calls.length,0);
      assert.equal(h.node('experiment-start-button').disabled,false);
      assert.match(h.node('experiment-state').textContent,/Choose the folder|Could not arm ZEN|ZEN wait stopped/);
    }finally{h.restore();}
  }
});

test('a file trigger waits for an in-flight telemetry request and is consumed even if START fails',async()=>{
  const h=harness();
  try{
    h.enable();await h.submit();h.state.session.pending=true;
    h.files(['existing.czi','new.czi']);await tick();assert.equal(h.calls.length,0);
    h.state.session.startExperiment=async config=>{h.calls.push(config);throw Error('Controller rejected START');};
    h.state.session.pending=false;await tick();
    assert.equal(h.calls.length,1);
    assert.match(h.node('experiment-state').textContent,/START failed: Controller rejected/);
    await tick();assert.equal(h.calls.length,1);
  }finally{h.restore();}
});

test('one ZEN image starts a full sweep with the existing per-run progression',async()=>{
  const h=harness('sinusoidal');
  try{
    h.enable();h.node('experiment-sweep-enabled').checked=true;h.node('experiment-frequencies').value='1, 2';
    await h.submit();
    assert.deepEqual(h.ipc.find(c=>c[0]==='experiment_validate')[1].configs.map(c=>c.timing),[1,2]);
    h.files(['existing.czi','new.czi']);await tick();
    assert.equal(h.calls.length,1);assert.equal(h.calls[0].config.timing,1);
    experimentMessage({type:'experiment_dataset',dataset:{archive_index:0,metadata:{run:1,config:{kind:'sinusoidal',frequency_mhz:1000,cycles:3},outcome:'completed',count:0,elapsed_ms:0},samples:[]}});
    h.state.session.experimentOwned=false;experimentControls();await tick();
    assert.equal(h.calls.length,2);assert.equal(h.calls[1].config.timing,2);
    assert.equal(h.calls[1].append,true);
    await h.abort();
  }finally{h.restore();}
});

test('folder picker supports selection, cancellation and errors',async()=>{
  const h=harness();
  try{
    h.enable();h.node('experiment-zen-folder').value='';
    await h.node('experiment-zen-browse').onclick();assert.equal(h.node('experiment-zen-folder').value,'/ZEN images');
    h.choose(async()=>null);await h.node('experiment-zen-browse').onclick();
    assert.equal(h.node('experiment-zen-folder').value,'/ZEN images');
    h.choose(async()=>{throw Error('Dialog failed');});await h.node('experiment-zen-browse').onclick();
    assert.match(h.node('experiment-state').textContent,/Could not choose ZEN folder/);
    assert.equal(h.node('experiment-zen-browse').disabled,false);
  }finally{h.restore();}
});
