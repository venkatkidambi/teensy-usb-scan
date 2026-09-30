import test from 'node:test';
import assert from 'node:assert/strict';
import {setupExperiments,experimentControls,experimentMessage} from '../ui/experiment.js';

const tick=()=>new Promise(resolve=>setTimeout(resolve,10));
function harness() {
  const nodes=new Map(),calls=[];
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{value:'1',checked:false,elements:[],setAttribute(){},reportValidity:()=>true,getBoundingClientRect:()=>({width:0})});
    return nodes.get(id);
  };
  const oldDocument=globalThis.document,oldObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};globalThis.ResizeObserver=class{observe(){}};
  const state={connected:true,hello:{experiments:true,sine_position:true},session:{
    async startExperiment(config){calls.push(config);this.experimentOwned=true;return {run:calls.length};},
    async request(command){calls.push(command);},
  }};
  const ipc=[];let validate=async()=>{};
  setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls,
    async invoke(command,args){ipc.push([command,args]);if(command==='experiment_validate')await validate();return null;},
  });
  node('tab-sinusoidal').onclick();
  node('experiment-sweep-enabled').checked=true;node('experiment-frequencies').value='1, 5, 10';
  const submit=async()=>{await tick();return node('experiment-form').onsubmit({preventDefault(){}});};
  const result=(run,outcome='completed')=>experimentMessage({type:'experiment_dataset',dataset:{
    archive_index:run+10,
    metadata:{run,config:{kind:'sinusoidal',frequency_mhz:calls[run-1].timing*1000,cycles:3},outcome,reason:'test',count:0,elapsed_ms:0},samples:[],
  }});
  const release=()=>{state.session.experimentOwned=false;state.experimentUnresolved=false;experimentMessage({type:'experiment_ack',action:'data_received',run:calls.length});};
  return {node,calls,state,ipc,submit,result,release,setValidate(fn){validate=fn;},restore(){state.connected=false;experimentControls();globalThis.document=oldDocument;globalThis.ResizeObserver=oldObserver;}};
}
test('sweep prevalidates every frequency, waits for download and release, and exports selected history',async()=>{
  const h=harness();
  try{
    await h.submit();
    assert.deepEqual(h.ipc.find(c=>c[0]==='experiment_validate')[1].configs.map(c=>c.timing),[1,5,10]);
    assert.equal(h.calls.length,1);assert.equal(h.node('experiment-start-button').disabled,true);
    h.result(1);await tick();assert.equal(h.calls.length,1);
    h.state.session.pending=true;h.release();await tick();assert.equal(h.calls.length,1);
    h.state.session.pending=false;experimentControls();await tick();assert.equal(h.calls.length,2);
    // ACK may arrive before the result notification.
    h.release();await tick();assert.equal(h.calls.length,2);
    h.result(2);await tick();assert.equal(h.calls.length,3);
    h.result(3);h.release();await tick();
    assert.match(h.node('experiment-sweep-status').textContent,/Sweep complete: 3/);
    assert.deepEqual(h.calls.map(c=>c.timing),[1,5,10]);
    assert.ok(h.calls.every(c=>c.start_um===0&&c.increment_um===1&&c.repeats===3));
    h.node('experiment-run').value='0';h.node('experiment-run').onchange();
    assert.match(h.node('experiment-result-summary').textContent,/Run 1/);
    await h.node('experiment-save').onclick();
    // Native identity can differ from the visible list after recovery/reconnect.
    assert.deepEqual(h.ipc.at(-1),['experiment_save_csv',{index:11}]);
  }finally{h.restore();}
});
test('abort, fault, disconnect and failed START each cancel remaining frequencies',async()=>{
  for(const stop of ['abort','fault','disconnect','start_failure']){
    const h=harness();
    try{
      if(stop==='start_failure')h.state.session.startExperiment=async()=>{throw Error('Rejected');};
      await h.submit();
      if(stop==='abort')await h.node('experiment-abort').onclick();
      if(stop==='fault')h.result(1,'faulted');
      if(stop==='disconnect'){h.state.connected=false;experimentControls();h.state.connected=true;}
      if(stop!=='start_failure'){h.result(1);h.release();}
      await tick();
      assert.equal(h.calls.filter(c=>typeof c==='object').length,stop==='start_failure'?0:1);
      if(stop==='abort')assert.equal(h.calls.at(-1),'ABORT');
    }finally{h.restore();}
  }
});
test('invalid frequency list cannot start a partial sweep',async()=>{
  const h=harness();
  try{
    h.node('experiment-frequencies').value='1, 10.001';await h.submit();
    assert.equal(h.calls.length,0);assert.match(h.node('experiment-state').textContent,/validation failed/);
  }finally{h.restore();}
});
test('a request during prevalidation delays the first run and ABORT can cancel validation',async()=>{
  const h=harness();
  try{
    h.setValidate(async()=>{h.state.session.pending=true;});
    await h.submit();assert.equal(h.calls.length,0);
    h.state.session.pending=false;experimentControls();await tick();assert.equal(h.calls.length,1);
    await h.node('experiment-abort').onclick();h.state.session.experimentOwned=false;
    let finish;
    h.setValidate(()=>new Promise(resolve=>{finish=resolve;}));
    const pending=h.submit();await tick();await tick();
    await h.node('experiment-abort').onclick();finish();await pending;
    assert.equal(h.calls.filter(c=>typeof c==='object').length,1);
  }finally{h.restore();}
});
