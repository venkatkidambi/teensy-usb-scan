import test from 'node:test';
import assert from 'node:assert/strict';
import {plotPng} from '../ui/plot-export.js';
import {setupExperiments,experimentControls,experimentMessage} from '../ui/experiment.js';

function harness() {
  const previous={document:globalThis.document,ResizeObserver:globalThis.ResizeObserver,getComputedStyle:globalThis.getComputedStyle};
  const nodes=new Map(),draws=[],exports=[];
  const bytes=[137,80,78,71,13,10,26,10];
  const rect={left:10,top:20,width:400,height:320};
  const source={width:760,getBoundingClientRect:()=>({left:20,top:90,width:380,height:240})};
  const card={getBoundingClientRect:()=>rect,querySelector:()=>source,querySelectorAll:()=>[]};
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{value:'ms',elements:[],setAttribute(){},getBoundingClientRect:()=>({width:0})});
    return nodes.get(id);
  };
  for(const name of ['position','pressure','sweep-phase'])nodes.set(`experiment-${name}-card`,card);
  globalThis.document={getElementById:node,createElement:()=>{
    const canvas={getContext:()=>({scale(){},fillRect(){},strokeRect(){},drawImage(...args){draws.push(args);}}),
      toBlob:callback=>callback({arrayBuffer:async()=>new Uint8Array(bytes).buffer})};
    exports.push(canvas);return canvas;
  }};
  globalThis.ResizeObserver=class{observe(){}};
  globalThis.getComputedStyle=()=>({backgroundColor:'#fff',borderColor:'#e0e3e0'});
  return {node,card,rect,source,bytes,draws,exports,restore(){Object.assign(globalThis,previous);}};
}

test('PNG copies the visible plot bitmap at its displayed size and device pixel ratio',async()=>{
  const h=harness();
  try{
    assert.deepEqual(await plotPng(h.card),h.bytes);
    assert.equal(h.exports[0].width,800);
    assert.equal(h.exports[0].height,640);
    assert.deepEqual(h.draws,[[h.source,10,70,380,240]]);
    h.rect.width=0;
    await assert.rejects(plotPng(h.card),/visible plot/);
  }finally{h.restore();}
});

test('PNG export uses native Save As, locks selection and exports, and recovers after cancellation or failure',async()=>{
  const h=harness();
  try{
    const state={connected:true,hello:{experiments:true,force_tare:true,quasi_static_cycles:true,quasi_static_cycle_wait:true,sine_serial_displacement:true,sine_serial_velocity:true},session:{}};
    const calls=[];let resolve,reject;
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls,
      invoke(command,args){
        if(command==='experiment_results')return Promise.resolve(null);
        calls.push([command,args]);return new Promise((ok,fail)=>{resolve=ok;reject=fail;});
      }});
    await Promise.resolve();
    assert.equal(h.node('experiment-position-png').disabled,true);
    experimentMessage({type:'experiment_dataset',dataset:{archive_index:0,
      metadata:{run:7,config:{kind:'sinusoidal',frequency_mhz:1000,cycles:3},outcome:'completed',reason:'none',count:0,elapsed_ms:0},samples:[]}});
    for(const name of ['position','pressure','sweep-phase']){
      assert.equal(h.node(`experiment-${name}-png`).disabled,false);
      const pending=h.node(`experiment-${name}-png`).onclick();
      assert.equal(h.node('experiment-save').disabled,true);
      assert.equal(h.node('experiment-run').disabled,true);
      assert.equal(h.node('experiment-phase-unit').disabled,true);
      assert.equal(h.node('experiment-start-button').disabled,true);
      await h.node('experiment-pressure-png').onclick();
      await h.node('experiment-save').onclick();
      await new Promise(resolve=>setImmediate(resolve));
      assert.deepEqual(calls.at(-1),['experiment_save_png',{index:0,plot:name.replaceAll('-','_'),png:h.bytes}]);
      if(name==='position')resolve('/chosen folder/position.png');
      if(name==='pressure')resolve(null);
      if(name==='sweep-phase')reject('Permission denied');
      await pending;
      assert.match(h.node('experiment-data-status').textContent,name==='position'?/PNG saved to.*position.png/:name==='pressure'?/cancelled/:/PNG export failed: Permission denied/);
      assert.equal(h.node('experiment-save').disabled,false);
      assert.equal(h.node('experiment-run').disabled,false);
      assert.equal(h.node(`experiment-${name}-png`).disabled,false);
    }
    assert.equal(calls.length,3);
    state.experimentUnresolved=true;experimentControls();
    await h.node('experiment-position-png').onclick();
    assert.equal(h.node('experiment-position-png').disabled,true);
    assert.equal(calls.length,3);
  }finally{h.restore();}
});
