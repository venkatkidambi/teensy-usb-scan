import test from 'node:test';
import assert from 'node:assert/strict';
import { setupExperiments, experimentMessage, experimentControls } from '../ui/experiment.js';

test('START explains blockers, validates fields, submits once, and reports failures', async () => {
  const nodes=new Map();
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{value:'1',elements:[],setAttribute(){},
      reportValidity:()=>true,getBoundingClientRect:()=>({width:0})});
    return nodes.get(id);
  };
  const previousDocument=globalThis.document,previousObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};
  globalThis.ResizeObserver=class {observe(){}};
  try {
    const calls=[],logs=[];let reject;
    const state={connected:true,hello:{experiments:true,sine_velocity:true},session:{
      startExperiment(config){calls.push(config);return new Promise((_,fail)=>{reject=fail;});},
    }};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls,activity:message=>logs.push(message)});
    node('tab-sinusoidal').onclick();
    const submit=()=>node('experiment-form').onsubmit({preventDefault(){}});
    for(const [object,key,value,expected] of [
      [state,'connected',false,/Connect the Teensy/],
      [state,'experimentUnresolved',true,/Reconnect/],
      [state.hello,'experiments',false,/enable experiments/],
      [state.hello,'sine_velocity',false,/sinusoidal speed control/],
      [state.session,'pending',true,/current Teensy request/],
      [state.session,'experimentOwned',true,/owns the hardware/],
    ]){
      const old=object[key];object[key]=value;experimentControls();
      assert.equal(node('experiment-start-button').disabled,true);
      assert.match(node('experiment-start-reason').textContent,expected);
      await submit();assert.match(node('experiment-state').textContent,expected);
      object[key]=old;
    }
    assert.equal(calls.length,0);
    experimentControls();
    assert.equal(node('experiment-start-button').disabled,false);
    assert.equal(node('experiment-start-reason').hidden,true);
    node('experiment-repeats').reportValidity=()=>false;
    node('experiment-repeats').validationMessage='Please enter a whole number.';
    await submit();
    assert.match(node('experiment-state').textContent,/Number of cycles: Please enter a whole number/);
    assert.equal(calls.length,0);
    node('experiment-repeats').reportValidity=()=>true;
    for(const [key,value] of Object.entries({start:'374.556',increment:'5',timing:'1',repeats:'5'}))node(`experiment-${key}`).value=value;
    const pending=submit();
    assert.match(node('experiment-state').textContent,/Sending START/);
    assert.equal(node('experiment-start-button').disabled,true);
    await submit();
    assert.deepEqual(calls,[{kind:'sinusoidal',start_um:374.556,increment_um:5,timing:1,repeats:5}]);
    reject(Error('Controller is not ready'));await pending;
    assert.equal(node('experiment-state').textContent,'START failed: Controller is not ready');
    assert.equal(logs.at(-1),'START failed: Controller is not ready');
    assert.equal(node('experiment-start-button').disabled,false);
    state.session.startExperiment=()=>{throw Error('Bridge unavailable');};
    await submit();
    assert.equal(node('experiment-state').textContent,'START failed: Bridge unavailable');
    assert.equal(node('experiment-start-button').disabled,false);
  } finally {globalThis.document=previousDocument;globalThis.ResizeObserver=previousObserver;}
});

test('Save CSV opens native Save As, prevents duplicate requests, and retains results', async () => {
  const nodes = new Map();
  const node = id => {
    if (!nodes.has(id)) nodes.set(id, {
      value: '1', elements: [], setAttribute() {},
      getBoundingClientRect: () => ({ width: 0 }),
    });
    return nodes.get(id);
  };
  const previousDocument = globalThis.document;
  const previousObserver = globalThis.ResizeObserver;
  globalThis.document = { getElementById: node };
  globalThis.ResizeObserver = class { observe() {} };
  try {
    const state = { connected: true, hello: { experiments: true }, session: {} };
    const calls = [];
    let resolve, reject;
    setupExperiments({
      state: () => state,
      positionAge: () => Infinity,
      controls: experimentControls,
      invoke(command) {
        calls.push(command);
        if (command === 'experiment_result') return Promise.resolve(null);
        assert.equal(command, 'experiment_save_csv');
        return new Promise((ok, fail) => { resolve = ok; reject = fail; });
      },
    });
    assert.equal(node('experiment-save').disabled, true);
    experimentMessage({ type: 'experiment_dataset', dataset: {
      metadata: { run: 7, config: { kind: 'quasi_static' }, outcome: 'completed',
        reason: 'complete', count: 1, elapsed_ms: 50, missed_samples: 0,
        max_lateness_ms: 0, stop_queued: true },
      samples: [[0, 1, 2, 0, 0, 0, 0, 1, 0, 0]],
    } });
    assert.equal(node('experiment-save').disabled, false);
    const saving = node('experiment-save').onclick();
    assert.equal(node('experiment-save').disabled, true);
    assert.equal(node('experiment-start-button').disabled, true);
    await node('experiment-form').onsubmit({ preventDefault() {} });
    await node('experiment-save').onclick();
    assert.deepEqual(calls, ['experiment_result', 'experiment_save_csv']);
    resolve('/chosen folder/my run.csv');
    await saving;
    assert.match(node('experiment-state').textContent, /CSV saved to \/chosen folder\/my run.csv/);
    assert.equal(node('experiment-save').disabled, false);

    const cancelled = node('experiment-save').onclick();
    resolve(null);
    await cancelled;
    assert.match(node('experiment-state').textContent, /cancelled/);
    const failed = node('experiment-save').onclick();
    reject('Permission denied');
    await failed;
    assert.match(node('experiment-state').textContent, /CSV export failed: Permission denied/);
    assert.equal(node('experiment-save').disabled, false);
    assert.equal(node('experiment-results').hidden, false);

    state.experimentUnresolved = true;
    experimentControls();
    assert.equal(node('experiment-save').disabled, true);
    const count = calls.length;
    await node('experiment-save').onclick();
    assert.equal(calls.length, count);
  } finally {
    globalThis.document = previousDocument;
    globalThis.ResizeObserver = previousObserver;
  }
});

test('velocity sine requires firmware capability and reports incomplete restoration', () => {
  const nodes = new Map();
  const node = id => {
    if (!nodes.has(id)) nodes.set(id, { value:'1', elements:[], setAttribute(){}, getBoundingClientRect:()=>({width:0}) });
    return nodes.get(id);
  };
  const previousDocument=globalThis.document,previousObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};
  globalThis.ResizeObserver=class {observe(){}};
  try {
    const state={connected:true,hello:{experiments:true},session:{}};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls});
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-start-button').disabled,true);
    assert.match(node('experiment-motion-detail').textContent,/requires firmware/);
    state.hello.sine_velocity=true;
    experimentControls();
    assert.equal(node('experiment-start-button').disabled,false);
    experimentMessage({type:'experiment_dataset',dataset:{
      metadata:{run:1,config:{kind:'sinusoidal'},outcome:'aborted',reason:'PC ABORT',count:0,
        elapsed_ms:10,control:'sspd_scan_p',control_updates:2,max_control_lateness_ms:1,
        missed_samples:0,max_lateness_ms:0,stop_queued:true,settings_restore_sent:false,
        settings_restore_error:'Restore blocked: controller still scanning'},samples:[],
    }});
    assert.match(node('experiment-result-summary').textContent,/100 Hz speed control/);
    assert.match(node('experiment-result-summary').textContent,/restoration incomplete/);
    assert.match(node('experiment-result-summary').textContent,/Cleanup: Restore blocked: controller still scanning/);
    experimentMessage({type:'experiment_dataset',dataset:{
      metadata:{run:2,config:{kind:'sinusoidal'},outcome:'faulted',reason:'Scan tracking error exceeded guard',count:2,
        elapsed_ms:60,control:'sspd_scan_p',control_updates:6,max_control_lateness_ms:0,
        missed_samples:0,max_lateness_ms:0,stop_queued:true,settings_restore_sent:true,
        scan_fault:{elapsed_ms:60,epos_um:0,reference_um:1.841,error_um:1.841,guard_um:1.56,
          low_counts:-84,high_counts:84,epos_age_ms:0}},samples:[],
    }});
    assert.match(node('experiment-result-summary').textContent,/Fault at 60 ms: position 0.000 µm, reference 1.841 µm/);
    assert.match(node('experiment-result-summary').textContent,/allowed ±1.560 µm/);
    assert.match(node('experiment-result-summary').textContent,/travel -6.552 to 6.552 µm/);
  } finally {globalThis.document=previousDocument;globalThis.ResizeObserver=previousObserver;}
});
