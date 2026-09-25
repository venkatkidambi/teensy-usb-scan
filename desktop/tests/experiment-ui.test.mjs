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
    const state={connected:true,hello:{experiments:true,sine_position:true},session:{
      startExperiment(config){calls.push(config);return new Promise((_,fail)=>{reject=fail;});},
    }};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls,activity:message=>logs.push(message)});
    node('tab-sinusoidal').onclick();
    const submit=()=>node('experiment-form').onsubmit({preventDefault(){}});
    for(const [object,key,value,expected] of [
      [state,'connected',false,/Connect the Teensy/],
      [state,'experimentUnresolved',true,/Reconnect/],
      [state.hello,'experiments',false,/enable experiments/],
      [state.hello,'sine_position',false,/sinusoidal position control/],
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
    node('tab-quasi_static').onclick();
    await submit();
    assert.match(node('experiment-state').textContent,/firmware.*quasi-static speed control/);
    state.hello.quasi_static_speed=true;
    experimentControls();
    assert.equal(node('experiment-speed-field').hidden,false);
    assert.equal(node('experiment-speed').disabled,false);
    assert.equal(node('experiment-speed').value,'10');
    node('experiment-speed').reportValidity=()=>false;
    node('experiment-speed').validationMessage='Enter a positive whole number.';
    node('experiment-speed-label').textContent='Actuator speed (µm/s)';
    await submit();
    assert.match(node('experiment-state').textContent,/Actuator speed.*positive whole number/);
    node('experiment-speed').reportValidity=()=>true;
    node('experiment-speed').value='25';
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-speed-field').hidden,true);
    assert.equal(node('experiment-speed').disabled,true);
    node('tab-quasi_static').onclick();
    assert.equal(node('experiment-speed').value,'25');
    state.session.startExperiment=async config=>calls.push(config);
    await submit();
    assert.equal(calls.at(-1).speed_um_s,25);
    assert.equal(calls.at(-1).kind,'quasi_static');
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
