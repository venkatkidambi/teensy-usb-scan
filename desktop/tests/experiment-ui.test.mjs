import test from 'node:test';
import assert from 'node:assert/strict';
import { setupExperiments, experimentMessage, experimentControls, signalPoints, waveformExcursion } from '../ui/experiment.js';

test('waveform excursion catches experiment 7 spikes outside the reversal windows', () => {
  // Raw experiment-7.csv observations. Spikes are 41 ms before the 3.5 s
  // reversal and 44 ms after 4.5 s: neither is in its +/-40 ms window.
  const result={metadata:{elapsed_ms:5001,config:{kind:'sinusoidal',center_nm:2320188,
    amplitude_nm:10000,frequency_mhz:1000,cycles:5}},samples:[
    [3440,29826,4361599,8545,1,2,3,29865,60183,0,18388],
    [3460,29966,4483727,8545,1,2,3,29870,60383,0,18389],
    [3480,29862,4415913,8545,1,2,3,29873,60587,0,18390],
    [4540,29855,4374357,8545,16,2,18,29870,5686,0,18442],
    [4560,29953,4419806,8545,16,2,18,29865,5887,0,18443],
    [4580,29856,4389038,8545,16,2,18,29858,6086,0,18444],
  ]};
  const before=structuredClone(result);
  assert.deepEqual(waveformExcursion(result),{counts:92,receiptMs:3459,observations:6});
  assert.deepEqual(result,before); // no smoothing, clipping or timestamp rewriting
});

test('waveform excursion excludes pre/post-run readings and repeated publications', () => {
  const config={kind:'sinusoidal',center_nm:-780,amplitude_nm:117,frequency_mhz:1000,cycles:1};
  const sample=(ms,position,age,generation)=>[ms,position,0,0,age,0,0,0,0,0,generation];
  const result={metadata:{elapsed_ms:1200,config},samples:[
    sample(0,999,10,1), // pre-start
    sample(20,-12,0,2), // -897 nm rounds to -12; inside
    sample(40,-9,0,3), // -663 nm rounds to -9 (ties away from zero); inside
    sample(60,-15,1,4), // lower excursion
    sample(80,-15,21,4), // same publication
    sample(1000,-9,0,5),
    sample(1020,999,0,6), // after waveform STOP
  ]};
  assert.deepEqual(waveformExcursion(result),{counts:3,receiptMs:59,observations:4});
  result.metadata.elapsed_ms=40; // an aborted run
  assert.deepEqual(waveformExcursion(result),{counts:0,receiptMs:null,observations:2});
  result.samples=[];
  assert.equal(waveformExcursion(result),null);
  config.kind='quasi_static';
  assert.equal(waveformExcursion(result),null);
});

test('position uses receipt identity while pressure keeps every sample at elapsed time', () => {
  const samples = [
    [0, 10, 100, 0, 10, 3, 0, 20, 0, 0, 5, 9],
    [50, 10, 100, 0, 60, 2, 0, 21, 0, 0, 5, 10],
    [100, 10, 110, 0, 5, 1, 0, 22, 0, 0, 6, 10],
  ];
  assert.deepEqual(signalPoints(samples, 1), [[-10, 10], [95, 10]]);
  assert.deepEqual(signalPoints(samples, 2), [[0, 100], [50, 100], [100, 110]]);
});

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
    assert.match(node('experiment-motion-detail').textContent, /Starts from rest at center − amplitude/);
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
    node('tab-data').onclick();
    assert.equal(node('experiment-data-panel').hidden,false);
    assert.equal(node('experiment-data-empty').hidden,false);
    assert.equal(node('experiment-panel').hidden,true);
    assert.equal(node('dashboard-panel').hidden,true);
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-data-panel').hidden,true);
    assert.equal(node('experiment-panel').hidden,false);
    assert.equal(node('experiment-start').value,'374.556');
    assert.equal(node('experiment-increment').value,'5');
    const pending=submit();
    assert.match(node('experiment-state').textContent,/Sending START/);
    assert.equal(node('experiment-start-button').disabled,true);
    await submit();
    assert.deepEqual(calls,[{kind:'sinusoidal',start_um:374.556,increment_um:5,timing:1,repeats:5,compensation_enabled:false,compensation_ms:1}]);
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
        if (command === 'experiment_results') return Promise.resolve(null);
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
    assert.equal(node('experiment-data-panel').hidden, false);
    assert.equal(node('experiment-panel').hidden, true);
    assert.equal(node('experiment-data-empty').hidden, true);
    assert.equal(node('experiment-phase').hidden, true);
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-data-panel').hidden, true);
    experimentMessage({ type: 'experiment_dataset', dataset: {
      metadata: { run: 8, config: { kind: 'sinusoidal' }, outcome: 'completed',
        scan_gain: {before:40,applied:30,cap:30,saved:false},
        reason: 'none', count: 1, elapsed_ms: 1000, missed_samples: 0,
        max_lateness_ms: 0, stop_queued: true, reversals: [
          {direction:-1,max_error_counts:2,commands:[null,null,null,[1,0,250,251],[2,-1,250,253],[1,1,260,261]]},
          {direction:1,max_error_counts:3,commands:[null,null,null,[1,0,750,751],[2,1,750,753],[1,1,760,761]]},
        ] }, samples: [],
    } });
    assert.equal(node('experiment-data-panel').hidden, false);
    assert.equal(node('experiment-phase').hidden, false);
    assert.match(node('experiment-data-summary').textContent, /COMPLETED · Run 8 · sinusoidal/);
    assert.match(node('experiment-result-summary').textContent, /scanning gain PRO2 40 → 30 \(cap 30, not saved to controller flash\)/);
    assert.match(node('experiment-result-summary').textContent, /positive peak: 1 reversals, 10–10 ms zero→resume queued, max receipt-time error 0.156 µm/);
    assert.match(node('experiment-result-summary').textContent, /negative peak: 1 reversals, 10–10 ms zero→resume queued, max receipt-time error 0.234 µm/);
    assert.equal(node('experiment-save').disabled, false);
    const saving = node('experiment-save').onclick();
    assert.equal(node('experiment-save').disabled, true);
    assert.equal(node('experiment-start-button').disabled, true);
    await node('experiment-form').onsubmit({ preventDefault() {} });
    await node('experiment-save').onclick();
    assert.deepEqual(calls, ['experiment_results', 'experiment_save_csv']);
    resolve('/chosen folder/my run.csv');
    await saving;
    assert.match(node('experiment-state').textContent, /CSV saved to \/chosen folder\/my run.csv/);
    assert.match(node('experiment-data-status').textContent, /CSV saved to \/chosen folder\/my run.csv/);
    assert.equal(node('experiment-save').disabled, false);

    const cancelled = node('experiment-save').onclick();
    resolve(null);
    await cancelled;
    assert.match(node('experiment-state').textContent, /cancelled/);
    assert.match(node('experiment-data-status').textContent, /cancelled/);
    const failed = node('experiment-save').onclick();
    reject('Permission denied');
    await failed;
    assert.match(node('experiment-state').textContent, /CSV export failed: Permission denied/);
    assert.match(node('experiment-data-status').textContent, /CSV export failed: Permission denied/);
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

test('feed-forward controls support A/B runs, validated entry and retained settings across tabs', async () => {
  const nodes=new Map(), calls=[];
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{value:id==='experiment-compensation-ms'?'17':'1',checked:false,elements:[],setAttribute(){},reportValidity:()=>true,getBoundingClientRect:()=>({width:0})});
    return nodes.get(id);
  };
  const previousDocument=globalThis.document,previousObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};globalThis.ResizeObserver=class{observe(){}};
  try {
    const state={connected:true,hello:{experiments:true,sine_position:true,quasi_static_speed:true,compensation_max_ms:25},session:{async startExperiment(config){calls.push(config);}}};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls});
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-compensation-enabled').checked,false);
    assert.equal(node('experiment-compensation-value').hidden,true);
    assert.equal(node('experiment-compensation-ms').max,'25');
    const submit=()=>node('experiment-form').onsubmit({preventDefault(){}});
    await submit();
    assert.equal(calls[0].compensation_enabled,false);
    assert.equal(calls[0].compensation_ms,17);
    node('experiment-compensation-enabled').checked=true;
    node('experiment-compensation-enabled').onchange();
    assert.equal(node('experiment-compensation-value').hidden,false);
    assert.equal(node('experiment-compensation-ms').disabled,false);
    await submit();
    assert.deepEqual(calls[1],{...calls[0],compensation_enabled:true});
    node('experiment-compensation-ms').reportValidity=()=>false;
    node('experiment-compensation-ms').validationMessage='Out of range';
    await submit();assert.equal(calls.length,2);
    assert.match(node('experiment-state').textContent,/Compensation.*Out of range/);
    node('experiment-compensation-ms').reportValidity=()=>true;
    node('tab-quasi_static').onclick();
    assert.equal(node('experiment-compensation-fields').hidden,true);
    await submit();assert.equal(calls[2].compensation_enabled,undefined);
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-compensation-enabled').checked,true);
    assert.equal(node('experiment-compensation-ms').value,'17');
    state.session.experimentOwned=true;experimentControls();
    assert.equal(node('experiment-compensation-ms').disabled,true);
  } finally {globalThis.document=previousDocument;globalThis.ResizeObserver=previousObserver;}
});
