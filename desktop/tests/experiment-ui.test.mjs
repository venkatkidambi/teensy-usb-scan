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
    const state={connected:true,hello:{experiments:true,force_tare:true,quasi_static_cycles:true,quasi_static_cycle_wait:true,sine_serial_displacement:true,sine_serial_velocity:true},session:{
      startExperiment(config){calls.push(config);return new Promise((_,fail)=>{reject=fail;});},
    }};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls,activity:message=>logs.push(message)});
    node('tab-sinusoidal').onclick();
    assert.match(node('experiment-motion-detail').textContent, /start the sine at center − amplitude/);
    const submit=()=>node('experiment-form').onsubmit({preventDefault(){}});
    for(const [object,key,value,expected] of [
      [state,'connected',false,/Connect the Teensy/],
      [state,'experimentUnresolved',true,/Reconnect/],
      [state.hello,'experiments',false,/enable experiments/],
      [state.hello,'force_tare',false,/automatic force zeroing/],
      [state.hello,'sine_serial_displacement',false,/sine control/],
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
    node('tab-sinusoidal').onclick();
    assert.equal(node('experiment-data-panel').hidden,true);
    assert.equal(node('experiment-panel').hidden,false);
    assert.equal(node('experiment-start').value,'374.556');
    assert.equal(node('experiment-increment').value,'5');
    const pending=submit();
    assert.match(node('experiment-state').textContent,/Sending START/);
    assert.equal(node('experiment-start-button').disabled,true);
    await submit();
    assert.deepEqual(calls,[{kind:'sinusoidal',start_um:374.556,increment_um:5,timing:1,repeats:5,sine_control:'displacement'}]);
    reject(Error('Controller is not ready'));await pending;
    assert.equal(node('experiment-state').textContent,'START failed: Controller is not ready');
    assert.equal(logs.at(-1),'START failed: Controller is not ready');
    assert.equal(node('experiment-start-button').disabled,false);
    node('experiment-control').value='velocity';
    node('experiment-control').onchange();
    assert.match(node('experiment-motion-detail').textContent,/position can drift/);
    state.hello.sine_serial_velocity=false;experimentControls();
    assert.equal(node('experiment-start-button').disabled,true);
    state.hello.sine_serial_velocity=true;experimentControls();
    let selected;
    state.session.startExperiment=config=>{selected=config;throw Error('Bridge unavailable');};
    await submit();
    assert.equal(selected.sine_control,'velocity');
    assert.equal(node('experiment-state').textContent,'START failed: Bridge unavailable');
    assert.equal(node('experiment-start-button').disabled,false);
    node('experiment-control').value='gpio';
    experimentControls();
    assert.equal(node('experiment-start-button').disabled,true);
    state.hello.sine_gpio=true;experimentControls();
    assert.equal(node('experiment-start-button').disabled,false);
    assert.match(node('experiment-motion-detail').textContent,/GPIO Pulse\/Direction/);
    await submit();
    assert.equal(selected.sine_control,'gpio');
    node('experiment-control').value='velocity_position';
    node('experiment-control').onchange();
    assert.match(node('experiment-motion-detail').textContent,/gradual encoder position correction/);
    assert.equal(node('experiment-start-button').disabled,true);
    await submit();
    assert.equal(selected.sine_control,'gpio');
    state.hello.sine_serial_velocity_position=true;experimentControls();
    assert.equal(node('experiment-start-button').disabled,false);
    await submit();
    assert.equal(selected.sine_control,'velocity_position');
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
    node('experiment-cycle-wait').value='12.5';
    node('experiment-predelay').value='2.5';node('experiment-cycles').value='5';node('experiment-return').checked=true;
    node('experiment-form').oninput();
    assert.match(node('experiment-planner-summary').textContent,/5 cycles/);
    state.hello.quasi_static_cycles=false;experimentControls();
    assert.match(node('experiment-start-reason').textContent,/predelay, cycles and return/);
    state.hello.quasi_static_cycles=true;experimentControls();
    node('tab-sinusoidal').onclick();assert.equal(node('experiment-quasi-fields').hidden,true);
    node('tab-quasi_static').onclick();
    await submit();
    assert.equal(calls.at(-1).cycle_wait_s,12.5);
    assert.equal(calls.at(-1).predelay_s,2.5);
    assert.equal(calls.at(-1).cycles,5);
    assert.equal(calls.at(-1).return_to_start,true);
    assert.equal(calls.at(-1).speed_um_s,25);
    assert.equal(calls.at(-1).kind,'quasi_static');
    const motion=structuredClone(calls.at(-1));
    node('planner-zen-enabled').checked=true;node('planner-zen-enabled').oninput();
    assert.match(node('planner-zen-summary').textContent,/40 s acquiring \+ 20 s waiting = 60 s/);
    node('planner-zen-interval').value='20';node('planner-zen-interval').oninput();
    assert.match(node('planner-zen-summary').textContent,/must be at least/);
    await submit();assert.deepEqual(calls.at(-1),motion); // Invalid visualization cannot change or block START.
    assert.equal(node('experiment-zen-enabled').checked,false);
    state.hello.quasi_static_cycle_wait=false;experimentControls();
    assert.match(node('experiment-start-reason').textContent,/waiting between cycles/);
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
    const state = { connected: true, hello: { experiments: true, force_tare: true, quasi_static_cycles: true, quasi_static_cycle_wait: true }, session: {} };
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
    const samples=Array.from({length:101},(_,i)=>{
      const ms=i*20,angle=t=>2*Math.PI*t/1000;
      return [ms,500-19*Math.cos(angle(ms-3-7)),100000-40*Math.cos(angle(ms)),0,3,0,0,-20*Math.cos(angle(ms)),0,0,i,i];
    });
    experimentMessage({ type: 'experiment_dataset', dataset: {
      metadata: { run: 8, config: { kind: 'sinusoidal', frequency_mhz: 1000, cycles: 2 }, outcome: 'completed',
        reason: 'none', count: samples.length, elapsed_ms: 2020, missed_samples: 0,
        max_lateness_ms: 0, stop_queued: true }, samples,
    } });
    assert.equal(node('experiment-amplitude').textContent,'1.482 / 1.560 µm · 95.00% achieved · error -0.078 µm (-5.00%) · fit R² 1.000');
    experimentMessage({ type: 'experiment_dataset', dataset: {
      metadata: { run: 8, config: { kind: 'sinusoidal' }, outcome: 'completed',
        reason: 'none', count: 1, elapsed_ms: 1000, missed_samples: 0,
        max_lateness_ms: 0, stop_queued: true }, samples: [],
    } });
    assert.equal(node('experiment-data-panel').hidden, false);
    assert.equal(node('experiment-phase').hidden, false);
    assert.match(node('experiment-amplitude').textContent,/^Unavailable: Frequency or cycle metadata/);
    assert.match(node('experiment-data-summary').textContent, /COMPLETED · Run 8 · sinusoidal/);
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

test('calculator previews without changing a run, applies on request and locks while hardware is owned',async()=>{
  const nodes=new Map();
  const node=id=>{if(!nodes.has(id))nodes.set(id,{value:'1',elements:[],setAttribute(){},reportValidity:()=>true,getBoundingClientRect:()=>({width:0})});return nodes.get(id);};
  const previousDocument=globalThis.document,previousObserver=globalThis.ResizeObserver;
  globalThis.document={getElementById:node};globalThis.ResizeObserver=class {observe(){}};
  try{
    const sent=[];
    const state={connected:true,hello:{experiments:true,force_tare:true,quasi_static_speed:true,quasi_static_cycles:true,quasi_static_cycle_wait:true},session:{startExperiment:async config=>sent.push(config)}};
    setupExperiments({state:()=>state,positionAge:()=>Infinity,controls:experimentControls});
    node('tab-quasi_static').onclick();
    for(const [key,value] of Object.entries({start:'0',increment:'1',timing:'1',repeats:'5',speed:'10',cycles:'3',predelay:'0','cycle-wait':'0'}))node(`experiment-${key}`).value=value;
    node('calculator-stack').value='10';node('calculator-buffer').value='0.5';
    const suggestion=node('calculator-calculate').onclick();
    assert.ok(suggestion);assert.equal(node('calculator-results').hidden,false);
    for(const key of ['interval','predelay','hold','wait','offset'])assert.match(node(`calculator-${key}-result`).textContent,/^\d+ s$/);
    assert.match(node('calculator-status').textContent,/18 planned acquisitions/);
    assert.equal(node('calculator-zen-interval').textContent,`${suggestion.interval_s} s`);
    assert.equal(node('calculator-zen-count').textContent,'18');
    assert.match(node('calculator-status').textContent,/assumes actuator already/);
    assert.equal(node('experiment-timing').value,'1');assert.equal(node('planner-zen-enabled').checked,false);
    assert.equal(sent.length,0);
    state.session.experimentOwned=true;experimentControls();
    assert.equal(node('calculator-apply').disabled,true);assert.equal(node('calculator-calculate').disabled,true);
    node('calculator-apply').onclick();assert.equal(node('experiment-timing').value,'1');
    state.session.experimentOwned=false;experimentControls();
    node('calculator-buffer').value='0';node('calculator-buffer').oninput();
    assert.equal(node('calculator-results').hidden,true);assert.equal(node('calculator-apply').disabled,true);
    node('calculator-calculate').onclick();assert.match(node('calculator-status').textContent,/Could not calculate/);
    node('calculator-buffer').value='0.5';node('calculator-calculate').onclick();
    node('calculator-apply').onclick();
    assert.equal(node('experiment-timing').value,String(suggestion.config.timing));
    assert.equal(node('experiment-predelay').value,String(suggestion.config.predelay_s));
    assert.equal(node('experiment-cycle-wait').value,String(suggestion.config.cycle_wait_s));
    assert.equal(node('planner-zen-interval').value,String(suggestion.interval_s));
    assert.equal(node('planner-zen-offset').value,String(suggestion.start_s));
    assert.equal(node('planner-zen-enabled').checked,true);
    assert.equal(node('experiment-zen-enabled').checked,false);
    assert.equal(sent.length,0);
    assert.match(node('calculator-status').textContent,/Suggested timings applied/);
    assert.equal(node('planner-zen-interval-setting').textContent,`${suggestion.interval_s} s`);
    assert.equal(node('planner-zen-count-setting').textContent,'18');
    await node('experiment-form').onsubmit({preventDefault(){}});
    assert.equal(sent.length,1);
    assert.deepEqual(sent[0],{kind:'quasi_static',start_um:0,increment_um:1,timing:suggestion.config.timing,repeats:5,speed_um_s:10,cycles:3,predelay_s:suggestion.config.predelay_s,cycle_wait_s:suggestion.config.cycle_wait_s,return_to_start:false});
    node('calculator-calculate').onclick();node('experiment-increment').value='2';node('experiment-form').oninput();
    assert.equal(node('calculator-apply').disabled,true);assert.match(node('calculator-status').textContent,/settings changed/);
    // Full run: 2 s predelay + 5,000 × 2 s steps/holds + 4,999 × 3 s returns/waits.
    for(const [key,value] of Object.entries({start:'0',increment:'39',timing:'1',repeats:'1',speed:'39',cycles:'5000',predelay:'2','cycle-wait':'2'}))node(`experiment-${key}`).value=value;
    node('planner-zen-enabled').checked=false;
    node('planner-zen-stack').value='40';node('planner-zen-interval').value='60';node('planner-zen-offset').value='5';
    node('experiment-form').oninput();
    assert.match(node('experiment-planner-summary').textContent,/preview shows only the beginning/);
    assert.equal(node('planner-zen-interval-setting').textContent,'60 s');
    assert.equal(node('planner-zen-count-setting').textContent,'417');
    node('planner-zen-stack').value='20';node('planner-zen-interval').value='30';node('planner-zen-interval').oninput();
    assert.equal(node('planner-zen-count-setting').textContent,'834');
    node('planner-zen-offset').value='24999';node('planner-zen-offset').oninput();
    assert.equal(node('planner-zen-count-setting').textContent,'0');
    node('planner-zen-stack').value='40';node('planner-zen-stack').oninput();
    assert.equal(node('planner-zen-count-setting').textContent,'—');
    assert.equal(node('planner-zen-interval-setting').textContent,'—');
    assert.equal(sent.length,1); // Displaying settings never sends commands.

  }finally{globalThis.document=previousDocument;globalThis.ResizeObserver=previousObserver;}
});
