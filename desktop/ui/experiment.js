import {quasiStaticPlan,zenAcquisitions,calculateExperiment,drawPlan} from './planner.js';
import {phaseDelays,parseFrequencies} from './phase.js';
import {plotPng} from './plot-export.js';
import {forceMillinewtons,forceRangeLabel} from './force.js';
// UI only: native Rust validates configuration, receives binary data and builds CSV.
const $=id=>document.getElementById(id);
const text=(id,value)=>{const node=$(id);if(node.textContent!==value)node.textContent=value;};
let mode='dashboard',view='dashboard',showResults=false,dataset=null,context=null,saving=false,starting=false;
let sweep=null,history=[],selectedIndex=null,sweepTimer=null,historyLoading=false,validationCancelled=false;
let phaseCache=new WeakMap(),sweepPhases=[];
let zenWait=null,zenTimer=null,zenChoosing=false;
let calculator=null;
const plotNames=['position','pressure','sweep-phase'];
const modes=['dashboard','quasi_static','sinusoidal','data'];
const fields=['start','increment','timing','repeats','speed'];
const configurations={quasi_static:['0','1','1','3','10'],sinusoidal:['0','1','1','3','10']};
function startBlocked({session,connected,hello,experimentUnresolved},ignoreZen=false) {
  if(historyLoading)return 'Loading recorded experiments…';
  if(zenWait&&!ignoreZen)return 'Waiting for a new ZEN .czi image. Press ABORT to cancel.';
  if(starting)return 'Sending START to the Teensy…';
  if(sweep)return 'Frequency sweep in progress. Press ABORT to stop it.';
  if(saving)return 'Wait for export to finish.';
  if(!connected||!session)return 'Connect the Teensy to start an experiment.';
  if(session.experimentOwned)return 'An experiment owns the hardware. Wait for its results or press ABORT.';
  if(experimentUnresolved)return 'Reconnect to recover the previous experiment before starting another.';
  if(!hello?.experiments)return 'Update the Teensy firmware to enable experiments.';
  if(!hello?.force_tare)return 'Update the Teensy firmware to enable automatic force zeroing.';
  if(mode==='quasi_static'&&!hello?.quasi_static_speed)return 'Update the Teensy firmware to enable quasi-static speed control.';
  if(mode==='quasi_static'&&!hello?.quasi_static_cycles)return 'Update the Teensy firmware to enable quasi-static predelay, cycles and return.';
  if(mode==='quasi_static'&&!hello?.quasi_static_cycle_wait)return 'Update the Teensy firmware to enable waiting between cycles.';
  if(mode==='sinusoidal'&&!hello?.[$('experiment-control').value==='gpio'?'sine_gpio':`sine_serial_${$('experiment-control').value||'displacement'}`])return 'Update the Teensy firmware to enable the selected sine control mode.';
  if(session.pending)return 'Waiting for the current Teensy request to finish…';
  return '';
}
function startStatus(message) {
  $('experiment-state').textContent=message;
  context.activity?.(message);
}
function cancelZen(message) {
  zenWait=null;clearTimeout(zenTimer);zenTimer=null;
  if(message)startStatus(message);
}
function scheduleZen(current) {
  if(zenWait===current)zenTimer=setTimeout(()=>{zenTimer=null;void pollZen(current);},100);
}
async function pollZen(current) {
  try {
    if(zenWait!==current)return;
    if(!current.file){
      const files=await context.invoke('zeiss_czi_files',{folder:current.folder});
      if(zenWait!==current)return;
      current.file=files.find(file=>!current.files.has(file));
      current.files=new Set(files);
    }
    const state=context.state();
    if(!state.connected||state.session!==current.session)throw Error('Connection changed. Arm again after reconnecting.');
    // Telemetry may have a request in flight. Retain the trigger until it finishes.
    const blocked=startBlocked(state,true);
    if(blocked&&!state.session.pending)throw Error(blocked);
    if(!current.file||blocked){scheduleZen(current);return;}
    cancelZen();
    startStatus(`ZEN image detected: ${current.file}. Sending START to the Teensy…`);
    if(current.sweeping){
      sweep={configs:current.configs,index:0,completed:0,session:current.session,received:false,run:null};
      sweepProgress('ZEN image detected. Starting sweep…');
      await nextSweepRun();
    }else{
      starting=true;experimentControls();
      try{await current.session.startExperiment(current.configs[0]);}
      catch(e){startStatus(`START failed: ${e.message??e}`);}
      finally{starting=false;}
    }
  }catch(e){
    if(zenWait===current)cancelZen(`ZEN wait stopped: ${e.message??e}`);
  }finally{context.controls();}
}
async function armZen(config,state) {
  const folder=$('experiment-zen-folder').value.trim();
  if(!folder){startStatus('Choose the folder where ZEN saves .czi images.');return;}
  const current={folder,session:state.session,sweeping:mode==='sinusoidal'&&$('experiment-sweep-enabled').checked};
  zenWait=current;startStatus('Checking ZEN folder and experiment settings…');experimentControls();
  try {
    current.configs=current.sweeping?parseFrequencies($('experiment-frequencies').value).map(timing=>({...config,timing})):[config];
    await context.invoke('experiment_validate',{configs:current.configs});
    if(zenWait!==current)return;
    const files=await context.invoke('zeiss_czi_files',{folder});
    if(zenWait!==current)return;
    if(!context.state().connected||context.state().session!==current.session)throw Error('Connection changed. Arm again after reconnecting.');
    current.files=new Set(files);
    startStatus(`Waiting for a new ZEN .czi image in ${folder}. Press ABORT to cancel.`);
    scheduleZen(current);
  }catch(e){if(zenWait===current)cancelZen(`Could not arm ZEN link: ${e.message??e}`);}
  finally{context.controls();}
}
function sweepProgress(message,state='active') {
  const {completed,configs}=sweep;
  $('experiment-sweep-progress').setAttribute('data-state',state);
  $('experiment-sweep-bar').max=configs.length;
  $('experiment-sweep-bar').value=completed;
  text('experiment-sweep-count',`${completed} of ${configs.length} completed · ${Math.round(completed/configs.length*100)}%`);
  text('experiment-sweep-status',message);
}
export function experimentControls() {
  if(!context)return;
  const {session,connected,hello,snapshot,experimentUnresolved}=context.state();
  if(zenWait&&(!connected||session!==zenWait.session||session?.experimentOwned||experimentUnresolved)){
    cancelZen('ZEN wait cancelled: connection or experiment ownership changed. Arm again when ready.');
  }
  if(sweep&&(!connected||session!==sweep.session)){
    sweepProgress('Sweep stopped after disconnect. Recover the current run before starting another sweep.','stopped');sweep=null;
  }
  const owned=!!zenWait||!!sweep||starting||!!session?.experimentOwned||experimentUnresolved;
  for(const name of modes) $(`tab-${name}`).disabled=owned;
  $('dashboard-panel').inert=owned;
  for(const control of $('experiment-form').elements)control.disabled=owned;
  $('calculator-calculate').disabled=owned;
  $('calculator-apply').disabled=owned||!calculator;
  for(const key of ['stack','buffer'])$(`calculator-${key}`).disabled=owned;
  $('experiment-speed').disabled=owned||mode!=='quasi_static';
  $('experiment-quasi-fields').hidden=mode!=='quasi_static';
  $('experiment-planner').hidden=mode!=='quasi_static';
  for(const key of ['predelay','cycle-wait','cycles','return'])$(`experiment-${key}`).disabled=owned||mode!=='quasi_static';
  $('experiment-control-field').hidden=mode!=='sinusoidal';
  $('experiment-control').disabled=owned||mode!=='sinusoidal';
  const sweeping=mode==='sinusoidal'&&$('experiment-sweep-enabled').checked;
  $('experiment-sweep-progress').hidden=!sweeping||!$('experiment-sweep-status').textContent;
  $('experiment-frequencies').disabled=owned||!sweeping;
  $('experiment-timing').disabled=owned||sweeping;
  $('experiment-sweep-fields').hidden=mode!=='sinusoidal';
  const zenEnabled=$('experiment-zen-enabled').checked;
  $('experiment-zen-options').hidden=!zenEnabled;
  $('experiment-zen-folder').disabled=owned||!zenEnabled;
  $('experiment-zen-browse').disabled=owned||!zenEnabled||zenChoosing;
  text('experiment-start-button',zenEnabled?'Wait for ZEN image':'Start experiment');
  $('experiment-run').disabled=!!saving||owned||!history.length;
  $('experiment-phase-unit').disabled=!!saving;
  const blocked=startBlocked(context.state());
  $('experiment-start-button').disabled=!!blocked;
  $('experiment-start-button').title=blocked;
  text('experiment-start-reason',blocked);
  $('experiment-start-reason').hidden=!blocked;
  text('experiment-motion-detail',mode==='sinusoidal'?($('experiment-control').value==='gpio'?`GPIO Pulse/Direction controls the approach and x = center − A·cos(2πft) on the Teensy. Starts and finishes at center − amplitude; waits for encoder confirmation. Peak pulse limit: ${hello?.max_pulse_hz??10000} Hz.`:`${$('experiment-control').value==='velocity_position'?'Serial velocity control adds a bounded, gradual encoder position correction to reduce drift.':$('experiment-control').value==='velocity'?'Serial velocity control varies scan speed in 1 µm/s steps; position can drift.':'Serial displacement control uses small position targets and a varying speed limit to reduce jerking.'} All modes use a smooth approach ramp and start the sine at center − amplitude. They finish at the last cycle endpoint without returning to center. No GPIO wiring required. Lower frequencies allow more serial updates per cycle.`):'Speed applies to start positioning and every step. The controller confirms the speed before motion; each hold begins after confirmed arrival.');
  $('experiment-current').disabled=owned||!connected||!snapshot?.values.EPOS||context.positionAge()>=3000;
  $('experiment-abort').disabled=zenWait?false:!connected||!owned;
  $('experiment-save').disabled=!!saving||owned||!dataset;
  text('experiment-save',saving==='csv'?'Saving…':'Save CSV');
  for(const name of plotNames){
    $(`experiment-${name}-png`).disabled=!!saving||owned||!dataset||(name==='sweep-phase'&&!sweepPhases.length);
    text(`experiment-${name}-png`,saving===name?'Saving…':'Save PNG');
  }
  if(sweep&&(sweep.received||sweep.index===0)&&!starting&&!session?.experimentOwned&&!session?.pending&&!experimentUnresolved){
    if(sweepTimer===null)sweepTimer=setTimeout(()=>{sweepTimer=null;void nextSweepRun();},0);
  }
  // Keep the running experiment and ABORT visible until hardware is released.
  if(showResults&&!owned){showResults=false;tab('data');}
  drawPlanner();
}
function tab(next) {
  if(configurations[mode])configurations[mode]=fields.map(k=>$(`experiment-${k}`).value);
  view=next;
  if(next!=='data')mode=next;
  for(const name of modes)$(`tab-${name}`).setAttribute('aria-selected',String(view===name));
  $('dashboard-panel').hidden=view!=='dashboard';
  $('experiment-panel').hidden=!configurations[view];
  $('experiment-data-panel').hidden=view!=='data';
  if(configurations[view]){
    const sine=mode==='sinusoidal';
    $('experiment-speed-field').hidden=sine;
    $('experiment-title').textContent=sine?'Sinusoidal experiment':'Quasi-static experiment';
    for(const [key,label] of Object.entries({start:sine?'Center position (µm)':'Start position (µm)',increment:sine?'Amplitude (µm)':'Signed step (µm)',timing:sine?'Frequency (Hz)':'Hold time (s)',repeats:sine?'Number of cycles':'Number of steps'}))$(`experiment-${key}-label`).textContent=label;
    fields.forEach((k,i)=>{$(`experiment-${k}`).value=configurations[mode][i];});
    $('experiment-timing').min=sine?'0.1':'0.001';$('experiment-timing').max=sine?'10':'4294907';
  }
  experimentControls();draw();
  context.viewChanged?.();
}
export function experimentMessage(message) {
  if(message.type==='experiment_state') {
    if(message.state!=='dashboard'&&(view==='dashboard'||view==='data'))tab(configurations[mode]?mode:'quasi_static');
    $('experiment-state').textContent=message.state==='running'?`Run ${message.run}: Teensy controlling experiment. Live telemetry paused.`:message.state==='data_ready'?`Run ${message.run}: downloading retained dataset…`:'Ready to configure an experiment.';
  }
  if(message.type==='experiment_ack') {
    if(sweep&&message.action==='start')sweep.run=message.run;
    $('experiment-state').textContent=message.action==='start'?`Run ${message.run}: accepted. Teensy positioning and acquiring; live telemetry paused.`:message.action==='abort'?'ABORT received. Downloading retained samples…':'Dataset verified and acknowledged. Hardware released.';
  }
  if(message.type==='experiment_dataset') {
    dataset=message.dataset;
    if(!message.restored){
      // Native index zero starts a new experiment; later indices belong to its sweep.
      if(dataset.archive_index===0)history=[];
      history.push(dataset);selectedIndex=history.length-1;updateRunPicker();
    }
    sweepPhases=sweepPhasePoints(history);
    $('experiment-sweep-results').hidden=!sweepPhases.length;
    const available=sweepPhases.filter(point=>!point.reason).length;
    text('experiment-sweep-phase-summary',`${available} of ${sweepPhases.length} runs plotted${available<sweepPhases.length?' · unavailable fits omitted':''}`);
    if(sweep&&dataset.metadata.run===sweep.run){
      if(dataset.metadata.outcome==='completed'){
        if(!sweep.received)sweep.completed++;
        sweep.received=true;
        sweepProgress(`Sweep ${sweep.index}/${sweep.configs.length} · ${sweep.configs[sweep.index-1].timing} Hz verified. Waiting for hardware release…`);
      }
      else {sweepProgress(`Sweep stopped: run ${sweep.index} ${dataset.metadata.outcome}. Results retained.`,'stopped');sweep=null;}
    }
    const m=dataset.metadata;
    if(!message.restored&&view!=='data'&&mode!==m.config.kind)tab(m.config.kind);
    showResults=true;
    $('experiment-data-empty').hidden=true;
    $('experiment-results').hidden=false;
    $('experiment-phase').hidden=m.config.kind!=='sinusoidal';
    text('experiment-data-status','');
    text('experiment-data-summary',`${m.outcome.toUpperCase()} · Run ${m.run} · ${m.config.kind.replaceAll('_',' ')} · ${m.count} samples · ${(m.elapsed_ms/1000).toFixed(3)} s`);
    text('experiment-force-calibration',dataset.samples.length?forceRangeLabel(dataset.samples.map(s=>s[2])):'No force samples');
    $('experiment-result-summary').textContent=`${m.outcome.toUpperCase()} · Run ${m.run} · ${m.config.kind.replaceAll('_',' ')} · ${m.reason} · ${m.count} samples · ${(m.elapsed_ms/1000).toFixed(3)} s · CRC verified · missed samples ${m.missed_samples} · max logging lateness ${m.max_lateness_ms} ms${m.config.control==='gpio_pulse_direction'?' · GPIO output disabled':m.stop_queued?'':' · STOP could not be queued'}`;
    if(m.fault_stat!==undefined)$('experiment-result-summary').textContent+=` · Fault STAT ${m.fault_stat}`;
    if(m.timing){const t=m.timing;
      $('experiment-result-summary').textContent+=` · EPOS ${(t.epos.rate_millihz/1000).toFixed(1)} Hz (${t.epos.updates} updates), max gap ${t.epos.max_ms} ms, max receipt age ${t.max_epos_age_ms} ms, ${t.epos_repeated_records} repeated records · pressure ${(t.pressure.rate_millihz/1000).toFixed(1)} Hz · USB max service gap ${t.max_usb_service_gap_ms} ms, RX peak ${t.rx_peak_bytes} B · feedback settings ${t.feedback_verified?'verified':'unverified'}`;
    }
    if(Number.isFinite(m.force_tare_raw))$('experiment-result-summary').textContent+=` · force zero baseline ${m.force_tare_raw} LPS counts`;
    $('experiment-result-summary').textContent+=' · EPOS timing: host receipt; controller TIME is unassociated, acquisition/USB latency unknown';
    const excursion=waveformExcursion(dataset);
    if(excursion){
      $('experiment-result-summary').textContent+=` · largest logged waveform envelope excursion ${(excursion.counts*.078).toFixed(3)} µm${excursion.counts?` at ${(excursion.receiptMs/1000).toFixed(3)} s`:''} (host receipt timing; ${excursion.observations} unique readings)`;
    }
    if(m.envelope&&m.config.kind==='sinusoidal')$('experiment-result-summary').textContent+=m.envelope.enabled?` · envelope STOP margin ${(m.envelope.margin_nm/1000).toFixed(3)} µm`:' · envelope STOP guard disabled';
    const delays=datasetPhases(dataset);
    const format=value=>value.reason?`Unavailable: ${value.reason}`:`${value.ms.toFixed(2)} ms (${value.degrees.toFixed(1)}°), fit R² ${value.r2.toFixed(3)}`;
    text('experiment-phase-command',delays.reason||format(delays.commandToPosition));
    text('experiment-phase-pressure',delays.reason||format(delays.positionToPressure));
    const fit=delays.commandToPosition;
    if(delays.reason||fit.reason){
      text('experiment-amplitude',`Unavailable: ${delays.reason||fit.reason}`);
    }else{
      const measured=fit.amplitude.measured*.078,commanded=fit.amplitude.reference*.078,ratio=measured/commanded;
      const signed=(value,digits)=>{const rounded=+value.toFixed(digits);return `${rounded>0?'+':''}${rounded.toFixed(digits)}`;};
      text('experiment-amplitude',`${measured.toFixed(3)} / ${commanded.toFixed(3)} µm · ${(ratio*100).toFixed(2)}% achieved · error ${signed(measured-commanded,3)} µm (${signed((ratio-1)*100,2)}%) · fit R² ${fit.r2.toFixed(3)}`);
    }
    draw();
  }
  experimentControls();
}
// Pressure retains every logger sample at experiment elapsed time.
// Position uses unique publications at host receipt time, including pre-start samples.
// TIME is an independent report, never an EPOS timestamp or a phase correction.
export function signalPoints(samples, field, scale=1) {
  if(field===2)return samples.map(s=>[s[0],s[field]*scale]);
  const age=4, generation=10;
  let previous;
  return samples.filter(s=>{const fresh=s[generation]!==previous;previous=s[generation];return fresh;})
    .map(s=>[s[0]-s[age],s[field]*scale]);
}
// Scan all unique logged encoder publications for waveform envelope excursions.
// immediately before/after it. Preserve raw data and use the same rounded
// encoder endpoints as firmware. This is an observed excursion, not a claim
// about unrecorded motion or the exact sensor conversion time.
export function waveformExcursion(result) {
  const c=result.metadata.config;
  if(c.kind!=='sinusoidal'||![c.center_nm,c.amplitude_nm,c.frequency_mhz,c.cycles,result.metadata.elapsed_ms].every(Number.isFinite)
    ||!(c.amplitude_nm>0&&c.frequency_mhz>0&&c.cycles>0))return null;
  const end=Math.min(result.metadata.elapsed_ms,c.cycles*1e6/c.frequency_mhz);
  const counts=nm=>Math.trunc((nm+(nm<0?-39:39))/78);
  const lower=counts(c.center_nm-c.amplitude_nm),upper=counts(c.center_nm+c.amplitude_nm);
  const worst={counts:0,receiptMs:null,observations:0};
  for(const [ms,position] of signalPoints(result.samples,1)){
    if(ms<0||ms>end)continue;
    worst.observations++;
    const beyond=Math.max(lower-position,position-upper,0);
    if(beyond>worst.counts){worst.counts=beyond;worst.receiptMs=ms;}
  }
  return worst.observations?worst:null;
}
export function forcePoints(samples,tareRaw=null) {
  return samples.map(s=>[s[0],forceMillinewtons(s[2],tareRaw)]);
}
function datasetPhases(result) {
  // Positive affine calibration preserves phase. Fit raw counts to keep the
  // existing signal-quality thresholds independent of force display units.
  if(!phaseCache.has(result))phaseCache.set(result,phaseDelays(result,signalPoints(result.samples,1),signalPoints(result.samples,2)));
  return phaseCache.get(result);
}
export function sweepPhasePoints(results) {
  return results.flatMap((result,index)=>{
    if(result.metadata.config.kind!=='sinusoidal')return [];
    const frequency=result.metadata.config.frequency_mhz/1000,delays=datasetPhases(result);
    const phase=delays.reason?{reason:delays.reason}:delays.positionToPressure;
    return [{index,frequency,...phase}];
  });
}
function drawPlot(id,field,scale) {
  const canvas=$(id),rect=canvas.getBoundingClientRect();if(!rect.width||!dataset)return;
  const ctx=canvas.getContext('2d'),dpr=Math.min(devicePixelRatio||1,2);
  const width=Math.round(rect.width*dpr),height=Math.round(rect.height*dpr);
  if(canvas.width!==width)canvas.width=width;
  if(canvas.height!==height)canvas.height=height;
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  const samples=dataset.samples,l=65,r=rect.width-16,t=15,b=rect.height-30;
  const points=field===2?forcePoints(samples,dataset.metadata.force_tare_raw):signalPoints(samples,field,scale);
  const command=field===1?samples.map(s=>[s[0],s[7]*scale]):[];
  let lo=Infinity,hi=-Infinity;for(const [,value] of [...points,...command]){lo=Math.min(lo,value);hi=Math.max(hi,value);}
  if(!samples.length){ctx.fillText('No samples acquired',l,40);return;}
  const pad=Math.max((hi-lo)*.1,scale);lo-=pad;hi+=pad;const end=Math.max(dataset.metadata.elapsed_ms,1);
  const start=field===1?points.reduce((min,p)=>Math.min(min,p[0]),0):0;
  const x=ms=>l+(ms-start)/(end-start)*(r-l),y=value=>b-(value-lo)/(hi-lo)*(b-t);
  ctx.font='10px system-ui';ctx.fillStyle='#646b65';
  for(let i=0;i<=4;i++){
    const y=t+(b-t)*i/4;ctx.strokeStyle='#e0e3e0';ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(r,y);ctx.stroke();ctx.textAlign='right';ctx.fillText((hi-(hi-lo)*i/4).toFixed(3),l-6,y+3);
    ctx.textAlign='center';ctx.fillText(((start+(end-start)*i/4)/1000).toFixed(2),l+(r-l)*i/4,rect.height-8);
  }
  ctx.strokeStyle='#919b96';ctx.lineWidth=1;ctx.beginPath();
  command.forEach(([ms,value],i)=>{if(i)ctx.lineTo(x(ms),y(value));else ctx.moveTo(x(ms),y(value));});ctx.stroke();
  if(field===2){
    ctx.strokeStyle='#32684d';ctx.lineWidth=1.5;ctx.beginPath();
    points.forEach(([ms,value],i)=>{if(i)ctx.lineTo(x(ms),y(value));else ctx.moveTo(x(ms),y(value));});ctx.stroke();
    return;
  }
  // Position points only: no interpolation across missing/deduplicated data.
  ctx.fillStyle='#32684d';
  for(const [ms,value] of points){ctx.beginPath();ctx.arc(x(ms),y(value),2,0,2*Math.PI);ctx.fill();}
}
function drawSweepPhase() {
  const canvas=$('experiment-sweep-phase-plot'),rect=canvas.getBoundingClientRect();
  if(!rect.width||!sweepPhases.length)return;
  const ctx=canvas.getContext('2d'),dpr=Math.min(devicePixelRatio||1,2);
  const width=Math.round(rect.width*dpr),height=Math.round(rect.height*dpr);
  if(canvas.width!==width)canvas.width=width;
  if(canvas.height!==height)canvas.height=height;
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  const selectedUnit=$('experiment-phase-unit').value;
  const unit=['degrees','radians'].includes(selectedUnit)?selectedUnit:'ms';
  const points=sweepPhases.filter(p=>!p.reason&&Number.isFinite(p[unit]));
  const frequencies=sweepPhases.map(p=>p.frequency).filter(f=>Number.isFinite(f)&&f>0);
  let xmin=frequencies.length?Math.min(...frequencies):0,xmax=frequencies.length?Math.max(...frequencies):1;
  const xpad=Math.max((xmax-xmin)*.05,xmax*.05,.01);xmin=Math.max(0,xmin-xpad);xmax+=xpad;
  const phaseLimit=unit==='degrees'?180:Math.PI;
  let ymin=unit==='ms'?Math.min(0,...points.map(p=>p.ms)):-phaseLimit;
  let ymax=unit==='ms'?Math.max(0,...points.map(p=>p.ms)):phaseLimit;
  if(unit==='ms'){const pad=Math.max((ymax-ymin)*.1,1);ymin-=pad;ymax+=pad;}
  const l=75,r=rect.width-20,t=20,b=rect.height-48;
  const x=value=>l+(value-xmin)/(xmax-xmin)*(r-l),y=value=>b-(value-ymin)/(ymax-ymin)*(b-t);
  ctx.font='10px system-ui';ctx.fillStyle='#646b65';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const value=ymin+(ymax-ymin)*i/4,py=y(value);
    ctx.strokeStyle='#e0e3e0';ctx.beginPath();ctx.moveTo(l,py);ctx.lineTo(r,py);ctx.stroke();
    const label=unit==='radians'?['−π','−π/2','0','π/2','π'][i]:value.toFixed(unit==='degrees'?0:1);
    ctx.textAlign='right';ctx.fillText(label,l-8,py+3);
    const frequency=xmin+(xmax-xmin)*i/4;
    ctx.textAlign='center';ctx.fillText(String(Number(frequency.toFixed(3))),x(frequency),b+16);
  }
  ctx.font='12px system-ui';ctx.textAlign='center';ctx.fillText('Frequency (Hz)',(l+r)/2,rect.height-7);
  ctx.save();ctx.translate(16,(t+b)/2);ctx.rotate(-Math.PI/2);
  ctx.fillText(`Phase delay (${unit==='degrees'?'°':unit==='radians'?'rad':'ms'})`,0,0);ctx.restore();
  ctx.strokeStyle='#919b96';ctx.beginPath();ctx.moveTo(l,y(0));ctx.lineTo(r,y(0));ctx.stroke();
  ctx.fillStyle='#32684d';ctx.strokeStyle='#32684d';
  for(const point of points){
    ctx.beginPath();ctx.arc(x(point.frequency),y(point[unit]),3,0,Math.PI*2);ctx.fill();
    if(point.index===selectedIndex){ctx.beginPath();ctx.arc(x(point.frequency),y(point[unit]),6,0,Math.PI*2);ctx.stroke();}
  }
  if(!points.length){ctx.fillStyle='#646b65';ctx.fillText('No reliable phase fits available',(l+r)/2,(t+b)/2-12);}
}
function quasiOptions() {
  return {cycle_wait_s:Number($('experiment-cycle-wait').value),predelay_s:Number($('experiment-predelay').value),cycles:Number($('experiment-cycles').value),return_to_start:$('experiment-return').checked};
}
function quasiConfig() {
  return {kind:'quasi_static',start_um:Number($('experiment-start').value),increment_um:Number($('experiment-increment').value),timing:Number($('experiment-timing').value),repeats:Number($('experiment-repeats').value),speed_um_s:Number($('experiment-speed').value),...quasiOptions()};
}
function plannerPosition() {
  const state=context.state(),position=state.snapshot?.values.EPOS?.value;
  return state.connected&&context.positionAge()<3000&&Number.isFinite(position)?position*.078:null;
}
function calculatorLocked() {
  const state=context.state();
  return mode!=='quasi_static'||starting||!!zenWait||!!sweep||!!state.session?.experimentOwned||state.experimentUnresolved;
}
function clearCalculator(message='Enter the z-stack time and buffer, then calculate a suggested plan.') {
  calculator=null;$('calculator-results').hidden=true;$('calculator-apply').disabled=true;
  text('calculator-status',message);
}
function calculateTiming() {
  if(calculatorLocked())return null;
  clearCalculator();
  try {
    calculator=calculateExperiment(quasiConfig(),{stack_s:Number($('calculator-stack').value),buffer_s:Number($('calculator-buffer').value)},plannerPosition());
    const r=calculator,fmt=value=>`${value} s`;
    for(const [key,value] of Object.entries({interval:r.interval_s,predelay:r.config.predelay_s,hold:r.config.timing,wait:r.config.cycle_wait_s,offset:r.start_s}))text(`calculator-${key}-result`,fmt(value));
    text('calculator-status',`${r.acquisitions} planned acquisitions: one in the initial predelay, each step hold and each wait between cycles. At least ${r.buffer_s} s buffer before and after each stack in the nominal plan. ${r.assumed_start?'Position unavailable: assumes actuator already at the configured start.':`Initial position used: ${r.initial_um.toFixed(3)} µm.`}${r.plan.truncated?' Preview shows only the beginning of the plan.':''}`);
    text('calculator-zen-interval',`${r.interval_s} s`);
    text('calculator-zen-count',String(r.acquisitions));
    $('calculator-results').hidden=false;$('calculator-apply').disabled=false;
    drawPlan($('calculator-plot'),r.plan,r.overlay);
    return r;
  }catch(e){clearCalculator(`Could not calculate: ${e.message??e}`);return null;}
}
function drawPlanner() {
  if(mode!=='quasi_static'||view!=='quasi_static')return;
  let plan=null;
  try {
    plan=quasiStaticPlan(quasiConfig(),plannerPosition());
    text('experiment-planner-summary',`${plan.cycles} cycle${plan.cycles===1?'':'s'} × ${plan.steps} steps · estimated ${plan.duration.toFixed(3)} s · finish at ${plan.endpoint.toFixed(3)} µm${plan.known?'':' · initial approach omitted until fresh position is available'}${plan.truncated?' · preview shows only the beginning of this long plan':''}`);
  }catch(e){text('experiment-planner-summary',e.message);}
  // Report the entire run, even when the motion preview is truncated or hidden.
  text('planner-zen-interval-setting','—');text('planner-zen-count-setting','—');
  text('planner-zen-settings-note','Enter valid motion and ZEN timing to see the settings for the entire run.');
  if(plan)try {
    const schedule=zenAcquisitions(plan.duration,{stack_s:Number($('planner-zen-stack').value),interval_s:Number($('planner-zen-interval').value),start_s:Number($('planner-zen-offset').value)});
    text('planner-zen-interval-setting',`${schedule.interval_s} s`);
    text('planner-zen-count-setting',String(schedule.count));
    text('planner-zen-settings-note','Current inputs · counts acquisitions starting before the entire planned run ends, using the first-stack delay above.');
  }catch{}
  const enabled=$('planner-zen-enabled').checked;
  $('planner-zen-options').hidden=!enabled;
  let overlay=null;
  text('planner-zen-summary','');
  if(enabled)try {
    overlay=zenAcquisitions(plan?.segments.at(-1)?.end??0,{stack_s:Number($('planner-zen-stack').value),interval_s:Number($('planner-zen-interval').value),start_s:Number($('planner-zen-offset').value)});
    text('planner-zen-summary',`${overlay.stack_s} s acquiring + ${overlay.wait_s} s waiting = ${overlay.interval_s} s start-to-start · first stack at ${overlay.start_s} s${overlay.dense?' · stacks too dense to distinguish; shading shows average acquisition coverage':''}`);
  }catch(e){text('planner-zen-summary',e.message);}
  drawPlan($('experiment-planner-plot'),plan,overlay);
}
function draw(){drawPlot('experiment-position-plot',1,.078);drawPlot('experiment-pressure-plot',2,1);drawSweepPhase();}
function updateRunPicker() {
  const picker=$('experiment-run');
  if(picker.replaceChildren){
    picker.replaceChildren(...history.map((result,index)=>{
      const option=document.createElement('option'),m=result.metadata;
      option.value=String(index);option.textContent=`Run ${m.run} · ${m.config.frequency_mhz?`${m.config.frequency_mhz/1000} Hz`:m.config.kind} · ${m.outcome}`;
      return option;
    }));
  }
  picker.value=String(selectedIndex);
}
async function nextSweepRun() {
  const current=sweep,state=context.state();
  if(!current||starting||!state.connected||state.session!==current.session||state.session.experimentOwned||state.session.pending||state.experimentUnresolved)return;
  if(current.index===current.configs.length){
    sweepProgress(`Sweep complete: ${current.index} runs. Select a run to view or save its CSV.`,'complete');sweep=null;experimentControls();return;
  }
  current.received=false;starting=true;
  const config=current.configs[current.index++];
  sweepProgress(`Sweep ${current.index}/${current.configs.length} · ${config.timing} Hz`);
  experimentControls();
  try {const ack=await current.session.startExperiment(config,current.index>1);if(sweep===current)current.run=ack.run;}
  catch(e){if(sweep===current){sweepProgress(`Sweep stopped: ${e.message??e}. Results retained.`,'stopped');sweep=null;}}
  finally {starting=false;context.controls();}
}
export function setupExperiments(api) {
  cancelZen();zenChoosing=false;
  context=api;historyLoading=!!api.invoke;validationCancelled=false;clearTimeout(sweepTimer);sweepTimer=null;sweep=null;history=[];selectedIndex=null;dataset=null;starting=false;saving=false;mode='dashboard';view='dashboard';showResults=false;
  text('experiment-sweep-status','');
  $('experiment-results').hidden=true;
  $('experiment-data-empty').hidden=false;
  phaseCache=new WeakMap();sweepPhases=[];
  $('experiment-sweep-results').hidden=true;
  $('experiment-phase-unit').onchange=drawSweepPhase;
  for(const name of plotNames)$(`experiment-${name}-png`).onclick=async()=>{
    if(saving||zenWait||sweep||starting||api.state().session?.experimentOwned||api.state().experimentUnresolved||!dataset||view!=='data')return;
    if(name==='sweep-phase'&&!sweepPhases.length)return;
    const index=dataset.archive_index??selectedIndex;
    saving=name;experimentControls();
    try{
      draw();
      const png=await plotPng($(`experiment-${name}-card`));
      const path=await api.invoke('experiment_save_png',{index,plot:name.replaceAll('-','_'),png});
      text('experiment-data-status',path?`PNG saved to ${path}`:'Save cancelled. The plot is still available.');
    }catch(e){text('experiment-data-status',`PNG export failed: ${e.message??e}`);}
    finally{saving=false;experimentControls();}
  };
  $('experiment-control').value='displacement';
  $('experiment-control').onchange=experimentControls;
  $('experiment-sweep-enabled').onchange=experimentControls;
  $('experiment-zen-enabled').checked=false;
  $('experiment-zen-enabled').onchange=()=>{
    if(!$('experiment-zen-enabled').checked&&zenWait)cancelZen('ZEN wait cancelled.');
    experimentControls();
  };
  $('experiment-zen-browse').onclick=async()=>{
    if(zenChoosing||zenWait||starting||sweep)return;
    zenChoosing=true;experimentControls();
    try{
      const folder=await api.invoke('zeiss_choose_folder');
      if(folder&&!zenWait&&!starting&&!sweep)$('experiment-zen-folder').value=folder;
    }catch(e){startStatus(`Could not choose ZEN folder: ${e.message??e}`);}
    finally{zenChoosing=false;experimentControls();}
  };
  $('experiment-run').onchange=()=>{
    selectedIndex=Number($('experiment-run').value);
    if(history[selectedIndex])experimentMessage({type:'experiment_dataset',dataset:history[selectedIndex],restored:true});
  };
  for(const name of modes)$(`tab-${name}`).onclick=()=>tab(name);
  $('experiment-current').onclick=()=>{$('experiment-start').value=(api.state().snapshot.values.EPOS.value*.078).toFixed(3);clearCalculator('Start position changed. Calculate again to update the suggestion.');drawPlanner();};
  $('experiment-form').oninput=()=>{clearCalculator('Experiment settings changed. Calculate again to update the suggestion.');drawPlanner();};
  clearCalculator();$('calculator-stack').value='40';$('calculator-buffer').value='1';
  for(const key of ['stack','buffer'])$(`calculator-${key}`).oninput=()=>clearCalculator('Calculator inputs changed. Calculate again to update the suggestion.');
  $('calculator-calculate').onclick=calculateTiming;
  $('calculator-apply').onclick=()=>{
    if(calculatorLocked()||!calculator)return;
    // Recompute with the latest encoder position before applying a stored preview.
    const r=calculateTiming();if(!r)return;
    $('experiment-predelay').value=String(r.config.predelay_s);
    $('experiment-timing').value=String(r.config.timing);
    $('experiment-cycle-wait').value=String(r.config.cycle_wait_s);
    $('planner-zen-enabled').checked=true;
    $('planner-zen-stack').value=String(r.stack_s);
    $('planner-zen-interval').value=String(r.interval_s);
    $('planner-zen-offset').value=String(r.start_s);
    clearCalculator('Suggested timings applied. Set the interval and first-stack delay in ZEN separately; no acquisition or experiment has been started.');
    drawPlanner();
  };
  new ResizeObserver(()=>{if(calculator)drawPlan($('calculator-plot'),calculator.plan,calculator.overlay);}).observe($('calculator-results'));
  $('planner-zen-enabled').checked=false;
  $('planner-zen-stack').value='40';$('planner-zen-interval').value='60';$('planner-zen-offset').value='0';
  for(const key of ['enabled','stack','interval','offset'])$(`planner-zen-${key}`).oninput=drawPlanner;
  new ResizeObserver(drawPlanner).observe($('experiment-planner'));
  $('experiment-cycle-wait').value='0';
  $('experiment-predelay').value='0';$('experiment-cycles').value='1';$('experiment-return').checked=false;
  $('experiment-form').onsubmit=async event=>{
    event.preventDefault();
    const state=api.state(),blocked=startBlocked(state);
    if(blocked){startStatus(blocked);return;}
    // Validate here so a native number-input rejection cannot swallow START
    // before we can display a persistent explanation in the panel and log.
    for(const key of [...fields,...(mode==='quasi_static'?['predelay','cycle-wait','cycles']:[])]){
      if(key==='speed'&&mode!=='quasi_static'||key==='timing'&&mode==='sinusoidal'&&$('experiment-sweep-enabled').checked)continue;
      const input=$(`experiment-${key}`);
      if(!input.reportValidity()){
        startStatus(`${$(`experiment-${key}-label`).textContent}: ${input.validationMessage}`);
        return;
      }
    }
    const config={kind:mode,start_um:Number($('experiment-start').value),increment_um:Number($('experiment-increment').value),timing:Number($('experiment-timing').value),repeats:Number($('experiment-repeats').value)};
    if(mode==='sinusoidal')config.sine_control=$('experiment-control').value;
    if(mode==='quasi_static')Object.assign(config,{speed_um_s:Number($('experiment-speed').value),...quasiOptions()});
    if($('experiment-zen-enabled').checked){await armZen(config,state);return;}
    if(mode==='sinusoidal'&&$('experiment-sweep-enabled').checked){
      validationCancelled=false;starting=true;experimentControls();
      try {
        const configs=parseFrequencies($('experiment-frequencies').value).map(timing=>({...config,timing}));
        await api.invoke('experiment_validate',{configs});
        if(validationCancelled)throw Error('Sweep cancelled.');
        if(api.state().session!==state.session||!api.state().connected)throw Error('Connection changed during validation.');
        sweep={configs,index:0,completed:0,session:state.session,received:false,run:null};
        sweepProgress('Sweep ready. Waiting to start…');
      }catch(e){startStatus(`Sweep validation failed: ${e.message??e}`);}
      finally{starting=false;}
      if(sweep)await nextSweepRun();
      api.controls();return;
    }
    starting=true;
    startStatus('Sending START to the Teensy…');
    experimentControls();
    try{await state.session.startExperiment(config);}
    catch(e){startStatus(`START failed: ${e.message??e}`);}
    finally{starting=false;api.controls();}
  };
  $('experiment-abort').onclick=async()=>{
    if(zenWait)cancelZen('ZEN wait cancelled. No experiment was started.');
    validationCancelled=true;
    if(sweep){sweepProgress('Sweep cancelled. No further frequencies will start.','stopped');sweep=null;}
    if(!api.state().session?.experimentOwned&&!api.state().experimentUnresolved){api.controls();return;}
    try{await api.state().session.request('ABORT');}
    catch(e){$('experiment-state').textContent=`ABORT not acknowledged: ${e.message}. Autonomous operation can continue after PC disconnect. Reconnect and retry ABORT.`;}
    finally{api.controls();}
  };
  $('experiment-save').onclick=async()=>{
    if(saving||zenWait||sweep||starting||api.state().session?.experimentOwned||api.state().experimentUnresolved||!dataset)return;
    saving='csv';experimentControls();
    try{
      const path=await api.invoke('experiment_save_csv',{index:dataset.archive_index??selectedIndex});
      const message=path?`CSV saved to ${path}`:'Save cancelled. The dataset is still available.';
      text('experiment-state',message);text('experiment-data-status',message);
    }catch(e){const message=`CSV export failed: ${e}`;text('experiment-state',message);text('experiment-data-status',message);}
    finally{saving=false;experimentControls();}
  };
  new ResizeObserver(draw).observe($('experiment-results'));
  if(api.invoke)void api.invoke('experiment_results').then(results=>{
    if(!results?.length)return;
    history=results;selectedIndex=history.length-1;updateRunPicker();
    experimentMessage({type:'experiment_dataset',dataset:history[selectedIndex],restored:true});
  }).catch(e=>startStatus(`Could not restore recorded runs: ${e.message??e}`)).finally(()=>{historyLoading=false;experimentControls();});
  experimentControls();
}
