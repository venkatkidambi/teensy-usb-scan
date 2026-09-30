import {phaseDelays,parseFrequencies} from './phase.js';
// UI only: native Rust validates configuration, receives binary data and builds CSV.
const $=id=>document.getElementById(id);
const text=(id,value)=>{const node=$(id);if(node.textContent!==value)node.textContent=value;};
let mode='dashboard',view='dashboard',showResults=false,dataset=null,context=null,saving=false,starting=false;
let sweep=null,history=[],selectedIndex=null,sweepTimer=null,historyLoading=false,validationCancelled=false;
const modes=['dashboard','quasi_static','sinusoidal','data'];
const fields=['start','increment','timing','repeats','speed'];
const configurations={quasi_static:['0','1','1','3','10'],sinusoidal:['0','1','1','3','10']};
function startBlocked({session,connected,hello,experimentUnresolved}) {
  if(historyLoading)return 'Loading recorded experiments…';
  if(starting)return 'Sending START to the Teensy…';
  if(sweep)return 'Frequency sweep in progress. Press ABORT to stop it.';
  if(saving)return 'Wait for CSV saving to finish.';
  if(!connected||!session)return 'Connect the Teensy to start an experiment.';
  if(session.experimentOwned)return 'An experiment owns the hardware. Wait for its results or press ABORT.';
  if(experimentUnresolved)return 'Reconnect to recover the previous experiment before starting another.';
  if(!hello?.experiments)return 'Update the Teensy firmware to enable experiments.';
  if(mode==='quasi_static'&&!hello?.quasi_static_speed)return 'Update the Teensy firmware to enable quasi-static speed control.';
  if(mode==='sinusoidal'&&!hello?.sine_position)return 'Update the Teensy firmware to enable sinusoidal position control.';
  if(session.pending)return 'Waiting for the current Teensy request to finish…';
  return '';
}
function startStatus(message) {
  $('experiment-state').textContent=message;
  context.activity?.(message);
}
export function experimentControls() {
  if(!context)return;
  const {session,connected,hello,snapshot,experimentUnresolved}=context.state();
  if(sweep&&(!connected||session!==sweep.session)){
    sweep=null;text('experiment-sweep-status','Sweep stopped after disconnect. Recover the current run before starting another sweep.');
  }
  const owned=!!sweep||starting||!!session?.experimentOwned||experimentUnresolved;
  for(const name of modes) $(`tab-${name}`).disabled=owned;
  $('dashboard-panel').inert=owned;
  for(const control of $('experiment-form').elements)control.disabled=owned;
  $('experiment-speed').disabled=owned||mode!=='quasi_static';
  const sweeping=mode==='sinusoidal'&&$('experiment-sweep-enabled').checked;
  $('experiment-frequencies').disabled=owned||!sweeping;
  $('experiment-timing').disabled=owned||sweeping;
  $('experiment-sweep-fields').hidden=mode!=='sinusoidal';
  $('experiment-run').disabled=owned||!history.length;
  const blocked=startBlocked(context.state());
  $('experiment-start-button').disabled=!!blocked;
  $('experiment-start-button').title=blocked;
  text('experiment-start-reason',blocked);
  $('experiment-start-reason').hidden=!blocked;
  text('experiment-motion-detail',mode==='sinusoidal'?`Sine ${hello?.sine_velocity?'velocity/direction control':'position commands'}. Starts at center and finishes on confirmed center return, without a hold.`:'Speed applies to start positioning and every step. The controller confirms the speed before motion; each hold begins after confirmed arrival.');
  $('experiment-current').disabled=owned||!connected||!snapshot?.values.EPOS||context.positionAge()>=3000;
  $('experiment-abort').disabled=!connected||!owned;
  $('experiment-save').disabled=saving||owned||!dataset;
  text('experiment-save',saving?'Saving…':'Save CSV');
  if(sweep&&(sweep.received||sweep.index===0)&&!starting&&!session?.experimentOwned&&!session?.pending&&!experimentUnresolved){
    if(sweepTimer===null)sweepTimer=setTimeout(()=>{sweepTimer=null;void nextSweepRun();},0);
  }
  // Keep the running experiment and ABORT visible until hardware is released.
  if(showResults&&!owned){showResults=false;tab('data');}
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
    $('experiment-timing').min=sine?'0.1':'0.001';$('experiment-timing').max=sine?'10':'81.9';
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
    if(sweep&&dataset.metadata.run===sweep.run){
      if(dataset.metadata.outcome==='completed')sweep.received=true;
      else {text('experiment-sweep-status',`Sweep stopped: run ${sweep.index} ${dataset.metadata.outcome}. Results retained.`);sweep=null;}
    }
    const m=dataset.metadata;
    if(!message.restored&&view!=='data'&&mode!==m.config.kind)tab(m.config.kind);
    showResults=true;
    $('experiment-data-empty').hidden=true;
    $('experiment-results').hidden=false;
    $('experiment-phase').hidden=m.config.kind!=='sinusoidal';
    text('experiment-data-status','');
    text('experiment-data-summary',`${m.outcome.toUpperCase()} · Run ${m.run} · ${m.config.kind.replaceAll('_',' ')} · ${m.count} samples · ${(m.elapsed_ms/1000).toFixed(3)} s`);
    $('experiment-result-summary').textContent=`${m.outcome.toUpperCase()} · Run ${m.run} · ${m.config.kind.replaceAll('_',' ')} · ${m.reason} · ${m.count} samples · ${(m.elapsed_ms/1000).toFixed(3)} s · CRC verified · missed samples ${m.missed_samples} · max logging lateness ${m.max_lateness_ms} ms${m.stop_queued?'':' · STOP could not be queued'}`;
    if(m.fault_stat!==undefined)$('experiment-result-summary').textContent+=` · Fault STAT ${m.fault_stat}`;
    if(m.timing){const t=m.timing;
      $('experiment-result-summary').textContent+=` · EPOS ${(t.epos.rate_millihz/1000).toFixed(1)} Hz (${t.epos.updates} updates), max gap ${t.epos.max_ms} ms, max receipt age ${t.max_epos_age_ms} ms, ${t.epos_repeated_records} repeated records · pressure ${(t.pressure.rate_millihz/1000).toFixed(1)} Hz · USB max service gap ${t.max_usb_service_gap_ms} ms, RX peak ${t.rx_peak_bytes} B · feedback settings ${t.feedback_verified?'verified':'unverified'}`;
    }
    if(m.reversals?.length){
      const summary=[[-1,'positive'],[1,'negative']].map(([direction,label])=>{
        const turns=m.reversals.filter(r=>r.direction===direction);
        if(!turns.length)return `${label} peak: no reversal captured`;
        const durations=turns.filter(r=>r.commands[3]&&r.commands[5]).map(r=>r.commands[5][2]-r.commands[3][2]);
        const duration=durations.length?`${Math.min(...durations)}–${Math.max(...durations)} ms zero→nonzero`:'resume not captured';
        const error=Math.max(...turns.map(r=>r.max_error_counts))*.078;
        return `${label} peak: ${turns.length} reversals, ${duration}, max receipt-time error ${error.toFixed(3)} µm`;
      });
      $('experiment-result-summary').textContent+=` · ${summary.join(' · ')} · reversal command/feedback details in CSV metadata`;
    }
    if(m.reversal_total>m.reversal_count)$('experiment-result-summary').textContent+=` · reversal details retained for ${m.reversal_count} of ${m.reversal_total} peaks`;
    const delays=phaseDelays(dataset,signalPoints(dataset.samples,1),signalPoints(dataset.samples,2));
    const format=value=>value.reason?`Unavailable: ${value.reason}`:`${value.ms.toFixed(2)} ms (${value.degrees.toFixed(1)}°), fit R² ${value.r2.toFixed(3)}`;
    text('experiment-phase-command',delays.reason||format(delays.commandToPosition));
    text('experiment-phase-pressure',delays.reason||format(delays.positionToPressure));
    draw();
  }
  experimentControls();
}
// Pressure retains every logger sample at experiment elapsed time.
// Position uses unique publications at host receipt time, including pre-start samples.
export function signalPoints(samples, field, scale=1) {
  if(field===2)return samples.map(s=>[s[0],s[field]*scale]);
  const age=4, generation=10;
  let previous;
  return samples.filter(s=>{const fresh=s[generation]!==previous;previous=s[generation];return fresh;})
    .map(s=>[s[0]-s[age],s[field]*scale]);
}
function drawPlot(id,field,scale) {
  const canvas=$(id),rect=canvas.getBoundingClientRect();if(!rect.width||!dataset)return;
  const ctx=canvas.getContext('2d'),dpr=Math.min(devicePixelRatio||1,2);
  const width=Math.round(rect.width*dpr),height=Math.round(rect.height*dpr);
  if(canvas.width!==width)canvas.width=width;
  if(canvas.height!==height)canvas.height=height;
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  const samples=dataset.samples,l=65,r=rect.width-16,t=15,b=rect.height-30;
  const points=signalPoints(samples,field,scale);
  const command=field===1?samples.map(s=>[s[0],s[7]*scale]):[];
  let lo=Infinity,hi=-Infinity;for(const [,value] of [...points,...command]){lo=Math.min(lo,value);hi=Math.max(hi,value);}
  if(!samples.length){ctx.fillText('No samples acquired',l,40);return;}
  const pad=Math.max((hi-lo)*.1,scale);lo-=pad;hi+=pad;const end=Math.max(dataset.metadata.elapsed_ms,1);
  const start=field===1?Math.min(0,...points.map(p=>p[0])):0;
  const x=ms=>l+(ms-start)/(end-start)*(r-l),y=value=>b-(value-lo)/(hi-lo)*(b-t);
  ctx.font='10px system-ui';ctx.fillStyle='#646b65';
  for(let i=0;i<=4;i++){
    const y=t+(b-t)*i/4;ctx.strokeStyle='#e0e3e0';ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(r,y);ctx.stroke();ctx.textAlign='right';ctx.fillText((hi-(hi-lo)*i/4).toFixed(field===1?3:0),l-6,y+3);
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
function draw(){drawPlot('experiment-position-plot',1,.078);drawPlot('experiment-pressure-plot',2,1);}
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
    text('experiment-sweep-status',`Sweep complete: ${current.index} runs. Select a run to view or save its CSV.`);sweep=null;experimentControls();return;
  }
  current.received=false;starting=true;
  const config=current.configs[current.index++];
  text('experiment-sweep-status',`Sweep ${current.index}/${current.configs.length} · ${config.timing} Hz`);
  experimentControls();
  try {const ack=await current.session.startExperiment(config,current.index>1);if(sweep===current)current.run=ack.run;}
  catch(e){if(sweep===current)sweep=null;text('experiment-sweep-status',`Sweep stopped: ${e.message??e}. Results retained.`);}
  finally {starting=false;context.controls();}
}
export function setupExperiments(api) {
  context=api;historyLoading=!!api.invoke;validationCancelled=false;clearTimeout(sweepTimer);sweepTimer=null;sweep=null;history=[];selectedIndex=null;dataset=null;starting=false;saving=false;mode='dashboard';view='dashboard';showResults=false;
  $('experiment-results').hidden=true;
  $('experiment-data-empty').hidden=false;
  $('experiment-sweep-enabled').onchange=experimentControls;
  $('experiment-run').onchange=()=>{
    selectedIndex=Number($('experiment-run').value);
    if(history[selectedIndex])experimentMessage({type:'experiment_dataset',dataset:history[selectedIndex],restored:true});
  };
  for(const name of modes)$(`tab-${name}`).onclick=()=>tab(name);
  $('experiment-current').onclick=()=>{$('experiment-start').value=(api.state().snapshot.values.EPOS.value*.078).toFixed(3);};
  $('experiment-form').onsubmit=async event=>{
    event.preventDefault();
    const state=api.state(),blocked=startBlocked(state);
    if(blocked){startStatus(blocked);return;}
    // Validate here so a native number-input rejection cannot swallow START
    // before we can display a persistent explanation in the panel and log.
    for(const key of fields){
      if(key==='speed'&&mode!=='quasi_static'||key==='timing'&&mode==='sinusoidal'&&$('experiment-sweep-enabled').checked)continue;
      const input=$(`experiment-${key}`);
      if(!input.reportValidity()){
        startStatus(`${$(`experiment-${key}-label`).textContent}: ${input.validationMessage}`);
        return;
      }
    }
    const config={kind:mode,start_um:Number($('experiment-start').value),increment_um:Number($('experiment-increment').value),timing:Number($('experiment-timing').value),repeats:Number($('experiment-repeats').value)};
    if(mode==='quasi_static')config.speed_um_s=Number($('experiment-speed').value);
    if(mode==='sinusoidal'&&$('experiment-sweep-enabled').checked){
      validationCancelled=false;starting=true;experimentControls();
      try {
        const configs=parseFrequencies($('experiment-frequencies').value).map(timing=>({...config,timing}));
        await api.invoke('experiment_validate',{configs});
        if(validationCancelled)throw Error('Sweep cancelled.');
        if(api.state().session!==state.session||!api.state().connected)throw Error('Connection changed during validation.');
        sweep={configs,index:0,session:state.session,received:false,run:null};
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
    validationCancelled=true;
    if(sweep){sweep=null;text('experiment-sweep-status','Sweep cancelled. No further frequencies will start.');}
    if(!api.state().session?.experimentOwned&&!api.state().experimentUnresolved){api.controls();return;}
    try{await api.state().session.request('ABORT');}
    catch(e){$('experiment-state').textContent=`ABORT not acknowledged: ${e.message}. Autonomous operation can continue after PC disconnect. Reconnect and retry ABORT.`;}
    finally{api.controls();}
  };
  $('experiment-save').onclick=async()=>{
    if(saving||sweep||starting||api.state().session?.experimentOwned||api.state().experimentUnresolved||!dataset)return;
    saving=true;experimentControls();
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
