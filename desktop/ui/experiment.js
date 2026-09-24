// UI only: native Rust validates configuration, receives binary data and builds CSV.
const $=id=>document.getElementById(id);
let mode='dashboard',dataset=null,context=null,saving=false,starting=false;
const modes=['dashboard','quasi_static','sinusoidal'];
const fields=['start','increment','timing','repeats'];
const configurations={quasi_static:['0','1','1','3'],sinusoidal:['0','1','1','3']};
function startBlocked({session,connected,hello,experimentUnresolved}) {
  if(starting)return 'Sending START to the Teensy…';
  if(saving)return 'Wait for CSV saving to finish.';
  if(!connected||!session)return 'Connect the Teensy to start an experiment.';
  if(session.experimentOwned)return 'An experiment owns the hardware. Wait for its results or press ABORT.';
  if(experimentUnresolved)return 'Reconnect to recover the previous experiment before starting another.';
  if(!hello?.experiments)return 'Update the Teensy firmware to enable experiments.';
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
  const owned=starting||!!session?.experimentOwned||experimentUnresolved;
  for(const name of modes) $(`tab-${name}`).disabled=owned;
  $('dashboard-panel').inert=owned;
  for(const control of $('experiment-form').elements)control.disabled=owned;
  const blocked=startBlocked(context.state());
  $('experiment-start-button').disabled=!!blocked;
  $('experiment-start-button').title=blocked;
  $('experiment-start-reason').textContent=blocked;
  $('experiment-start-reason').hidden=!blocked;
  $('experiment-motion-detail').textContent=mode==='sinusoidal'?'100 Hz sine position commands. Starts and finishes at the center position.':'Steps use confirmed position arrival.';
  $('experiment-current').disabled=owned||!connected||!snapshot?.values.EPOS||context.positionAge()>=3000;
  $('experiment-abort').disabled=!connected||!owned;
  $('experiment-save').disabled=saving||owned||!dataset;
  $('experiment-save').textContent=saving?'Saving…':'Save CSV';
}
function tab(next) {
  if(mode!=='dashboard')configurations[mode]=fields.map(k=>$(`experiment-${k}`).value);
  mode=next;
  for(const name of modes)$(`tab-${name}`).setAttribute('aria-selected',String(mode===name));
  $('dashboard-panel').hidden=mode!=='dashboard';$('experiment-panel').hidden=mode==='dashboard';
  if(mode!=='dashboard'){
    const sine=mode==='sinusoidal';
    $('experiment-title').textContent=sine?'Sinusoidal experiment':'Quasi-static experiment';
    for(const [key,label] of Object.entries({start:sine?'Center position (µm)':'Start position (µm)',increment:sine?'Amplitude (µm)':'Signed step (µm)',timing:sine?'Frequency (Hz)':'Hold time (s)',repeats:sine?'Number of cycles':'Number of steps'}))$(`experiment-${key}-label`).textContent=label;
    fields.forEach((k,i)=>{$(`experiment-${k}`).value=configurations[mode][i];});
    $('experiment-timing').min=sine?'0.1':'0.001';$('experiment-timing').max=sine?'2':'204.75';
  }
  experimentControls();draw();
}
export function experimentMessage(message) {
  if(message.type==='experiment_state') {
    if(message.state!=='dashboard'&&mode==='dashboard')tab('quasi_static');
    $('experiment-state').textContent=message.state==='running'?`Run ${message.run}: Teensy controlling experiment. Live telemetry paused.`:message.state==='data_ready'?`Run ${message.run}: downloading retained dataset…`:'Ready to configure an experiment.';
  }
  if(message.type==='experiment_ack') {
    $('experiment-state').textContent=message.action==='start'?`Run ${message.run}: accepted. Teensy positioning and acquiring; live telemetry paused.`:message.action==='abort'?'ABORT received. Downloading retained samples…':'Dataset verified and acknowledged. Hardware released.';
  }
  if(message.type==='experiment_dataset') {
    dataset=message.dataset;
    const m=dataset.metadata;
    if(mode!==m.config.kind)tab(m.config.kind);
    $('experiment-results').hidden=false;
    $('experiment-result-summary').textContent=`${m.outcome.toUpperCase()} · Run ${m.run} · ${m.config.kind.replaceAll('_',' ')} · ${m.reason} · ${m.count} samples · ${(m.elapsed_ms/1000).toFixed(3)} s · CRC verified · missed samples ${m.missed_samples} · max logging lateness ${m.max_lateness_ms} ms${m.stop_queued?'':' · STOP could not be queued'}`;
    if(m.fault_stat!==undefined)$('experiment-result-summary').textContent+=` · Fault STAT ${m.fault_stat}`;
    draw();
  }
  experimentControls();
}
function drawPlot(id,field,scale) {
  const canvas=$(id),rect=canvas.getBoundingClientRect();if(!rect.width||!dataset)return;
  const ctx=canvas.getContext('2d'),dpr=Math.min(devicePixelRatio||1,2);
  canvas.width=rect.width*dpr;canvas.height=rect.height*dpr;ctx.scale(dpr,dpr);
  const samples=dataset.samples,l=65,r=rect.width-16,t=15,b=rect.height-30;
  let lo=Infinity,hi=-Infinity;for(const s of samples){lo=Math.min(lo,s[field]*scale);hi=Math.max(hi,s[field]*scale);}
  if(!samples.length){ctx.fillText('No samples acquired',l,40);return;}
  const pad=Math.max((hi-lo)*.1,scale);lo-=pad;hi+=pad;const end=Math.max(dataset.metadata.elapsed_ms,1);
  ctx.font='10px system-ui';ctx.fillStyle='#646b65';
  for(let i=0;i<=4;i++){
    const y=t+(b-t)*i/4;ctx.strokeStyle='#e0e3e0';ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(r,y);ctx.stroke();ctx.textAlign='right';ctx.fillText((hi-(hi-lo)*i/4).toFixed(field===1?3:0),l-6,y+3);
    ctx.textAlign='center';ctx.fillText((end*i/4000).toFixed(2),l+(r-l)*i/4,rect.height-8);
  }
  ctx.strokeStyle='#32684d';ctx.lineWidth=1.5;ctx.beginPath();
  samples.forEach((s,i)=>{const x=l+s[0]/end*(r-l),y=b-(s[field]*scale-lo)/(hi-lo)*(b-t);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();
}
function draw(){drawPlot('experiment-position-plot',1,.078);drawPlot('experiment-pressure-plot',2,1);}
export function setupExperiments(api) {
  context=api;
  for(const name of modes)$(`tab-${name}`).onclick=()=>tab(name);
  $('experiment-current').onclick=()=>{$('experiment-start').value=(api.state().snapshot.values.EPOS.value*.078).toFixed(3);};
  $('experiment-form').onsubmit=async event=>{
    event.preventDefault();
    const state=api.state(),blocked=startBlocked(state);
    if(blocked){startStatus(blocked);return;}
    // Validate here so a native number-input rejection cannot swallow START
    // before we can display a persistent explanation in the panel and log.
    for(const key of fields){
      const input=$(`experiment-${key}`);
      if(!input.reportValidity()){
        startStatus(`${$(`experiment-${key}-label`).textContent}: ${input.validationMessage}`);
        return;
      }
    }
    const config={kind:mode,start_um:Number($('experiment-start').value),increment_um:Number($('experiment-increment').value),timing:Number($('experiment-timing').value),repeats:Number($('experiment-repeats').value)};
    starting=true;
    startStatus('Sending START to the Teensy…');
    experimentControls();
    try{await state.session.startExperiment(config);}
    catch(e){startStatus(`START failed: ${e.message??e}`);}
    finally{starting=false;api.controls();}
  };
  $('experiment-abort').onclick=async()=>{
    try{await api.state().session.request('ABORT');}
    catch(e){$('experiment-state').textContent=`ABORT not acknowledged: ${e.message}. Autonomous operation can continue after PC disconnect. Reconnect and retry ABORT.`;}
    finally{api.controls();}
  };
  $('experiment-save').onclick=async()=>{
    if(saving||api.state().session?.experimentOwned||api.state().experimentUnresolved||!dataset)return;
    saving=true;experimentControls();
    try{
      const path=await api.invoke('experiment_save_csv');
      $('experiment-state').textContent=path?`CSV saved to ${path}`:'Save cancelled. The dataset is still available.';
    }catch(e){$('experiment-state').textContent=`CSV export failed: ${e}`;}
    finally{saving=false;experimentControls();}
  };
  new ResizeObserver(draw).observe($('experiment-results'));
  if(api.invoke)void api.invoke('experiment_result').then(result=>{if(result)experimentMessage({type:'experiment_dataset',dataset:result});}).catch(()=>{});
  experimentControls();
}
