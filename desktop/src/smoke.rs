// Compiled only for local validation; absent from normal/release app commands.
#[tauri::command]
pub fn smoke_report(result: serde_json::Value, app: tauri::AppHandle) {
    println!("NATIVE_SMOKE {}", result);
    app.exit(if result["ok"] == true { 0 } else { 1 });
}

pub const SCRIPT: &str = r#"
(async () => {
  try {
    for (let i=0; i<100 && (!document.querySelector('#demo') || document.querySelector('#port').textContent.includes('Scanning')); i++) await new Promise(r=>setTimeout(r,50));
    const invoke=window.__TAURI__.core.invoke;
    if(document.querySelector('.brand, .mark, .device-icon'))throw Error('Decorative branding remains');
    if(document.querySelector('h1')?.textContent!=='Teensy Panel')throw Error('Panel heading missing');
    const ports=await invoke('serial_ports');
    if(!Array.isArray(ports))throw Error('Native port listing failed');
    let saveRejected=false;
    try{await invoke('experiment_save_csv');}catch(e){saveRejected=String(e).includes('No verified experiment dataset');}
    if(!saveRejected)throw Error('Native Save As command did not reject an absent dataset');
    let rejected=false;
    try{await invoke('serial_connect',{port:'missing-test-port'});}catch{rejected=true;}
    if(!rejected)throw Error('Missing serial port was accepted');
    document.querySelector('#demo').click();
    await new Promise(r=>setTimeout(r,300));
    if(!document.querySelector('#connection').textContent.includes('Demo'))throw Error('Demo UI did not start');
    if(!document.querySelector('#flags').textContent.includes('Motor on'))throw Error('Status flags did not render');
    if(document.querySelector('#epos').textContent==='—')throw Error('Telemetry did not render');
    for(const id of ['plunger-up','plunger-down','plunger-zero','plunger-stop'])if(!document.getElementById(id).disabled)throw Error('Demo enabled physical plunger controls');
    if(document.querySelector('#pressure-raw').textContent==='—')throw Error('Pressure demo did not render');
    for(const mode of ['quasi_static','sinusoidal','dashboard']) {
      document.getElementById(`tab-${mode}`).click();
      if(document.getElementById(`tab-${mode}`).getAttribute('aria-selected')!=='true')throw Error('Mode tab failed');
      if(mode!=='dashboard'&&!document.getElementById('experiment-start-button').disabled)throw Error('Demo enabled experiment START');
      if(mode!=='dashboard'){
        let mutations=0;
        const observer=new MutationObserver(records=>{mutations+=records.length;});
        for(const id of ['plot','pressure-plot','pressure-raw'])observer.observe(document.getElementById(id),{attributes:true,childList:true,subtree:true});
        await new Promise(r=>setTimeout(r,600));
        observer.disconnect();
        if(mutations)throw Error('Hidden dashboard continued rendering');
      }else if(document.getElementById('pressure-raw').textContent==='—')throw Error('Dashboard did not refresh on return');
    }
    const {experimentMessage}=await import('./experiment.js');
    experimentMessage({type:'experiment_dataset',dataset:{metadata:{run:1,config:{kind:'sinusoidal'},outcome:'aborted',reason:'test fixture',count:3,elapsed_ms:100,missed_samples:0,max_lateness_ms:0,stop_queued:true},samples:[[0,0,10,0,5,3,0,0,0,0,1,1],[50,10,20,0,5,2,0,10,0,0,2,2],[100,0,10,0,5,1,0,0,0,0,3,3]]}});
    await new Promise(r=>setTimeout(r,100));
    if(!document.getElementById('experiment-result-summary').textContent.includes('ABORTED'))throw Error('Experiment outcome missing');
    for(const name of ['position','pressure'])if(document.getElementById(`experiment-${name}-plot`).width<100)throw Error('Experiment plot did not render');
    // Exercise sweep fitting and real WebKit PNG rendering with synthetic signals.
    for(const [index,frequency] of [1,5].entries()){
      const samples=[],duration=3000/frequency;
      for(let ms=0;ms<=duration;ms+=20){
        const sine=delay=>-Math.cos(2*Math.PI*frequency*(ms-delay)/1000);
        samples.push([ms,100+20*sine(0),100000+40*sine(20),0,0,0,0,100+20*sine(0),0,0,ms/20,ms/20]);
      }
      experimentMessage({type:'experiment_dataset',dataset:{archive_index:index,metadata:{run:index+2,config:{kind:'sinusoidal',frequency_mhz:frequency*1000,cycles:3},outcome:'completed',reason:'none',count:samples.length,elapsed_ms:duration},samples}});
    }
    if(document.getElementById('experiment-sweep-results').hidden)throw Error('Sweep phase plot is hidden');
    if(document.getElementById('experiment-sweep-phase-summary').textContent!=='2 of 2 runs plotted')throw Error('Sweep fits missing');
    const {plotPng}=await import('./plot-export.js');
    for(const unit of ['ms','degrees']){
      const picker=document.getElementById('experiment-phase-unit');picker.value=unit;picker.dispatchEvent(new Event('change'));
      for(const name of ['position','pressure','sweep-phase']){
        const card=document.getElementById(`experiment-${name}-card`),source=card.querySelector('canvas');
        const png=await plotPng(card);
        if(png.length<1000||png.slice(0,8).join(',')!=='137,80,78,71,13,10,26,10')throw Error('PNG encoding failed');
        const image=new Image();
        image.src='data:image/png;base64,'+btoa(png.map(byte=>String.fromCharCode(byte)).join(''));
        await image.decode();
        const rect=card.getBoundingClientRect(),scale=source.width/source.getBoundingClientRect().width;
        if(image.width!==Math.round(rect.width*scale)||image.height!==Math.round(rect.height*scale))throw Error('PNG changed plot dimensions');
        const copy=document.createElement('canvas');copy.width=image.width;copy.height=image.height;
        const ctx=copy.getContext('2d');ctx.drawImage(image,0,0);
        const data=ctx.getImageData(0,0,copy.width,copy.height).data;
        let green=0;
        for(let i=0;i<data.length;i+=4)if(data[i+1]>data[i]+20&&data[i+1]>data[i+2]+10)green++;
        if(green<10)throw Error('PNG lost the green plot or legend');
        // The real native command accepts this image, then rejects the absent
        // native archive before opening any dialog or writing a file.
        let rejected=false;
        try{await invoke('experiment_save_png',{plot:name.replaceAll('-','_'),png});}catch(e){rejected=String(e).includes('No verified experiment dataset');}
        if(!rejected)throw Error('Native PNG command did not validate the dataset');
      }
    }
    document.getElementById('tab-dashboard').click();
    if(document.documentElement.scrollWidth>innerWidth)throw Error('Layout overflows window');
    document.querySelector('#connect').click();
    await new Promise(r=>setTimeout(r,100));
    if(document.querySelector('#epos').textContent!=='—'||document.querySelector('#pressure-raw').textContent!=='—')throw Error('Disconnect failed to clear data');
    // Exercise actual webview button/form behavior with an isolated fake sender.
    // No physical serial port is opened and no motion command leaves this test.
    const {setupExperiments,experimentControls}=await import('./experiment.js');
    const calls=[];let rejectStart;
    const state={connected:true,hello:{experiments:true,sine_position:true,quasi_static_speed:true},session:{
      startExperiment(config){calls.push(config);return new Promise((_,reject)=>{rejectStart=reject;});}
    }};
    setupExperiments({state:()=>state,positionAge:()=>0,controls:experimentControls});
    for(const mode of ['quasi_static','sinusoidal']){
      document.getElementById(`tab-${mode}`).click();
      const panel=document.getElementById('experiment-panel');
      const results=document.getElementById('experiment-results');
      const height=panel.getBoundingClientRect().height,top=results.getBoundingClientRect().top;
      state.session.pending={expected:'pong'};experimentControls();
      if(!document.getElementById('experiment-start-button').disabled)throw Error('Pending request enabled START');
      if(panel.getBoundingClientRect().height!==height||results.getBoundingClientRect().top!==top)throw Error('Background PING shifted experiment layout');
      state.session.pending=null;experimentControls();
      if(panel.getBoundingClientRect().height!==height||results.getBoundingClientRect().top!==top)throw Error('Completed PING shifted experiment layout');
    }
    const start=document.getElementById('experiment-start-button');
    const status=document.getElementById('experiment-state');
    const repeats=document.getElementById('experiment-repeats');
    repeats.value='1.5';
    start.click();
    if(calls.length||!status.textContent.includes('Number of cycles:'))throw Error('Invalid START was silent or sent a command');
    for(const [key,value] of Object.entries({start:'374.556',increment:'5',timing:'1',repeats:'5'}))document.getElementById(`experiment-${key}`).value=value;
    start.click();start.click();
    if(calls.length!==1||!status.textContent.includes('Sending START'))throw Error('START click did not submit exactly once with visible status');
    if(JSON.stringify(calls[0])!==JSON.stringify({kind:'sinusoidal',start_um:374.556,increment_um:5,timing:1,repeats:5}))throw Error('START form values incorrect');
    rejectStart(Error('Test rejection'));
    await new Promise(r=>setTimeout(r,0));
    if(start.disabled||status.textContent!=='START failed: Test rejection')throw Error('START rejection did not recover visibly');
    state.hello.sine_position=false;experimentControls();
    if(!start.disabled||!document.getElementById('experiment-start-reason').textContent.includes('sinusoidal position control'))throw Error('Disabled START has no explanation');
    await invoke('smoke_report',{result:{ok:true,ports:ports.length,nativeIPC:true,demo:true,disconnect:true,startClick:true,startValidation:true,stableExperimentLayout:true,hiddenDashboardIdle:true,sweepPhasePlot:true,plotPng:true}});
  }catch(e){await window.__TAURI__.core.invoke('smoke_report',{result:{ok:false,error:String(e)}});}
})();
"#;
