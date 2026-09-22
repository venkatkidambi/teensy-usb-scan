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
    }
    const {experimentMessage}=await import('./experiment.js');
    experimentMessage({type:'experiment_dataset',dataset:{metadata:{run:1,config:{kind:'sinusoidal'},outcome:'aborted',reason:'test fixture',count:3,elapsed_ms:100,missed_samples:0,max_lateness_ms:0,stop_queued:true},samples:[[0,0,10],[50,10,20],[100,0,10]]}});
    await new Promise(r=>setTimeout(r,100));
    if(!document.getElementById('experiment-result-summary').textContent.includes('ABORTED'))throw Error('Experiment outcome missing');
    for(const name of ['position','pressure'])if(document.getElementById(`experiment-${name}-plot`).width<100)throw Error('Experiment plot did not render');
    document.getElementById('tab-dashboard').click();
    if(document.documentElement.scrollWidth>innerWidth)throw Error('Layout overflows window');
    document.querySelector('#connect').click();
    await new Promise(r=>setTimeout(r,100));
    if(document.querySelector('#epos').textContent!=='—'||document.querySelector('#pressure-raw').textContent!=='—')throw Error('Disconnect failed to clear data');
    await invoke('smoke_report',{result:{ok:true,ports:ports.length,nativeIPC:true,demo:true,disconnect:true}});
  }catch(e){await window.__TAURI__.core.invoke('smoke_report',{result:{ok:false,error:String(e)}});}
})();
"#;
