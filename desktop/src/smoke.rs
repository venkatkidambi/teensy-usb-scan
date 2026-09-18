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
    const ports=await invoke('serial_ports');
    if(!Array.isArray(ports))throw Error('Native port listing failed');
    let rejected=false;
    try{await invoke('serial_connect',{port:'missing-test-port'});}catch{rejected=true;}
    if(!rejected)throw Error('Missing serial port was accepted');
    document.querySelector('#demo').click();
    await new Promise(r=>setTimeout(r,300));
    if(!document.querySelector('#connection').textContent.includes('Demo'))throw Error('Demo UI did not start');
    if(!document.querySelector('#flags').textContent.includes('Motor on'))throw Error('Status flags did not render');
    if(document.querySelector('#epos').textContent==='—')throw Error('Telemetry did not render');
    if(document.documentElement.scrollWidth>innerWidth)throw Error('Layout overflows window');
    document.querySelector('#connect').click();
    await new Promise(r=>setTimeout(r,100));
    if(document.querySelector('#epos').textContent!=='—')throw Error('Disconnect failed to clear data');
    await invoke('smoke_report',{result:{ok:true,ports:ports.length,nativeIPC:true,demo:true,disconnect:true}});
  }catch(e){await window.__TAURI__.core.invoke('smoke_report',{result:{ok:false,error:String(e)}});}
})();
"#;
