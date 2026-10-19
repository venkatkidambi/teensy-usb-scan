export function setupMaintenance({invoke, makeChannel, state, disconnect, refresh, setBusy}) {
  const $ = id => document.getElementById(id);
  let info = null, working = false, candidate = null;
  const status = message => { $('maintenance-status').textContent = String(message); };
  function blocked() {
    const s = state();
    return working || s.busy || s.demo || s.experimentUnresolved || !!s.session?.experimentOwned;
  }
  function render() {
    const locked = blocked();
    $('firmware-install').disabled = locked || !info?.firmware_sha256 || !$('maintenance-ready').checked;
    $('update-check').disabled = locked || !info?.updater;
    $('update-install').disabled = locked || !candidate || !$('maintenance-ready').checked;
    $('maintenance-ready').disabled = working;
  }
  function progress() {
    const channel = makeChannel();
    channel.onmessage = status;
    return channel;
  }
  async function perform(command) {
    if (blocked() || !$('maintenance-ready').checked) return;
    working = true; setBusy(true); render();
    try {
      // The native backend independently rejects connected ports, concurrent
      // updates and retained autonomous ownership after this close completes.
      await disconnect('Disconnected for update');
      if (state().experimentUnresolved) throw Error('Reconnect and resolve the experiment before updating.');
      status(command === 'firmware_install' ? 'Starting Teensy flasher…' : 'Downloading app update…');
      await invoke(command, {progress: progress()});
      if (command === 'firmware_install') {
        status('Firmware installed. Wait for the Teensy to restart, then click Refresh and Connect Teensy.');
        await refresh();
      }
    } catch (error) {
      status(`Update failed: ${error.message ?? error}`);
      if (command === 'app_update_install') candidate = null;
    } finally {
      working = false; setBusy(false); $('maintenance-ready').checked = false; render();
    }
  }
  $('maintenance-ready').addEventListener('change', render);
  $('firmware-install').onclick = () => perform('firmware_install');
  $('update-install').onclick = () => perform('app_update_install');
  $('update-check').onclick = async () => {
    if (blocked() || !info?.updater) return;
    working = true; candidate = null; render(); status('Checking for updates…');
    try {
      candidate = await invoke('app_update_check');
      status(candidate ? `Version ${candidate} is available. Save your data before installing; the app will restart.` : `You have the latest ${info.channel} release.`);
    } catch (error) { status(`Could not check for updates: ${error.message ?? error}`); }
    finally { working = false; render(); }
  };
  const ready = (async () => {
    if (!invoke) { status('Open the installed desktop app to manage firmware and updates.'); render(); return; }
    try {
      info = await invoke('app_release_info');
      if(info.repository)$('release-downloads').textContent=`github.com/${info.repository}/releases`;
      $('app-version').textContent = `${info.version} · ${info.channel} · ${info.commit.slice(0, 12)}`;
      $('bundled-firmware').textContent = info.firmware_sha256 ? `Included firmware: ${info.version} · Teensy 4.1` : 'Firmware flashing is available in packaged releases.';
      status(info.updater ? `Updates stay on the ${info.channel} channel. Firmware is installed separately.` : 'In-app updates are not enabled in this build. Download new installers from the GitHub Releases page.');
    } catch (error) { status(`Could not read release information: ${error.message ?? error}`); }
    render();
  })();
  return {render, ready};
}
