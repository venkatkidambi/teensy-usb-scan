import test from 'node:test';
import assert from 'node:assert/strict';
import {setupMaintenance} from '../ui/maintenance.js';

function harness(invoke) {
  const previous = globalThis.document;
  const nodes = new Map();
  const node = id => {
    if(!nodes.has(id)) nodes.set(id, {checked: false, addEventListener() {}});
    return nodes.get(id);
  };
  globalThis.document = {getElementById: node};
  const state = {session: null, connected: false, busy: false, demo: false, experimentUnresolved: false};
  const calls = [];
  const ui = setupMaintenance({
    state: () => state, makeChannel: () => ({}),
    invoke: async (command, args) => {
      calls.push(command);
      if(command === 'app_release_info') return {version:'0.2.0', channel:'stable', commit:'a'.repeat(40), firmware_sha256:'b'.repeat(64), updater:true};
      return invoke(command, args);
    },
    disconnect: async () => { calls.push('disconnect'); state.session = null; state.connected = false; },
    refresh: async () => { calls.push('refresh'); },
    setBusy: value => calls.push(`busy:${value}`),
  });
  return {ui, node, state, calls, restore: () => {globalThis.document = previous;}};
}

test('flashing requires acknowledgement, closes serial first, and blocks duplicate clicks', async () => {
  let finish;
  const h = harness(() => new Promise(resolve => { finish = resolve; }));
  try {
    await h.ui.ready;
    await h.node('firmware-install').onclick();
    assert.deepEqual(h.calls, ['app_release_info']);
    h.node('maintenance-ready').checked = true;
    const flashing = h.node('firmware-install').onclick();
    await Promise.resolve();
    await h.node('firmware-install').onclick();
    assert.deepEqual(h.calls, ['app_release_info', 'busy:true', 'disconnect', 'firmware_install']);
    finish(); await flashing;
    assert.match(h.node('maintenance-status').textContent, /Firmware installed/);
    assert.equal(h.node('maintenance-ready').checked, false);
    assert.equal(h.calls.at(-1), 'busy:false');
  } finally { h.restore(); }
});

test('active or unresolved experiments block firmware and app installation', async () => {
  const h = harness(async () => '0.3.0');
  try {
    await h.ui.ready;
    await h.node('update-check').onclick();
    h.node('maintenance-ready').checked = true;
    for (const state of [{experimentUnresolved:true}, {experimentUnresolved:false, session:{experimentOwned:true}}]) {
      Object.assign(h.state, state); h.ui.render();
      assert.equal(h.node('firmware-install').disabled, true);
      assert.equal(h.node('update-install').disabled, true);
      await h.node('firmware-install').onclick();
      await h.node('update-install').onclick();
    }
    assert.deepEqual(h.calls, ['app_release_info', 'app_update_check']);
  } finally { h.restore(); }
});

test('failed updates release the busy state and require a new update check', async () => {
  const h = harness(async command => {
    if(command === 'app_update_check') return '0.3.0';
    throw Error('signature rejected');
  });
  try {
    await h.ui.ready; await h.node('update-check').onclick();
    h.node('maintenance-ready').checked = true;
    await h.node('update-install').onclick();
    assert.match(h.node('maintenance-status').textContent, /signature rejected/);
    assert.equal(h.node('update-install').disabled, true);
    assert.equal(h.calls.at(-1), 'busy:false');
  } finally { h.restore(); }
});
