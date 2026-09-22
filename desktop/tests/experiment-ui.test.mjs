import test from 'node:test';
import assert from 'node:assert/strict';
import { setupExperiments, experimentMessage, experimentControls } from '../ui/experiment.js';

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
        if (command === 'experiment_result') return Promise.resolve(null);
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
    assert.equal(node('experiment-save').disabled, false);
    const saving = node('experiment-save').onclick();
    assert.equal(node('experiment-save').disabled, true);
    assert.equal(node('experiment-start-button').disabled, true);
    await node('experiment-form').onsubmit({ preventDefault() {} });
    await node('experiment-save').onclick();
    assert.deepEqual(calls, ['experiment_result', 'experiment_save_csv']);
    resolve('/chosen folder/my run.csv');
    await saving;
    assert.match(node('experiment-state').textContent, /CSV saved to \/chosen folder\/my run.csv/);
    assert.equal(node('experiment-save').disabled, false);

    const cancelled = node('experiment-save').onclick();
    resolve(null);
    await cancelled;
    assert.match(node('experiment-state').textContent, /cancelled/);
    const failed = node('experiment-save').onclick();
    reject('Permission denied');
    await failed;
    assert.match(node('experiment-state').textContent, /CSV export failed: Permission denied/);
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
