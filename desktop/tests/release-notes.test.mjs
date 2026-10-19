import test from 'node:test';
import assert from 'node:assert/strict';
import {previousRelease, commitSummary, formatChangelog, generateChangelog, releaseTitle, releaseNotes} from '../scripts/release-notes.mjs';

const previous = {
  version: '0.2.0-nightly.4.1', tag: 'nightly-0.2.0-nightly.4.1',
  channel: 'nightly', commit: 'a'.repeat(40),
  repository: 'owner/downloads', source_repository: 'owner/private-source',
};
const current = {...previous, version: '0.2.0-nightly.6.1', tag: 'nightly-0.2.0-nightly.6.1', commit: 'b'.repeat(40)};
const published = (metadata, date, extra = {}) => ({tag_name: metadata.tag, draft: false, prerelease: metadata.channel === 'nightly', published_at: date, ...extra});

test('baselines exclude drafts, channel pointers, other channels, current and newer releases', () => {
  const older = published(previous, '2026-10-18T00:00:00Z');
  const stable = published({...previous, tag: 'v0.2.0', channel: 'stable'}, '2026-10-18T02:00:00Z');
  const releases = [
    published({...current, tag: 'nightly-0.2.0-nightly.7.1'}, '2026-10-20T00:00:00Z'),
    published(current, '2026-10-19T00:00:00Z'),
    published({...current, tag: 'nightly-0.2.0-nightly.5.1'}, '2026-10-18T03:00:00Z', {draft: true}),
    published({...current, tag: 'nightly'}, '2026-10-18T04:00:00Z'),
    stable, older,
  ];
  assert.equal(previousRelease(releases, current), older);
  assert.equal(previousRelease(releases, {...current, channel: 'stable', tag: 'v0.3.0'}), stable);
  assert.equal(previousRelease([], current), null);
});

test('same-commit rebuilds are explicit even when version or workflow trigger changes', () => {
  const changes = formatChangelog({...current, commit: previous.commit}, previous);
  assert.equal(changes.label, 'Routine rebuild');
  assert.match(changes.markdown, /no new source commits/);
  assert.match(changes.markdown, /Build tools or packaging may still differ/);
  assert.match(releaseTitle(current, changes), /Routine rebuild$/);
  assert.match(changes.markdown, /owner\/downloads\/releases\/tag\/nightly-0.2.0-nightly.4.1/);
});

test('PR titles and direct commit subjects become readable public summaries without bodies', () => {
  const merge = 'Merge pull request #2 from owner/private-branch\n\nPolish firmware confirmation\n\nPrivate body';
  assert.equal(commitSummary(merge), 'Polish firmware confirmation');
  const changes = formatChangelog(current, previous, [merge, 'Fix motor controls\n\nPrivate detail']);
  assert.equal(changes.label, 'New changes');
  assert.match(changes.markdown, /- Polish firmware confirmation\n- Fix motor controls/);
  assert.doesNotMatch(changes.markdown, /private|Private|Merge pull request/);
});

test('initial releases and uncomparable history never claim to be routine rebuilds', () => {
  assert.equal(formatChangelog(current, null).label, 'Initial release');
  const missing = formatChangelog(current, previous, [], false);
  assert.equal(missing.label, 'Comparison unavailable');
  assert.match(missing.markdown, /Source revision changed/);
  assert.equal(formatChangelog(current, previous, []).label, 'New changes');
});

test('summaries escape Markdown and bound long histories without hiding the overflow', () => {
  const changes = formatChangelog(current, previous, ['Fix <b>text</b> [link](url)', ...Array(101).fill('Update controls')]);
  assert.match(changes.markdown, /\\<b\\>/);
  assert.match(changes.markdown, /\\\[link\\\]/);
  assert.equal(changes.markdown.split('\n').filter(line => line.startsWith('- ')).length, 100);
  assert.match(changes.markdown, /Plus 2 earlier changes/);
});

function runner({baseline = previous, ancestryStatus, apiError, pages = [[published(previous, '2026-10-18T00:00:00Z')]]} = {}) {
  const calls = [];
  const run = (command, args) => {
    calls.push([command, args]);
    if (command === 'gh' && args[0] === 'api') {
      if (apiError) throw apiError;
      return pages.flat().map(release => JSON.stringify(release)).join('\n');
    }
    if (command === 'gh' && args[0] === 'release') return JSON.stringify(baseline);
    if (command === 'git' && args[0] === 'merge-base') {
      if (ancestryStatus) throw Object.assign(Error('Cannot compare revisions'), {status: ancestryStatus});
      return '';
    }
    if (command === 'git' && args[0] === 'log') return 'Merge pull request #2 from owner/branch\n\nFix UI spacing\n\0\nFix status labels\n\0';
    throw Error(`Unexpected command: ${command} ${args}`);
  };
  return {run, calls};
}

test('generation uses paginated release metadata and first-parent source history across repositories', () => {
  const {run, calls} = runner({pages: [[published({...current, tag: 'nightly'}, '2026-10-19T00:00:00Z')], [published(previous, '2026-10-18T00:00:00Z')]]});
  const changes = generateChangelog(current, run);
  assert.match(changes.markdown, /Fix UI spacing/);
  assert.match(changes.markdown, /Fix status labels/);
  assert.ok(calls[0][1].includes('--paginate'));
  assert.ok(calls[1][1].includes(previous.tag));
  assert.ok(calls[1][1].includes('owner/downloads'));
  assert.deepEqual(calls[3], ['git', ['log', '--first-parent', '--format=%B%x00', `${previous.commit}..${current.commit}`, '--']]);
});

test('generation handles the initial release, rebuilds and rewritten or missing history', () => {
  assert.equal(generateChangelog(current, runner({pages: [[]]}).run).label, 'Initial release');
  const rebuild = runner({baseline: {...previous, commit: current.commit}});
  assert.equal(generateChangelog(current, rebuild.run).label, 'Routine rebuild');
  assert.equal(rebuild.calls.length, 2);
  for (const ancestryStatus of [1, 128]) {
    assert.equal(generateChangelog(current, runner({ancestryStatus}).run).label, 'Comparison unavailable');
  }
  assert.throws(() => generateChangelog(current, runner({ancestryStatus: 2}).run), /Cannot compare/);
});

test('API failures and incompatible metadata cannot silently produce misleading notes', () => {
  assert.throws(() => generateChangelog(current, runner({apiError: Error('Permission denied')}).run), /Permission denied/);
  for (const override of [{source_repository: 'other/source'}, {channel: 'stable'}, {commit: '--bad'}, {tag: 'nightly'}]) {
    assert.throws(() => generateChangelog(current, runner({baseline: {...previous, ...override}}).run), /incompatible/);
  }
  assert.throws(() => generateChangelog({...current, commit: '--bad'}, runner().run), /Invalid release identity/);
});

test('release pages lead with changes and retain installer, updater and firmware instructions', () => {
  const changes = formatChangelog(current, previous, ['Fix firmware confirmation spacing']);
  const notes = releaseNotes(current, changes, {updater: true});
  assert.ok(notes.startsWith('## What changed'));
  assert.match(notes, /Fix firmware confirmation spacing/);
  assert.match(notes, /Signed in-app updates/);
  assert.match(notes, /Firmware installation is separate/);
  assert.match(notes, /Apple Silicon/);
  assert.match(releaseNotes(current, changes, {updater: false}), /In-app updates are disabled/);
});
