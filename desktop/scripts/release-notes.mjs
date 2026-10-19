// Release notes compare source commits, even when downloads live in a separate repo.
const shaPattern = /^[0-9a-f]{40}$/;
const repoPattern = /^[\w.-]+\/[\w.-]+$/;
const tagPattern = /^(?:v\d+\.\d+\.\d+|nightly-\d+\.\d+\.\d+-nightly\.\d+\.\d+)$/;
const plainText = value => String(value).replace(/[\r\n\t]+/g, ' ').replace(/[\\`*_{}\[\]()<>#!|~]/g, '\\$&');

export function previousRelease(releases, current) {
  const published = releases.find(release => release.tag_name === current.tag && !release.draft)?.published_at;
  return releases.filter(release => !release.draft && release.tag_name !== current.tag &&
    tagPattern.test(release.tag_name) &&
    (current.channel === 'nightly' ? release.prerelease && release.tag_name.startsWith('nightly-') : !release.prerelease && release.tag_name.startsWith('v')) &&
    (!published || release.published_at < published))
    .sort((a, b) => b.published_at.localeCompare(a.published_at))[0] ?? null;
}

export function commitSummary(message) {
  const lines = message.trim().split('\n');
  // GitHub merge commits carry the useful PR title after the generic subject.
  return /^Merge pull request #\d+ /.test(lines[0]) ? lines.slice(1).find(line => line.trim())?.trim() || lines[0] : lines[0];
}

export function formatChangelog(current, previous, messages = [], comparable = true) {
  if (!previous) return {label: 'Initial release', markdown: `## What changed\n\n**Initial ${current.channel} release.** This is the first published release on this channel.\n`};
  const link = `[${plainText(previous.version)}](https://github.com/${current.repository}/releases/tag/${previous.tag})`;
  const heading = `## What changed\n\nCompared with ${link}.\n\n`;
  if (current.commit === previous.commit) return {
    label: 'Routine rebuild',
    markdown: `${heading}**Routine rebuild — no new source commits.** This rebuild uses the same application and firmware source as the previous release. Build tools or packaging may still differ.\n`,
  };
  if (!comparable) return {
    label: 'Comparison unavailable',
    markdown: `${heading}**Source revision changed; comparison unavailable.** The previous source revision is missing from this checkout or is no longer an ancestor. This release is not classified as a routine rebuild.\n`,
  };
  const entries = messages.map(commitSummary).filter(Boolean);
  const bullets = entries.slice(0, 100).map(message => `- ${plainText(message)}`).join('\n');
  const overflow = entries.length > 100 ? `\n\nPlus ${entries.length - 100} earlier changes in this release.` : '';
  return {
    label: 'New changes',
    markdown: `${heading}**New changes**\n\n${bullets || 'Source history changed, but no individual change summaries were found.'}${overflow}\n`,
  };
}

export function generateChangelog(current, run) {
  if (!shaPattern.test(current.commit) || !repoPattern.test(current.repository) || !repoPattern.test(current.source_repository) || !tagPattern.test(current.tag)) throw Error('Invalid release identity for changelog');
  // Filter out asset payloads before collecting pages, keeping long daily histories small.
  const history = run('gh', ['api', '--paginate', `repos/${current.repository}/releases?per_page=100`, '--jq', '.[] | {tag_name, draft, prerelease, published_at}']);
  const releases = history.split('\n').filter(line => line.trim()).map(line => JSON.parse(line));
  const release = previousRelease(releases, current);
  if (!release) return formatChangelog(current, null);
  const previous = JSON.parse(run('gh', ['release', 'download', release.tag_name, '--repo', current.repository, '--pattern', 'firmware.json', '--output', '-']));
  if (!shaPattern.test(previous.commit) || previous.source_repository !== current.source_repository || previous.repository !== current.repository || previous.channel !== current.channel || previous.tag !== release.tag_name || typeof previous.version !== 'string') throw Error('Previous release has incompatible source metadata');
  if (previous.commit === current.commit) return formatChangelog(current, previous);
  try {
    run('git', ['merge-base', '--is-ancestor', previous.commit, current.commit]);
  } catch (error) {
    if (![1, 128].includes(error.status)) throw error;
    return formatChangelog(current, previous, [], false);
  }
  const messages = run('git', ['log', '--first-parent', '--format=%B%x00', `${previous.commit}..${current.commit}`, '--']).split('\0').filter(message => message.trim());
  return formatChangelog(current, previous, messages);
}

export function releaseTitle(metadata, changes) {
  return `Teensy Panel ${metadata.version} — ${changes.label}`;
}

export function releaseNotes(metadata, changes, {updater, platformSigning = 'not configured; macOS Gatekeeper and Windows SmartScreen may require manual approval'} = {}) {
  return `${changes.markdown}\n## Downloads and installation\n\nWindows: download the x86_64 setup.exe. macOS: download the aarch64 (Apple Silicon) or x86_64 (Intel) DMG. The app includes matching Teensy 4.1 firmware and its flasher; no compiler or development tools are needed. Open “App & firmware” to flash, and press the Teensy Program button when prompted.\n\n${updater ? 'Signed in-app updates are enabled for this channel.' : 'In-app updates are disabled for this build; download a newer installer to update.'}\n\nPlatform signing: ${platformSigning}.\n\nFirmware installation is separate from updating the desktop app. Save experiment data and stop the rig before updating.\n\nSource commit: ${metadata.commit}\n`;
}
