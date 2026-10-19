// Release preparation is intentionally independent of developer PATH/config files.
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {readFileSync, writeFileSync, mkdirSync, copyFileSync, readdirSync, existsSync} from 'node:fs';
import {resolve, join, basename} from 'node:path';
import {fileURLToPath} from 'node:url';

export const root = fileURLToPath(new URL('../../', import.meta.url));
export const loaderCommit = '03fca4156c244c7ad36bd368cf6e24531dbd566a';
const readJSON = path => JSON.parse(readFileSync(path, 'utf8'));
const json = (path, value) => writeFileSync(path, `${JSON.stringify(value, null, 2)}\n`);
export const sha256 = data => createHash('sha256').update(data).digest('hex');
const run = (command, args, options = {}) => execFileSync(command, args, {cwd: root, encoding: 'utf8', ...options}).trim();

export function releaseMetadata({ref, sha, baseVersion, runNumber, attempt = '1', repository}) {
  if (!/^[0-9a-f]{40}$/.test(sha)) throw Error('A full source commit SHA is required');
  if (!/^\d+\.\d+\.\d+$/.test(baseVersion)) throw Error('The base app version must be major.minor.patch');
  if (!/^[\w.-]+\/[\w.-]+$/.test(repository)) throw Error('Invalid GitHub repository');
  const stable = ref.startsWith('refs/tags/v');
  if (stable && ref !== `refs/tags/v${baseVersion}`) throw Error('Stable tag must match desktop/tauri.conf.json version');
  if (!stable && ref !== 'refs/heads/main') throw Error('Nightlies must be built from main');
  if (!stable && (!/^[1-9]\d*$/.test(runNumber) || !/^[1-9]\d*$/.test(attempt))) throw Error('Invalid nightly run number');
  const version = stable ? baseVersion : `${baseVersion}-nightly.${runNumber}.${attempt}`;
  return {
    version, channel: stable ? 'stable' : 'nightly', commit: sha, repository,
    tag: stable ? `v${version}` : `nightly-${version}`,
    loader_commit: loaderCommit,
  };
}

export function packageConfig(metadata, {target, publicKey = '', signingKey = ''}) {
  const windows = target === 'x86_64-pc-windows-msvc';
  if (!windows && !['aarch64-apple-darwin', 'x86_64-apple-darwin'].includes(target)) throw Error('Unsupported release target');
  if (Boolean(publicKey) !== Boolean(signingKey)) throw Error('Configure both TAURI_SIGNING_PUBLIC_KEY and TAURI_SIGNING_PRIVATE_KEY, or neither');
  // A desktop installation cannot authenticate to private GitHub release URLs.
  // Never put a repository token in a distributed application.
  publicKey = metadata.downloads_public ? publicKey : '';
  const nightly = metadata.channel === 'nightly';
  const downloads = `https://github.com/${metadata.repository}/releases`;
  return {
    version: metadata.version,
    productName: nightly ? 'Teensy Panel Nightly' : 'Teensy Panel',
    identifier: nightly ? 'local.teensy.instrument-panel.nightly' : 'local.teensy.instrument-panel',
    build: {frontendDist: 'dist'},
    app: {windows: [{label: 'main', title: nightly ? 'Teensy Panel — Nightly' : 'Teensy Panel', width: 1100, height: 860, minWidth: 780, minHeight: 650, backgroundColor: '#fafafa'}]},
    bundle: {
      active: true,
      targets: windows ? ['nsis'] : ['app', 'dmg'],
      createUpdaterArtifacts: Boolean(publicKey),
      externalBin: ['binaries/teensy_loader_cli'],
      resources: {'resources/': 'release/'},
      windows: {webviewInstallMode: {type: 'offlineInstaller'}, nsis: {installMode: 'currentUser'}},
      macOS: {minimumSystemVersion: '12.0'},
    },
    ...(publicKey ? {plugins: {updater: {pubkey: publicKey.trim(), endpoints: [nightly ? `${downloads}/download/nightly/latest.json` : `${downloads}/latest/download/latest.json`], windows: {installMode: 'passive'}}}} : {}),
  };
}

function walk(directory) {
  return readdirSync(directory, {withFileTypes: true}).flatMap(entry => {
    const path = join(directory, entry.name);
    return entry.isDirectory() ? walk(path) : [path];
  });
}

function prepareMetadata() {
  const metadata = releaseMetadata({
    ref: process.env.BUILD_ONLY === 'true' ? 'refs/heads/main' : process.env.GITHUB_REF, sha: process.env.GITHUB_SHA,
    baseVersion: readJSON(join(root, 'desktop/tauri.conf.json')).version,
    runNumber: process.env.GITHUB_RUN_NUMBER, attempt: process.env.GITHUB_RUN_ATTEMPT,
    repository: process.env.GITHUB_REPOSITORY,
  });
  metadata.source_repository = metadata.repository;
  metadata.repository = process.env.RELEASE_REPOSITORY || metadata.repository;
  if (!/^[\w.-]+\/[\w.-]+$/.test(metadata.repository)) throw Error('Invalid release repository');
  metadata.downloads_public = run('gh', ['repo', 'view', metadata.repository, '--json', 'visibility', '--jq', '.visibility']) === 'PUBLIC';
  const canPublish = process.env.BUILD_ONLY !== 'true' && (metadata.repository === metadata.source_repository || process.env.CROSS_REPO_PUBLISH === 'true');
  if (!canPublish && process.env.BUILD_ONLY !== 'true') console.log('::warning::RELEASE_TOKEN is not configured. Installers will be available as Actions artifacts; public publication is skipped.');
  mkdirSync(join(root, 'release-input'), {recursive: true});
  json(join(root, 'release-input/build.json'), metadata);
  if (process.env.GITHUB_OUTPUT) writeFileSync(process.env.GITHUB_OUTPUT,
    `version=${metadata.version}\nchannel=${metadata.channel}\ntag=${metadata.tag}\ncan_publish=${canPublish}\n`, {flag: 'a'});
}

function prepareFirmware() {
  const metadata = readJSON(join(root, 'release-input/build.json'));
  const hex = readFileSync(join(root, 'target/firmware/firmware.hex'));
  copyFileSync(join(root, 'target/firmware/firmware.hex'), join(root, 'release-input/firmware.hex'));
  json(join(root, 'release-input/firmware.json'), {...metadata, firmware_sha256: sha256(hex)});
}

function preparePackage(target) {
  const metadata = readJSON(join(root, 'release-input/firmware.json'));
  if (metadata.commit !== process.env.GITHUB_SHA) throw Error('Firmware and desktop commits differ');
  const firmware = readFileSync(join(root, 'release-input/firmware.hex'));
  if (sha256(firmware) !== metadata.firmware_sha256) throw Error('Firmware hash mismatch');
  const config = packageConfig(metadata, {target, publicKey: process.env.TAURI_SIGNING_PUBLIC_KEY, signingKey: process.env.TAURI_SIGNING_PRIVATE_KEY});
  const resources = join(root, 'desktop/resources');
  mkdirSync(resources, {recursive: true});
  mkdirSync(join(root, 'desktop/binaries'), {recursive: true});
  const source = join(root, 'loader-source');
  if (run('git', ['rev-parse', 'HEAD'], {cwd: source}) !== loaderCommit) throw Error('Unexpected Teensy loader source revision');
  const exe = target.includes('windows') ? '.exe' : '';
  copyFileSync(join(source, `teensy_loader_cli${exe}`), join(root, `desktop/binaries/teensy_loader_cli-${target}${exe}`));
  writeFileSync(join(resources, 'firmware.hex'), firmware);
  // Ship the exact corresponding source and license alongside the separate GPL loader.
  run(process.platform === 'win32' ? 'python' : 'python3', [join(root, 'desktop/scripts/archive-loader.py'), source, join(resources, 'teensy-loader-source.zip'), join(root, 'docs/RELEASING.md')]);
  copyFileSync(join(source, 'gpl3.txt'), join(resources, 'teensy-loader-GPL-3.0.txt'));
  copyFileSync(join(root, 'docs/RELEASING.md'), join(resources, 'RELEASING.md'));
  const updater = config.bundle.createUpdaterArtifacts;
  json(join(resources, 'release.json'), {...metadata, target, updater});
  json(join(root, 'desktop/tauri.release.json'), config);
}

function collect(target) {
  const metadata = readJSON(join(root, 'desktop/resources/release.json'));
  const bundle = join(root, `desktop/target/${target}/release/bundle`);
  const files = walk(bundle).filter(path => /\.(dmg|exe|app\.tar\.gz)(\.sig)?$/.test(path) && !path.includes('.app/'));
  const out = join(root, 'release-out');
  mkdirSync(out, {recursive: true});
  const assets = [];
  for (const path of files) {
    // Keep each build's filenames unique, including both Mac architectures.
    const name = `${target}-${basename(path).replaceAll(' ', '-')}`;
    copyFileSync(path, join(out, name));
    assets.push(name);
  }
  const windows = target.includes('windows');
  if (!assets.some(name => name.endsWith(windows ? '.exe' : '.dmg'))) throw Error('Installer missing');
  const updateAsset = assets.find(name => name.endsWith(windows ? '.exe' : '.app.tar.gz'));
  if (metadata.updater && (!updateAsset || !assets.includes(`${updateAsset}.sig`))) throw Error('Signed updater artifact missing');
  json(join(out, `${target}.json`), {
    ...metadata, assets,
    platform: windows ? 'windows-x86_64' : `darwin-${target.startsWith('aarch64') ? 'aarch64' : 'x86_64'}`,
    ...(metadata.updater ? {update: {signature: readFileSync(join(out, `${updateAsset}.sig`), 'utf8').trim(), url: `https://github.com/${metadata.repository}/releases/download/${metadata.tag}/${updateAsset}`}} : {}),
  });
}

export function updateManifest(builds) {
  if (builds.length !== 3 || new Set(builds.map(b => b.platform)).size !== 3) throw Error('All three platform builds are required');
  const first = builds[0];
  if (builds.some(b => ['version', 'commit', 'channel', 'tag', 'repository', 'source_repository', 'firmware_sha256', 'updater'].some(key => b[key] !== first[key]))) throw Error('Release artifacts do not belong to the same build');
  if (!first.updater) return null;
  if (builds.some(b => !b.update?.signature || !b.update?.url)) throw Error('Every update must be signed');
  return {version: first.version, notes: `${first.channel} release · commit ${first.commit}. Update the Teensy firmware separately from the app.`, pub_date: new Date().toISOString(), platforms: Object.fromEntries(builds.map(b => [b.platform, b.update]))};
}

function publish() {
  const out = join(root, 'release-out');
  const builds = readdirSync(out).filter(name => /^(aarch64-apple-darwin|x86_64-apple-darwin|x86_64-pc-windows-msvc)\.json$/.test(name)).map(name => readJSON(join(out, name)));
  const manifest = updateManifest(builds);
  const first = builds[0];
  const gh = args => run('gh', [...args, '--repo', first.repository]);
  if (first.channel === 'nightly') {
    const main = run('gh', ['api', `repos/${first.source_repository}/commits/main`, '--jq', '.sha'], {env: {...process.env, GH_TOKEN: process.env.SOURCE_GH_TOKEN || process.env.GH_TOKEN}});
    if (main !== first.commit) { console.log('A newer main commit exists; skipping this obsolete nightly publication.'); return; }
  }
  if (manifest) json(join(out, 'latest.json'), manifest);
  copyFileSync(join(root, 'release-input/firmware.hex'), join(out, 'firmware.hex'));
  copyFileSync(join(root, 'release-input/firmware.json'), join(out, 'firmware.json'));
  const assets = readdirSync(out).filter(name => name !== 'SHA256SUMS.txt' && name !== 'release-notes.md');
  writeFileSync(join(out, 'SHA256SUMS.txt'), assets.map(name => `${sha256(readFileSync(join(out, name)))}  ${name}\n`).join(''));
  const notes = `${first.channel === 'nightly' ? 'Nightly development build' : 'Stable release'} ${first.version}\n\nSource commit: ${first.commit}\n\nWindows: download the x86_64 setup.exe. macOS: download the aarch64 (Apple Silicon) or x86_64 (Intel) DMG. The app includes matching Teensy 4.1 firmware and its flasher; no compiler or development tools are needed. Open “App & firmware” to flash, and press the Teensy Program button when prompted.\n\n${manifest ? 'Signed in-app updates are enabled for this channel.' : 'In-app updates are disabled for this build; download a newer installer to update.'}\n\nPlatform signing: ${process.env.RELEASE_PLATFORM_SIGNING || 'not configured; macOS Gatekeeper and Windows SmartScreen may require manual approval'}.\n\nFirmware installation is separate from updating the desktop app. Save experiment data and stop the rig before updating.\n`;
  writeFileSync(join(out, 'release-notes.md'), notes);
  // Publish only after all builds pass. Stable releases are immutable; a rerun
  // may finish an existing draft, but never overwrite a published stable version.
  let existing;
  try { existing = JSON.parse(gh(['release', 'view', first.tag, '--json', 'isDraft'])); } catch {}
  if (existing && !existing.isDraft) throw Error(`Release ${first.tag} already exists; use a new version/run`);
  const sameRepo = first.repository === first.source_repository;
  const targetArgs = sameRepo ? ['--target', first.commit] : [];
  if (!existing) gh(['release', 'create', first.tag, ...targetArgs, '--draft', '--title', `Teensy Panel ${first.version}`, '--notes-file', join(out, 'release-notes.md'), ...(first.channel === 'nightly' ? ['--prerelease'] : sameRepo ? ['--verify-tag'] : [])]);
  gh(['release', 'upload', first.tag, ...assets.map(name => join(out, name)), join(out, 'SHA256SUMS.txt'), '--clobber']);
  gh(['release', 'edit', first.tag, '--draft=false', `--latest=${first.channel === 'stable'}`, '--notes-file', join(out, 'release-notes.md')]);
  if (first.channel === 'nightly' && manifest) {
    // Only the small channel pointer is mutable. Its URLs refer to this complete,
    // immutable nightly release, never a mixture of files from two runs.
    try { gh(['release', 'view', 'nightly']); }
    catch { gh(['release', 'create', 'nightly', ...targetArgs, '--prerelease', '--latest=false', '--title', 'Nightly update channel', '--notes', 'Updater manifest for the newest complete nightly. Installers are in the versioned nightly releases.']); }
    gh(['release', 'upload', 'nightly', join(out, 'latest.json'), '--clobber']);
  }
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const [command, target] = process.argv.slice(2);
  ({metadata: prepareMetadata, firmware: prepareFirmware, package: () => preparePackage(target), collect: () => collect(target), publish}[command] ?? (() => {throw Error('Expected metadata, firmware, package TARGET, collect TARGET, or publish');}))();
}
