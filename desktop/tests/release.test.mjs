import test from 'node:test';
import assert from 'node:assert/strict';
import {releaseMetadata, packageConfig, updateManifest} from '../scripts/release.mjs';

const input = {ref: 'refs/heads/main', sha: 'a'.repeat(40), baseVersion: '0.2.0', runNumber: '42', attempt: '1', repository: 'owner/source'};
const nightly = () => ({...releaseMetadata(input), downloads_public: true});

test('stable tags must match the source version; nightlies identify each run and attempt', () => {
  assert.equal(releaseMetadata({...input, ref: 'refs/tags/v0.2.0'}).version, '0.2.0');
  assert.equal(nightly().version, '0.2.0-nightly.42.1');
  assert.equal(releaseMetadata({...input, attempt: '2'}).version, '0.2.0-nightly.42.2');
  assert.throws(() => releaseMetadata({...input, ref: 'refs/tags/v0.3.0'}), /must match/);
  assert.throws(() => releaseMetadata({...input, ref: 'refs/heads/feature'}), /main/);
  assert.throws(() => releaseMetadata({...input, runNumber: 'bad'}), /run number/);
  assert.throws(() => releaseMetadata({...input, sha: 'untrusted\nvalue'}), /commit SHA/);
});

test('stable and nightly install separately and use different update feeds', () => {
  const options = {target: 'x86_64-pc-windows-msvc', publicKey: 'public', signingKey: 'private'};
  const dev = packageConfig(nightly(), options);
  const stable = packageConfig({...nightly(), channel: 'stable'}, options);
  assert.notEqual(dev.identifier, stable.identifier);
  assert.notEqual(dev.productName, stable.productName);
  assert.match(dev.plugins.updater.endpoints[0], /download\/nightly\/latest.json$/);
  assert.match(stable.plugins.updater.endpoints[0], /latest\/download\/latest.json$/);
  assert.equal(dev.bundle.windows.webviewInstallMode.type, 'offlineInstaller');
  assert.equal(dev.bundle.windows.nsis.installMode, 'currentUser');
});

test('private downloads cannot enable an unauthenticated updater and partial signing config fails', () => {
  const options = {target: 'aarch64-apple-darwin'};
  assert.equal(packageConfig(nightly(), options).bundle.createUpdaterArtifacts, false);
  assert.throws(() => packageConfig(nightly(), {...options, publicKey: 'public'}), /both/);
  const config = packageConfig({...nightly(), downloads_public: false}, {...options, publicKey: 'public', signingKey: 'private'});
  assert.equal(config.bundle.createUpdaterArtifacts, false);
  assert.equal(config.plugins, undefined);
  assert.throws(() => packageConfig(nightly(), {target: 'unknown'}), /Unsupported/);
});

test('update publication requires all three platforms from the same source and firmware', () => {
  const builds = ['windows-x86_64', 'darwin-x86_64', 'darwin-aarch64'].map(platform => ({...nightly(), platform, updater: true, firmware_sha256: 'b'.repeat(64), update: {signature: 'signed', url: `https://example.com/${platform}`}}));
  assert.equal(Object.keys(updateManifest(builds).platforms).length, 3);
  assert.throws(() => updateManifest(builds.slice(1)), /three platform/);
  assert.throws(() => updateManifest([builds[0], builds[0], builds[2]]), /three platform/);
  assert.throws(() => updateManifest(builds.map((b,i) => i === 1 ? {...b, commit: 'c'.repeat(40)} : b)), /same build/);
  assert.throws(() => updateManifest(builds.map((b,i) => i === 1 ? {...b, firmware_sha256: 'different'} : b)), /same build/);
  assert.throws(() => updateManifest(builds.map((b,i) => i === 1 ? {...b, update: undefined} : b)), /signed/);
  assert.equal(updateManifest(builds.map(b => ({...b, updater: false}))), null);
});
