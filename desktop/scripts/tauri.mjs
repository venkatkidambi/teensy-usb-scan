import { spawn, execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

// The parent Cargo configuration targets the Teensy; always select this PC.
const host = execFileSync('rustc', ['-vV'], { encoding: 'utf8' }).match(/^host: (.+)$/m)?.[1];
if (!host) throw new Error('Could not determine the Rust host target');
const mode = process.argv[2];
if (!['dev', 'build'].includes(mode)) throw new Error('Expected dev or build');
const cli = fileURLToPath(new URL('../../node_modules/@tauri-apps/cli/tauri.js', import.meta.url));
const args = process.argv.slice(3);
const target = args.some(arg => arg === '--target' || arg.startsWith('--target=')) ? [] : ['--target', host];
const child = spawn(process.execPath, [cli, mode, ...target,
  ...(mode === 'dev' ? [] : ['--config', JSON.stringify({ build: { frontendDist: 'dist' } })]), ...args], {
  cwd: fileURLToPath(new URL('../', import.meta.url)), stdio: 'inherit',
});
child.on('error', error => { console.error(error.message); process.exitCode = 1; });
child.on('exit', code => { process.exitCode = code ?? 1; });
