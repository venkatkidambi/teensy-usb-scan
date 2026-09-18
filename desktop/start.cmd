@echo off
setlocal
set "DESKTOP_HOST="
for /f "tokens=2" %%H in ('rustc -vV ^| findstr /b "host:"') do set "DESKTOP_HOST=%%H"
if not defined DESKTOP_HOST (
  echo Install Rust with the Windows MSVC toolchain first.
  exit /b 1
)
rem Explicit native target overrides the parent project's Teensy Cargo target.
cargo run --manifest-path "%~dp0Cargo.toml" --target %DESKTOP_HOST% --release --locked
exit /b %ERRORLEVEL%
