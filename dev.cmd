@echo off
setlocal
rem Use the batch launcher so PowerShell never needs to execute npm.ps1.
where npm.cmd >nul 2>&1
if errorlevel 1 (
  echo Install Node.js 22.12 or newer, then reopen your terminal.
  exit /b 1
)
pushd "%~dp0"
if errorlevel 1 exit /b 1

rem An incremental install also picks up dependency changes after a git pull.
call npm.cmd install --no-audit --no-fund
if errorlevel 1 goto finish
call npm.cmd run dev -- %*

:finish
set "DEV_EXIT_CODE=%ERRORLEVEL%"
popd
exit /b %DEV_EXIT_CODE%
