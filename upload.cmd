@echo off
setlocal
rem Execution policy applies only to this PowerShell process.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0upload.ps1" %*
exit /b %ERRORLEVEL%
