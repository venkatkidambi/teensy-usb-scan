#Requires -Version 5.1
[CmdletBinding()]
param(
    # An executable on PATH, or the full path to teensy_loader_cli.exe.
    [string]$TeensyLoader
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# An explicit argument wins over the project configuration, then PATH.
if (-not $PSBoundParameters.ContainsKey('TeensyLoader')) {
    $configPath = Join-Path $PSScriptRoot 'upload.config.json'
    if (Test-Path -LiteralPath $configPath -PathType Leaf) {
        $config = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
        if ($null -eq $config -or
            $null -eq $config.PSObject.Properties['TeensyLoader'] -or
            $config.TeensyLoader -isnot [string] -or
            [string]::IsNullOrWhiteSpace($config.TeensyLoader)) {
            throw "Expected a nonempty TeensyLoader string in '$configPath'."
        }
        $TeensyLoader = $config.TeensyLoader
    }
    else {
        $TeensyLoader = 'teensy_loader_cli.exe'
    }
}

function Get-RequiredApplication {
    param([string]$Name, [string]$Hint)

    $command = Get-Command -Name $Name -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -eq $command) {
        throw "Cannot find '$Name'. $Hint"
    }
    return $command.Source
}

$cargo = Get-RequiredApplication 'cargo' 'Install Rust/Cargo and add it to PATH.'
$null = Get-RequiredApplication 'cargo-objcopy' 'Run: cargo install cargo-binutils --locked'
$loader = Get-RequiredApplication $TeensyLoader 'Install teensy_loader_cli.exe and add it to PATH, or pass -TeensyLoader with its full path. The graphical Teensy Loader is a different program.'

# Cargo discovers configuration relative to the working directory.
Push-Location -LiteralPath $PSScriptRoot
try {
    & $cargo build --release --locked
    # Windows PowerShell does not automatically throw on native command failure.
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed (exit code $LASTEXITCODE)."
    }

    $firmwareDir = Join-Path $PSScriptRoot 'target/firmware'
    $null = New-Item -ItemType Directory -Path $firmwareDir -Force
    $hex = Join-Path $firmwareDir 'firmware.hex'

    # cargo-objcopy discovers the sole binary's ELF, including when
    # CARGO_TARGET_DIR is set. No Cargo artifact name or ELF path is assumed.
    & $cargo objcopy --release --locked -- -O ihex $hex
    if ($LASTEXITCODE -ne 0) {
        throw "ELF-to-HEX conversion failed (exit code $LASTEXITCODE)."
    }
    if (-not (Test-Path -LiteralPath $hex -PathType Leaf)) {
        throw "HEX conversion did not produce '$hex'."
    }

    Write-Host 'Press the Teensy Program button if the loader waits for the board.'
    & $loader '--mcu=TEENSY41' '-w' '-v' $hex
    if ($LASTEXITCODE -ne 0) {
        throw "Teensy upload failed (exit code $LASTEXITCODE)."
    }
}
finally {
    Pop-Location
}
