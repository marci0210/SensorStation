<#
.SYNOPSIS
    Builds and flashes the SensorStation firmware, auto-detecting the ESP32's COM port.

.DESCRIPTION
    Finds the PlatformIO CLI, looks for a USB-serial adapter typically used on ESP32 boards
    (CH340/CH9102, CP210x, FTDI, Espressif native USB), then runs "pio run -t upload" on that port.

.PARAMETER Port
    COM port to use (e.g. COM3). Skips auto-detection.

.PARAMETER Environment
    PlatformIO environment from platformio.ini. Default: lolin32.

.PARAMETER Monitor
    Open the serial monitor after a successful upload (set DEBUG_MODE to true in inc/log.hpp to see output).

.PARAMETER Erase
    Erase the whole flash before uploading. This also deletes the saved Wi-Fi credentials
    and the BSEC calibration state.

.PARAMETER List
    Only list the serial ports and show which one would be used. Nothing is built or flashed.

.EXAMPLE
    .\flash.ps1
    .\flash.ps1 -Monitor
    .\flash.ps1 -Port COM5
    .\flash.ps1 -List
#>
[CmdletBinding()]
param(
    [string]$Port,
    [string]$Environment = 'lolin32',
    [switch]$Monitor,
    [switch]$Erase,
    [switch]$List
)

$ErrorActionPreference = 'Stop'

# USB vendor IDs of the serial chips found on ESP32 boards
$KnownChips = @{
    '1A86' = 'WCH CH340 / CH9102'
    '10C4' = 'Silicon Labs CP210x'
    '0403' = 'FTDI'
    '303A' = 'Espressif native USB'
}

function Write-Step([string]$Text) { Write-Host "==> $Text" -ForegroundColor Cyan }
function Write-Problem([string]$Text) { Write-Host "ERROR: $Text" -ForegroundColor Red }

function Invoke-Pio {
    # Windows PowerShell 5.1 turns a native tool's stderr (compiler warnings, esptool progress) into
    # error records when output is redirected; with 'Stop' that would abort the script. Merge both
    # streams, print them as plain text, and let only the exit code decide success.
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $script:Pio @args 2>&1 | ForEach-Object { Write-Host "$_" }
        return $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previous
    }
}

function Find-Pio {
    $cmd = Get-Command pio -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $default = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
    if (Test-Path $default) { return $default }

    return $null
}

function Get-SerialPorts {
    # Every device that owns a COM port, with its USB vendor ID (if any)
    Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match '\((COM\d+)\)' } |
        ForEach-Object {
            $null = $_.Name -match '\((COM\d+)\)'
            $com = $Matches[1]
            $vid = $null
            if ($_.PNPDeviceID -match 'VID_([0-9A-F]{4})') { $vid = $Matches[1] }
            $chip = $null
            if ($vid -and $KnownChips.ContainsKey($vid)) { $chip = $KnownChips[$vid] }

            [PSCustomObject]@{
                Port        = $com
                Number      = [int]($com -replace 'COM', '')
                Description = $_.Name
                Chip        = $chip
                Status      = $_.Status
            }
        } |
        Sort-Object Number
}

function Select-EspPort {
    $all = @(Get-SerialPorts)
    $candidates = @($all | Where-Object { $_.Chip })

    if ($candidates.Count -eq 0) {
        Write-Problem 'No ESP32 board found.'
        if ($all.Count -gt 0) {
            Write-Host 'Serial ports present (none look like an ESP32 USB adapter):'
            $all | ForEach-Object { Write-Host ("  {0,-6} {1}" -f $_.Port, $_.Description) }
        }
        Write-Host 'Check the USB cable (some are charge-only) and the CH340/CP210x driver, or pass -Port COMx.'
        return $null
    }

    if ($candidates.Count -eq 1) {
        return $candidates[0].Port
    }

    Write-Host 'More than one possible board found:'
    for ($i = 0; $i -lt $candidates.Count; $i++) {
        Write-Host ("  [{0}] {1,-6} {2}" -f ($i + 1), $candidates[$i].Port, $candidates[$i].Description)
    }
    if (-not [Environment]::UserInteractive) {
        Write-Problem 'Pass the port explicitly with -Port COMx.'
        return $null
    }
    $choice = Read-Host "Select a board (1-$($candidates.Count))"
    $index = 0
    if ([int]::TryParse($choice, [ref]$index) -and $index -ge 1 -and $index -le $candidates.Count) {
        return $candidates[$index - 1].Port
    }
    Write-Problem 'Invalid selection.'
    return $null
}

# ---------------------------------------------------------------------------

Push-Location $PSScriptRoot
try {
    if ($List) {
        $ports = @(Get-SerialPorts)
        if ($ports.Count -eq 0) {
            Write-Host 'No serial ports found.'
        }
        else {
            $ports | Format-Table Port, Description, @{ Label = 'ESP32 adapter'; Expression = { if ($_.Chip) { $_.Chip } else { '-' } } }, Status -AutoSize
        }
        $candidates = @($ports | Where-Object { $_.Chip })
        if ($candidates.Count -eq 1) { Write-Host "Would flash to: $($candidates[0].Port)" -ForegroundColor Green }
        elseif ($candidates.Count -gt 1) { Write-Host 'Several candidates: you will be asked to choose, or use -Port.' }
        exit 0
    }

    $script:Pio = Find-Pio
    if (-not $script:Pio) {
        Write-Problem 'PlatformIO CLI (pio) not found. Install the PlatformIO VS Code extension or "pip install platformio".'
        exit 1
    }

    # Persoboard device credentials (the build fails without them)
    $deviceLine = Select-String -Path 'platformio.ini' -Pattern '^\s*custom_pb_device\s*=\s*(\S+)' | Select-Object -First 1
    if ($deviceLine) {
        $device = $deviceLine.Matches[0].Groups[1].Value
        $secrets = "certs\devices\$device\secrets.h"
        if (-not (Test-Path $secrets)) {
            Write-Problem "$secrets not found."
            Write-Host "Generate it on the Persoboard side (python scripts/pki.py device $device) and copy it here,"
            Write-Host 'or change custom_pb_device in platformio.ini. See README.md > Configuration.'
            exit 1
        }
        Write-Step "Device credentials: $secrets"
    }

    if (-not (Test-Path 'inc\config.hpp')) {
        Write-Host 'WARNING: inc\config.hpp not found. Building with an OPEN setup portal.' -ForegroundColor Yellow
        Write-Host '         Set the ingest server in the portal (Setup page). See README.md > Configuration.' -ForegroundColor Yellow
    }
    elseif (-not (Select-String -Path 'inc\config.hpp' -Pattern '^\s*#define\s+PORTAL_PASSWORD' -Quiet)) {
        Write-Host 'WARNING: PORTAL_PASSWORD is not defined in inc\config.hpp, so the setup portal is an OPEN network.' -ForegroundColor Yellow
    }

    if (-not $Port) {
        Write-Step 'Looking for the ESP32...'
        $Port = Select-EspPort
        if (-not $Port) { exit 1 }
    }
    $Port = $Port.ToUpper()
    Write-Step "Using $Port (environment: $Environment)"

    if ($Erase) {
        Write-Step 'Erasing flash (Wi-Fi credentials and BSEC calibration will be lost)...'
        $code = Invoke-Pio run -e $Environment -t erase --upload-port $Port
        if ($code -ne 0) { Write-Problem 'Erase failed.'; exit $code }
    }

    Write-Step 'Building and uploading...'
    $code = Invoke-Pio run -e $Environment -t upload --upload-port $Port
    if ($code -ne 0) {
        Write-Problem "Upload failed (exit code $code)."
        Write-Host 'Things to try:'
        Write-Host '  - Close any serial monitor that has the port open (VS Code, Arduino IDE, PuTTY).'
        Write-Host '  - Hold the BOOT button on the board while "Connecting..." is shown.'
        Write-Host '  - Lower upload_speed in platformio.ini (e.g. 460800 or 115200).'
        exit $code
    }
    Write-Host "Firmware flashed to $Port." -ForegroundColor Green

    if ($Monitor) {
        Write-Step 'Opening serial monitor (Ctrl+C to exit). Output requires DEBUG_MODE true in inc/log.hpp.'
        $ErrorActionPreference = 'Continue'
        & $script:Pio device monitor -e $Environment -p $Port   # interactive: keep it attached to the console
    }
}
finally {
    Pop-Location
}
