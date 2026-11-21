[CmdletBinding()]
param(
    [switch]$IncludeDisconnected,
    [switch]$Json,
    [switch]$Diff
)

$ErrorActionPreference = "Stop"

function Resolve-AdapterTag {
    param([string]$InstanceId, [string]$FriendlyName)
    if ($InstanceId -match "^BTHENUM\\") {
        return "Bluetooth Serial"
    }
    if ($InstanceId -match "VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})") {
        $vid = $Matches[1].ToUpper()
        $pidVal = $Matches[2].ToUpper()
        $vp = "${vid}:${pidVal}"
        switch ($vp) {
            "1A86:7523" { return "CH340 (Feetech/SO-101)" }
            "1A86:55D3" { return "CH343 (USB-Serial)" }
            "0483:5740" { return "STM32 USB CDC" }
            "0483:5711" { return "STM32 USB CDC" }
            "10C4:EA60" { return "CP210x" }
            default {
                if ($vid -eq "0403") { return "FTDI" }
                return "USB Serial ($vp)"
            }
        }
    }
    if ($FriendlyName -match "CH340") { return "CH340 (Feetech/SO-101)" }
    return "Serial Port"
}

function Get-HardwareSnapshot {
    $activeComs = @()
    try {
        $activeComs = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique)
    } catch {
        $activeComs = @()
    }

    $ports = @()
    $pnpPorts = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue
    if ($pnpPorts) {
        foreach ($dev in $pnpPorts) {
            $com = $null
            if ($dev.FriendlyName -match "\((COM\d+)\)") {
                $com = $Matches[1]
            }
            $vid = $null
            $pidVal = $null
            if ($dev.InstanceId -match "VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})") {
                $vid = $Matches[1].ToUpper()
                $pidVal = $Matches[2].ToUpper()
            }
            $isActive = ($dev.Status -eq "OK") -or ($com -and ($activeComs -contains $com))
            if (-not $IncludeDisconnected -and -not $isActive) {
                continue
            }
            $ports += [PSCustomObject]@{
                port        = $com
                status      = $dev.Status
                active      = [bool]$isActive
                tag         = Resolve-AdapterTag -InstanceId $dev.InstanceId -FriendlyName $dev.FriendlyName
                friendly    = $dev.FriendlyName
                vid         = $vid
                pid         = $pidVal
                instance_id = $dev.InstanceId
            }
        }
    }

    # Include any active .NET COM names not matched by PnP Ports
    $knownPorts = @($ports | Where-Object { $_.port } | ForEach-Object { $_.port })
    foreach ($c in $activeComs) {
        if ($knownPorts -notcontains $c) {
            $ports += [PSCustomObject]@{
                port        = $c
                status      = "OK"
                active      = $true
                tag         = "Active Serial (.NET)"
                friendly    = $c
                vid         = $null
                pid         = $null
                instance_id = $null
            }
        }
    }

    $cameras = @()
    $pnpCams = Get-PnpDevice -Class Camera,Image -ErrorAction SilentlyContinue
    if ($pnpCams) {
        foreach ($cam in $pnpCams) {
            if (-not $IncludeDisconnected -and $cam.Status -ne "OK") {
                continue
            }
            $vid = $null
            $pidVal = $null
            if ($cam.InstanceId -match "VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})") {
                $vid = $Matches[1].ToUpper()
                $pidVal = $Matches[2].ToUpper()
            }
            $cameras += [PSCustomObject]@{
                name        = $cam.FriendlyName
                class       = $cam.Class
                status      = $cam.Status
                vid         = $vid
                pid         = $pidVal
                instance_id = $cam.InstanceId
            }
        }
    }

    return [PSCustomObject]@{
        active_com_names = $activeComs
        serial_ports     = $ports
        cameras          = $cameras
    }
}

if ($Diff) {
    $before = Get-HardwareSnapshot
    $beforePorts = @($before.serial_ports | Where-Object { $_.active -and $_.port } | ForEach-Object { $_.port })
    Write-Host "Before ports: $($beforePorts -join ', ')" -ForegroundColor Cyan
    [void](Read-Host "Plug or unplug the arm101 USB cable, then press Enter")
    $after = Get-HardwareSnapshot
    $afterPorts = @($after.serial_ports | Where-Object { $_.active -and $_.port } | ForEach-Object { $_.port })
    $added = @($afterPorts | Where-Object { $beforePorts -notcontains $_ })
    $removed = @($beforePorts | Where-Object { $afterPorts -notcontains $_ })
    if ($Json) {
        [PSCustomObject]@{
            before  = $beforePorts
            after   = $afterPorts
            added   = $added
            removed = $removed
        } | ConvertTo-Json -Depth 4
    } else {
        Write-Host "Added ports  : $(if ($added) { $added -join ', ' } else { 'None' })" -ForegroundColor Green
        Write-Host "Removed ports: $(if ($removed) { $removed -join ', ' } else { 'None' })" -ForegroundColor Yellow
    }
    exit 0
}

$snapshot = Get-HardwareSnapshot

if ($Json) {
    $snapshot | ConvertTo-Json -Depth 5
    exit 0
}

Write-Host "=== Windows Serial Ports (arm101 / SO-101) ===" -ForegroundColor Yellow
if ($snapshot.serial_ports.Count -eq 0) {
    Write-Host "No matching serial ports found. Pass -IncludeDisconnected to view cached USB-serial ports."
} else {
    Write-Output ($snapshot.serial_ports | Select-Object port, status, tag, friendly | Format-Table -AutoSize | Out-String).TrimEnd()
}

Write-Host ""
Write-Host "=== Windows Cameras / Imaging Devices ===" -ForegroundColor Yellow
if ($snapshot.cameras.Count -eq 0) {
    Write-Host "No active cameras found."
} else {
    Write-Output ($snapshot.cameras | Select-Object name, class, status, vid, pid | Format-Table -AutoSize | Out-String).TrimEnd()
}
