[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [switch]$DryRun,
    [switch]$SkipLerobotInstall,
    [switch]$WithCoreScripts,
    [switch]$WithLeLab,
    [switch]$WithPlaywrightBrowser,
    [string]$TorchBackend = ""
)

$ErrorActionPreference = "Stop"

$WorkspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$VenvDir = Join-Path $WorkspaceRoot "envs\win\lerobot"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"
$LerobotRepo = Join-Path $WorkspaceRoot "repos\lerobot"
$LeLabRepo = Join-Path $WorkspaceRoot "repos\leLab"
$CheckEnvScript = Join-Path $WorkspaceRoot "scripts\common\check_env.py"

function Get-ToolPath {
    param([string]$Name)
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

function Invoke-Step {
    param(
        [string]$Description,
        [string]$Exe,
        [string[]]$ArgList
    )
    $displayArgs = ($ArgList | ForEach-Object { if ($_ -match "\s") { "`"$_`"" } else { $_ } }) -join " "
    if ($DryRun) {
        Write-Host "[DRY-RUN] $Description" -ForegroundColor Cyan
        Write-Host "          $Exe $displayArgs"
        return
    }
    Write-Host "[RUN] $Description" -ForegroundColor Green
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed (exit $LASTEXITCODE): $Exe $displayArgs"
    }
}

$uvPath = Get-ToolPath "uv"
$gitPath = Get-ToolPath "git"
$ffmpegPath = Get-ToolPath "ffmpeg"

$driveName = (Split-Path -Qualifier $WorkspaceRoot).TrimEnd(":")
$psDrive = Get-PSDrive -Name $driveName -ErrorAction SilentlyContinue
$freeGb = if ($psDrive) { [math]::Round($psDrive.Free / 1GB, 2) } else { -1 }

Write-Host "=== TUST-iMaker LeRobot Windows Setup ===" -ForegroundColor Yellow
Write-Host "Workspace Root : $WorkspaceRoot"
Write-Host "Target Venv    : $VenvDir"
Write-Host "uv             : $(if ($uvPath) { $uvPath } else { 'MISSING' })"
Write-Host "git            : $(if ($gitPath) { $gitPath } else { 'MISSING' })"
Write-Host "ffmpeg         : $(if ($ffmpegPath) { $ffmpegPath } else { 'MISSING (optional unless video codec used)' })"
Write-Host "Free Disk (${driveName}:): $freeGb GB"
Write-Host "Venv Python    : $(if (Test-Path $VenvPython) { $VenvPython } else { 'NOT CREATED' })"
Write-Host "repos/lerobot  : $(if (Test-Path (Join-Path $LerobotRepo 'pyproject.toml')) { 'PRESENT' } else { 'MISSING' })"

if ($freeGb -ge 0 -and $freeGb -lt 3) {
    Write-Warning "Free space on ${driveName}: is below 3 GB ($freeGb GB). Installation may fail."
}

if ($CheckOnly) {
    if (-not $uvPath) {
        Write-Warning "uv is missing. Install via: powershell -ExecutionPolicy ByPass -c `"irm https://astral.sh/uv/install.ps1 | iex`""
    }
    if (-not $gitPath) {
        Write-Warning "git is missing. Install Git for Windows."
    }
    if (-not (Test-Path $VenvPython)) {
        Write-Host "Next step: powershell -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1"
    }
    exit 0
}

if (-not $uvPath) {
    throw "uv is required but not found on PATH."
}
if (-not $gitPath) {
    throw "git is required but not found on PATH."
}

# Ensure Python 3.12 is available via uv
if ($DryRun) {
    Write-Host "[DRY-RUN] Check/install Python 3.12 via uv" -ForegroundColor Cyan
    Write-Host "          uv python find 3.12 || uv python install 3.12"
} else {
    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $null = & $uvPath python find 3.12 2>$null
        $pyFindExit = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prevEap
    }
    if ($pyFindExit -ne 0) {
        Invoke-Step "Install Python 3.12 via uv" $uvPath @("python", "install", "3.12")
    }
}

# Create/reuse venv
Invoke-Step "Create Python 3.12 virtual environment at $VenvDir" $uvPath @(
    "venv", $VenvDir, "--python", "3.12", "--allow-existing"
)

# Install workspace & Arm101 Studio dependencies (aligning opencv-python-headless with lerobot)
$baseDeps = @(
    "fastapi>=0.115.0,<1.0.0",
    "uvicorn[standard]>=0.32.0,<1.0.0",
    "websockets>=13.0,<18.0",
    "pyserial>=3.5,<4.0",
    "numpy>=2.0.0,<2.3.0",
    "pillow>=10.0.0,<14.0.0",
    "opencv-python-headless>=4.9.0,<4.14.0",
    "pyyaml>=6.0.0,<7.0.0",
    "pydantic>=2.0.0,<3.0.0",
    "pytest>=8.0.0",
    "pytest-asyncio>=0.23.0",
    "httpx>=0.27.0",
    "playwright>=1.49.0",
    "ruff>=0.6.0"
)

$pipBaseArgs = @("pip", "install", "--python", $VenvPython) + $baseDeps
Invoke-Step "Install workspace Studio & test/dev dependencies" $uvPath $pipBaseArgs

# Editable install of repos/lerobot[feetech] (or [feetech,core_scripts])
if (-not $SkipLerobotInstall) {
    $lerobotPyproject = Join-Path $LerobotRepo "pyproject.toml"
    if (-not (Test-Path $lerobotPyproject)) {
        throw "repos/lerobot/pyproject.toml not found. Run: python scripts\common\bootstrap_repos.py"
    }
    $extras = if ($WithCoreScripts) { "feetech,core_scripts" } else { "feetech" }
    $lerobotTarget = "${LerobotRepo}[${extras}]"
    $lerobotArgs = @("pip", "install", "--python", $VenvPython)
    if ($TorchBackend) {
        $lerobotArgs += @("--torch-backend", $TorchBackend)
    }
    $lerobotArgs += @("-e", $lerobotTarget)
    Invoke-Step "Install editable repos/lerobot[$extras]" $uvPath $lerobotArgs
} else {
    Write-Host "[SKIP] Skipping editable repos/lerobot install (-SkipLerobotInstall)." -ForegroundColor Yellow
}

# Optional: install root workspace package editably without clobbering deps
Invoke-Step "Install root workspace package (app) editably (--no-deps)" $uvPath @(
    "pip", "install", "--python", $VenvPython, "--no-deps", "-e", $WorkspaceRoot
)

# Optional: install LeLab non-editably with --no-deps so it builds frontend/dist without downgrading lerobot
if ($WithLeLab) {
    if (-not (Test-Path (Join-Path $LeLabRepo "pyproject.toml"))) {
        throw "repos/leLab/pyproject.toml not found. Run: python scripts\common\bootstrap_repos.py"
    }
    Invoke-Step "Install LeLab auxiliary dependencies (psutil, pygrabber)" $uvPath @(
        "pip", "install", "--python", $VenvPython, "psutil>=5.9.0", "pygrabber>=0.2"
    )
    Invoke-Step "Install repos/leLab (--no-deps, builds frontend/dist)" $uvPath @(
        "pip", "install", "--python", $VenvPython, "--no-deps", $LeLabRepo
    )
}

# Optional: Playwright Chromium browser install
if ($WithPlaywrightBrowser) {
    Invoke-Step "Install Playwright Chromium browser" $VenvPython @(
        "-m", "playwright", "install", "chromium"
    )
}

# Final environment verification
if (Test-Path $CheckEnvScript) {
    if ($DryRun) {
        Write-Host "[DRY-RUN] Run environment check" -ForegroundColor Cyan
        Write-Host "          `"$VenvPython`" `"$CheckEnvScript`""
    } else {
        Write-Host "[CHECK] Running check_env.py..." -ForegroundColor Green
        & $VenvPython $CheckEnvScript
    }
}

Write-Host "Setup complete." -ForegroundColor Green
