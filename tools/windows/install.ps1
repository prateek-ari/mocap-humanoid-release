#Requires -Version 5.1
<#
.SYNOPSIS
  One-click Windows installer for mocap_humanoid (started by Install-Windows.cmd).

.DESCRIPTION
  Prepares everything below without admin rights and without touching
  system-wide settings, then hands over to tools\windows\install.py:

    %LOCALAPPDATA%\mocap-humanoid\uv\uv.exe     pinned uv, SHA256-verified
    %LOCALAPPDATA%\mocap-humanoid\python\       uv-managed CPython (pinned)
    %LOCALAPPDATA%\mocap-humanoid\mingit\       pinned MinGit, only if git is not on PATH
    %LOCALAPPDATA%\mocap-humanoid\logs\         install logs
    <repo>\.venv                                project venv (Python 3.13)

  install.py installs the project, runs the setup checks and bootstraps the
  GMR runtime workspace. Finally this script adds "Mocap Setup" shortcuts and
  opens the Setup console. Every step is reused on a rerun.

  Keep this file ASCII-only: Windows PowerShell 5.1 reads BOM-less files in
  the ANSI code page.
#>
[CmdletBinding()]
param(
    # No questions; for CI and scripted installs.
    [switch]$Unattended,
    # Do not create the Start Menu and Desktop shortcuts.
    [switch]$NoShortcuts,
    # Do not open the Setup console at the end.
    [switch]$NoOpen,
    # GMR runtime workspace (default: %USERPROFILE%\mocap-runtime).
    [string]$Workspace = '',
    # Print what would happen; download, install and change nothing.
    [switch]$DryRun
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
# Invoke-WebRequest in 5.1 is extremely slow while it draws a progress bar.
$ProgressPreference = 'SilentlyContinue'
# Older Windows 10 .NET defaults may still offer TLS 1.0/1.1 first.
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

# ----------------------------------------------------------------- pins
# Bump deliberately; hashes come from the GitHub release assets (API digest)
# and were re-verified by downloading the files.
$UvVersion = '0.12.20'   # released 2026-09-28
$UvUrl = "https://github.com/astral-sh/uv/releases/download/$UvVersion/uv-x86_64-pc-windows-msvc.zip"
$UvZipSha256 = '95f9bc30fbb3574d276e28ac4a6de932d25153645853d13da8c21eec3bc88d06'
$UvExeSha256 = 'a0d2742d49564a32488753b02e76276e7b5ef1b1ea8cf30bcbf06ee28f60cd73'
$PythonVersion = '3.13.15'   # uv-managed; uv verifies its own download hashes
$MinGitTag = 'v2.55.0.windows.5'   # released 2026-08-20
$MinGitUrl = "https://github.com/git-for-windows/git/releases/download/$MinGitTag/MinGit-2.55.0.5-64-bit.zip"
$MinGitZipSha256 = '56d7b226b7693196cfc71fef26568f536c4a021ab6c37ff2db4287bed908e96e'
$MinGitExeSha256 = '78211c7ed73988da93a6d8a33d47ec6187f464d7ea2a9a00c182bbd7a1ecf30f'

# ----------------------------------------------------------------- paths
$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).ProviderPath.TrimEnd('\')
if (-not $env:LOCALAPPDATA) { throw 'LOCALAPPDATA is not set; run the installer as a normal user.' }
$ToolsDir = Join-Path $env:LOCALAPPDATA 'mocap-humanoid'
$LogDir = Join-Path $ToolsDir 'logs'
$UvDir = Join-Path $ToolsDir 'uv'
$UvExe = Join-Path $UvDir 'uv.exe'
$PythonDir = Join-Path $ToolsDir 'python'
$MinGitDir = Join-Path $ToolsDir 'mingit'
$MinGitCmd = Join-Path $MinGitDir 'cmd'
$VenvDir = Join-Path $RepoRoot '.venv'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'
$VenvPythonw = Join-Path $VenvDir 'Scripts\pythonw.exe'
$Launcher = Join-Path $RepoRoot 'tools\windows\console_launcher.py'
$InstallPy = Join-Path $RepoRoot 'tools\windows\install.py'
$Stage = 'start'

function Say([string]$Text) { Write-Host $Text }
function Step([string]$Name) {
    $script:Stage = $Name
    Write-Host ''
    Write-Host "== $Name =="
}
function Die([string]$Text) { throw $Text }

# Run a native program with an argv array; stream its output into the
# transcript (5.1 only records native output that passes through the pipeline)
# and fail on a nonzero exit code. Stderr lines must not become terminating
# errors, so the preference is relaxed inside this function only.
function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments = @())
    $shown = @($Exe) + $Arguments | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }
    Say ('+ ' + ($shown -join ' '))
    if ($DryRun) { return }
    $ErrorActionPreference = 'Continue'
    & $Exe @Arguments 2>&1 | ForEach-Object { "$_" } | Out-Host
    $code = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($code -ne 0) { Die "$(Split-Path -Leaf $Exe) failed with exit code $code" }
}

function Get-Sha256([string]$Path) {
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

# Download to a temporary file, verify SHA256, extract into a fresh staging
# folder and only then move it into place, so a partial download or
# extraction is never reused.
function Install-PinnedZip {
    param([string]$Url, [string]$ZipSha256, [string]$Destination, [string]$Name)
    Say "Downloading $Name"
    Say "  $Url"
    if ($DryRun) { return }
    $download = Join-Path $ToolsDir ("download-" + [IO.Path]::GetRandomFileName() + '.zip')
    $staging = "$Destination.staging"
    try {
        Invoke-WebRequest -Uri $Url -OutFile $download -UseBasicParsing
        $actual = Get-Sha256 $download
        if ($actual -ne $ZipSha256) {
            Die "SHA256 mismatch for $Name (expected $ZipSha256, got $actual); not installing it"
        }
        Say "  SHA256 verified: $actual"
        if (Test-Path -LiteralPath $staging) { Remove-Item -LiteralPath $staging -Recurse -Force }
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        [IO.Compression.ZipFile]::ExtractToDirectory($download, $staging)
        if (Test-Path -LiteralPath $Destination) { Remove-Item -LiteralPath $Destination -Recurse -Force }
        Move-Item -LiteralPath $staging -Destination $Destination
    } finally {
        if (Test-Path -LiteralPath $download) { Remove-Item -LiteralPath $download -Force }
        if (Test-Path -LiteralPath $staging) { Remove-Item -LiteralPath $staging -Recurse -Force }
    }
}

function Test-VenvPython([string]$Python) {
    if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) { return $false }
    $ErrorActionPreference = 'Continue'
    & $Python -c 'import sys; sys.exit(0 if sys.version_info >= (3, 13) else 1)' 2>&1 | Out-Null
    $ok = ($LASTEXITCODE -eq 0)
    $ErrorActionPreference = 'Stop'
    return $ok
}

function Confirm-Yes([string]$Question, [bool]$Default) {
    if ($Unattended) { return $Default }
    $hint = if ($Default) { 'Y/n' } else { 'y/N' }
    while ($true) {
        $reply = Read-Host "$Question [$hint]"
        if ([string]::IsNullOrWhiteSpace($reply)) { return $Default }
        if ($reply -match '^(y|yes)$') { return $true }
        if ($reply -match '^(n|no)$') { return $false }
    }
}

function New-Shortcut([string]$Path) {
    $shell = New-Object -ComObject WScript.Shell
    $link = $shell.CreateShortcut($Path)
    $link.TargetPath = $VenvPythonw
    $link.Arguments = '"' + $Launcher + '"'
    $link.WorkingDirectory = $RepoRoot
    $link.Description = 'Open the Mocap Setup console in the browser'
    $link.IconLocation = "$VenvPythonw,0"
    $link.Save()
    Say "  $Path"
}

# ----------------------------------------------------------------- main
# Returns $false when the user cancels, $true when setup completed.
function Invoke-Install {
    Say 'Mocap humanoid - Windows setup'
    Say "  Checkout:    $RepoRoot"
    Say "  Tools:       $ToolsDir"
    Say "  Project env: $VenvDir"
    if ($DryRun) { Say '  DRY RUN: nothing is downloaded, installed or changed.' }

    Step 'system check'
    if (-not [Environment]::Is64BitOperatingSystem) { Die 'a 64-bit Windows 10 or 11 is required' }
    if ([Environment]::OSVersion.Version.Major -lt 10) { Die 'Windows 10 or 11 is required' }
    $arch = if ($env:PROCESSOR_ARCHITEW6432) { $env:PROCESSOR_ARCHITEW6432 } else { $env:PROCESSOR_ARCHITECTURE }
    if ($arch -ne 'AMD64') { Say "Warning: $arch Windows; the x64 tools run under emulation and are not tested there." }
    $identity = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
    if ($identity.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        Say 'Note: running as administrator is not needed; everything installs for this user only.'
    }
    if ($RepoRoot.StartsWith('\\')) { Die "run the installer from a local drive, not a network path ($RepoRoot)" }

    $workspaceArg = @()
    $ws = $Workspace.Trim()
    if ($ws) {
        # install.py validates it (absolute, outside the checkout, no links).
        # A trailing backslash would escape the closing quote of the argument.
        if ($ws.Length -gt 3) { $ws = $ws.TrimEnd('\') }
        $workspaceArg = @('--workspace', $ws)
        Say "Runtime workspace: $ws"
    } else {
        Say "Runtime workspace: $(Join-Path $env:USERPROFILE 'mocap-runtime') (default)"
    }
    if (-not $Unattended -and -not $DryRun) {
        Say ''
        Say 'This downloads about 1 GB (uv, Python, the robot runtime) and takes 5-20 minutes.'
        if (-not (Confirm-Yes 'Start the setup?' $true)) { Say 'Setup cancelled.'; return $false }
    }

    Step 'unblock downloaded files'
    # A GitHub ZIP carries Mark-of-the-Web on every extracted file. Skip the
    # (large, self-created) .venv, .git and node_modules folders.
    if (-not $DryRun) {
        $skip = @('.venv', '.git', 'node_modules')
        Get-ChildItem -LiteralPath $RepoRoot -Force | Where-Object { $skip -notcontains $_.Name } | ForEach-Object {
            if ($_.PSIsContainer) {
                Get-ChildItem -LiteralPath $_.FullName -Recurse -File -Force -ErrorAction SilentlyContinue |
                    Unblock-File -ErrorAction SilentlyContinue
            } else {
                Unblock-File -LiteralPath $_.FullName -ErrorAction SilentlyContinue
            }
        }
        Say 'Done.'
    } else {
        Say "Would remove Mark-of-the-Web from the files in $RepoRoot."
    }

    Step 'uv (Python manager)'
    if (-not $DryRun) { New-Item -ItemType Directory -Force -Path $ToolsDir | Out-Null }
    if ((Test-Path -LiteralPath $UvExe -PathType Leaf) -and ((Get-Sha256 $UvExe) -eq $UvExeSha256)) {
        Say "uv $UvVersion already installed."
    } else {
        Install-PinnedZip -Url $UvUrl -ZipSha256 $UvZipSha256 -Destination $UvDir -Name "uv $UvVersion"
        if (-not $DryRun -and (Get-Sha256 $UvExe) -ne $UvExeSha256) { Die 'uv.exe does not match the pinned build' }
    }
    # Keep uv self-contained: its cache and Pythons under the tools dir, no
    # user/project uv config, no PATH, registry or ~\.local\bin changes.
    $env:UV_CACHE_DIR = Join-Path $UvDir 'cache'
    $env:UV_PYTHON_INSTALL_DIR = $PythonDir
    $env:UV_NO_CONFIG = '1'
    $env:UV_PYTHON_PREFERENCE = 'only-managed'
    $env:UV_NO_MODIFY_PATH = '1'
    $env:PYTHONUNBUFFERED = '1'

    Step "Python $PythonVersion"
    Invoke-Native $UvExe @('python', 'install', '--no-bin', '--no-registry', $PythonVersion)

    Step 'project environment (.venv)'
    if (Test-VenvPython $VenvPython) {
        Say "Reusing $VenvDir (Python >= 3.13)."
    } else {
        if (Test-Path -LiteralPath $VenvDir) {
            Say "$VenvDir exists but is not a Windows Python >= 3.13 environment."
            if (-not (Confirm-Yes 'Replace it?' $true)) { Die "move $VenvDir aside and rerun the installer" }
        }
        Invoke-Native $UvExe @('venv', '--clear', '--seed', '--python', $PythonVersion, $VenvDir)
    }

    Step 'Git'
    $pathGit = Get-Command git.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($pathGit) {
        Say "Using $($pathGit.Source)"
    } else {
        $minGitExe = Join-Path $MinGitCmd 'git.exe'
        if ((Test-Path -LiteralPath $minGitExe -PathType Leaf) -and ((Get-Sha256 $minGitExe) -eq $MinGitExeSha256)) {
            Say "Portable MinGit ($MinGitTag) already installed."
        } else {
            Install-PinnedZip -Url $MinGitUrl -ZipSha256 $MinGitZipSha256 -Destination $MinGitDir -Name "MinGit $MinGitTag"
            if (-not $DryRun -and (Get-Sha256 $minGitExe) -ne $MinGitExeSha256) { Die 'git.exe does not match the pinned MinGit build' }
        }
        # This process and its children only; the user PATH is not modified.
        $env:PATH = "$MinGitCmd;$env:PATH"
        Say "Using portable MinGit: $minGitExe"
    }

    Step 'project install, checks and GMR runtime (install.py)'
    $installArgs = @($InstallPy) + $workspaceArg
    if ($DryRun) { $installArgs += '--dry-run' }
    Invoke-Native $VenvPython $installArgs

    if (-not $NoShortcuts) {
        Step 'shortcuts'
        if (-not $DryRun) {
            New-Shortcut (Join-Path ([Environment]::GetFolderPath('Programs')) 'Mocap Setup.lnk')
            New-Shortcut (Join-Path ([Environment]::GetFolderPath('Desktop')) 'Mocap Setup.lnk')
        } else {
            Say 'Would add "Mocap Setup" to the Start Menu and the Desktop.'
        }
    }

    Say ''
    if ($DryRun) {
        Say 'DRY RUN: NOT VERIFIED; nothing was installed or checked.'
    } else {
        Say 'PASS: Windows setup completed.'
        Say 'Open the Setup console any time from the Start Menu or Desktop: Mocap Setup.'
        if ($ws) { Say "The Setup console uses this runtime folder by default: $ws" }
    }

    if (-not $NoOpen -and -not $DryRun) {
        Step 'Setup console'
        try {
            Invoke-Native $VenvPython @($Launcher)
        } catch {
            # The install itself passed; only opening the console failed.
            Say "Could not open the Setup console: $($_.Exception.Message)"
            Say "Open it from the Start Menu (Mocap Setup) or run: `"$VenvPython`" `"$Launcher`""
        }
    }
    return $true
}

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$logFile = Join-Path $LogDir "install-$stamp.log"
$transcript = $false
$exitCode = 1
try {
    if (-not $DryRun) {
        New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
        Start-Transcript -LiteralPath $logFile | Out-Null
        $transcript = $true
    }
    $null = Invoke-Install
    $exitCode = 0
} catch {
    Write-Host ''
    Write-Host "FAIL: $Stage stopped: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host 'Fix the error above and run Install-Windows.cmd again; finished steps are reused.'
    $exitCode = 1
} finally {
    if ($transcript) {
        Write-Host "Log: $logFile"
        Stop-Transcript | Out-Null
    }
}
exit $exitCode
