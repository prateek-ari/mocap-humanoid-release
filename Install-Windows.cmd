@echo off
rem Mocap humanoid one-click Windows setup. Double-click this file.
rem All logic lives in tools\windows\install.ps1; options pass through, e.g.
rem   Install-Windows.cmd -Unattended -NoShortcuts -NoOpen -Workspace "D:\mocap runtime"
rem Deliberately linear (no labels, goto or multi-line blocks), so it also
rem runs if the file ever arrives with LF line endings. Pauses so a
rem double-click window stays readable; never with -Unattended (CI).
setlocal
set "MOCAP_INTERACTIVE=0"
if "%~1"=="" set "MOCAP_INTERACTIVE=1"
set "MOCAP_UNATTENDED=0"
echo(%*| "%SystemRoot%\System32\findstr.exe" /i /c:"-Unattended" >nul
if not errorlevel 1 set "MOCAP_UNATTENDED=1"
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\windows\install.ps1" %*
set "MOCAP_RC=%ERRORLEVEL%"
if not "%MOCAP_RC%"=="0" echo Setup did not finish, exit code %MOCAP_RC%. See the messages above.
if not "%MOCAP_RC%"=="0" if "%MOCAP_UNATTENDED%"=="0" pause
if "%MOCAP_RC%"=="0" if "%MOCAP_INTERACTIVE%"=="1" pause
exit /b %MOCAP_RC%
