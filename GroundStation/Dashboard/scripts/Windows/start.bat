@echo off
setlocal enabledelayedexpansion

REM Ground station manager (Windows): hub dashboard + progress flowchart

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%i in ("%SCRIPT_DIR%\..\..") do set "PROJECT_DIR=%%~fi"

set "DEFAULT_PORT=20000"
set "DEFAULT_PPORT=20010"

if "%~1"=="" goto :do_start
if "%~1"=="start" goto :do_start
if "%~1"=="stop" goto :do_stop
if "%~1"=="restart" goto :do_restart
if "%~1"=="status" goto :do_status
if "%~1"=="setup" goto :do_setup
if "%~1"=="install" goto :do_install
if "%~1"=="update" goto :do_update
if "%~1"=="uninstall" goto :do_uninstall
if "%~1"=="help" goto :do_help

echo Unknown command: %~1
goto :do_help

REM ============================================
REM Start both services (silent, runs via pythonw)
REM ============================================
:do_start
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"
set "PPORT=%~3"
if "%PPORT%"=="" set "PPORT=%DEFAULT_PPORT%"

echo === Ground station start (hub: %PORT%, progress: %PPORT%) ===

if not exist "%PROJECT_DIR%\data" mkdir "%PROJECT_DIR%\data"
if not exist "%PROJECT_DIR%\logs" mkdir "%PROJECT_DIR%\logs"

REM ---- hub ----
netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
if !errorlevel! equ 0 (
    echo Hub already running on port %PORT%.
) else (
    set PYTHONIOENCODING=utf-8
    cd /d "%PROJECT_DIR%\backend"
    start "" pythonw app.py --port %PORT%
    echo %PORT%> "%PROJECT_DIR%\data\server.port"
    echo Hub started. Logs: %PROJECT_DIR%\logs\nohup.log
)

REM ---- progress flowchart (own port) ----
netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PPORT% " >nul
if !errorlevel! equ 0 (
    echo Progress page already running on port %PPORT%.
) else (
    set PYTHONIOENCODING=utf-8
    cd /d "%PROJECT_DIR%\progress"
    start "" pythonw app.py --port %PPORT% --hub http://127.0.0.1:%PORT%
    echo %PPORT%> "%PROJECT_DIR%\data\progress.port"
    echo Progress page started. Logs: %PROJECT_DIR%\logs\progress.log
)

echo.
echo Hub      : http://127.0.0.1:%PORT%
echo Progress : http://127.0.0.1:%PPORT%
goto :eof

REM ============================================
REM Stop both (kill by port; pythonw has no window title)
REM ============================================
:do_stop
echo === Ground station stop ===
call :kill_by_port
del "%PROJECT_DIR%\data\server.pid" 2>nul
del "%PROJECT_DIR%\data\server.port" 2>nul
del "%PROJECT_DIR%\data\progress.pid" 2>nul
del "%PROJECT_DIR%\data\progress.port" 2>nul
echo Stopped.
goto :eof

REM ============================================
REM Restart both
REM ============================================
:do_restart
echo === Ground station restart ===
call :kill_by_port
timeout /t 1 /nobreak >nul
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"
set "PPORT=%~3"
if "%PPORT%"=="" set "PPORT=%DEFAULT_PPORT%"
goto :do_start

REM ============================================
REM Status (both ports)
REM ============================================
:do_status
echo === Ground station status ===
set "PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" set /p PORT=<"%PROJECT_DIR%\data\server.port"
set "PPORT=%DEFAULT_PPORT%"
if exist "%PROJECT_DIR%\data\progress.port" set /p PPORT=<"%PROJECT_DIR%\data\progress.port"

netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
if !errorlevel! equ 0 (
    echo Hub      : running  http://127.0.0.1:%PORT%
) else (
    echo Hub      : not running
)

netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PPORT% " >nul
if !errorlevel! equ 0 (
    echo Progress : running  http://127.0.0.1:%PPORT%
) else (
    echo Progress : not running
)
goto :eof

REM ============================================
REM Setup (install dependencies for both)
REM ============================================
:do_setup
echo === Ground station setup ===
cd /d "%PROJECT_DIR%\backend"
python -m pip install -r requirements.txt
cd /d "%PROJECT_DIR%\progress"
python -m pip install -r requirements.txt
pause
goto :eof

REM ============================================
REM Install (check environment)
REM ============================================
:do_install
echo === Ground station environment check ===
where python >nul 2>&1
if !errorlevel! neq 0 (
    echo Python not found. Please install Python 3.8+ and add it to PATH.
    pause
    goto :eof
)
python --version
echo.
echo Run "start.bat setup" to install dependencies.
pause
goto :eof

REM ============================================
REM Update (git pull + deps)
REM ============================================
:do_update
echo === Ground station update ===
echo [1/3] Stopping...
call :kill_by_port
echo [2/3] Pulling latest code...
cd /d "%PROJECT_DIR%"
git pull origin ground-station
if !errorlevel! neq 0 echo Pull failed && goto :eof
echo [3/3] Updating dependencies...
cd /d "%PROJECT_DIR%\backend"
python -m pip install -r requirements.txt -q
cd /d "%PROJECT_DIR%\progress"
python -m pip install -r requirements.txt -q
echo.
echo Updated. Restart with: start.bat start
goto :eof

REM ============================================
REM Uninstall (remove runtime data only)
REM ============================================
:do_uninstall
echo === Ground station uninstall ===
call :kill_by_port
if exist "%PROJECT_DIR%\data" rmdir /s /q "%PROJECT_DIR%\data"
if exist "%PROJECT_DIR%\logs" rmdir /s /q "%PROJECT_DIR%\logs"
echo Removed runtime data (data/ and logs/). Code is kept.
pause
goto :eof

REM ============================================
REM Kill by port (hub + progress)
REM ============================================
:kill_by_port
set "KP_PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" set /p KP_PORT=<"%PROJECT_DIR%\data\server.port"
for /f "tokens=5" %%a in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":%KP_PORT% " 2^>nul') do (
    taskkill /PID %%a /T /F >nul 2>&1
)
set "KP_PPORT=%DEFAULT_PPORT%"
if exist "%PROJECT_DIR%\data\progress.port" set /p KP_PPORT=<"%PROJECT_DIR%\data\progress.port"
for /f "tokens=5" %%a in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":%KP_PPORT% " 2^>nul') do (
    taskkill /PID %%a /T /F >nul 2>&1
)
goto :eof

REM ============================================
REM Help
REM ============================================
:do_help
echo Ground station manager (Windows)
echo Manages 2 services together: hub dashboard + progress flowchart
echo.
echo Usage: start.bat [command] [args]
echo.
echo Commands:
echo   start [hub_port] [progress_port]   Start both (default %DEFAULT_PORT% / %DEFAULT_PPORT%)
echo   stop                               Stop both
echo   restart [hub_port] [progress_port] Restart both
echo   status                             Show status of both
echo   setup                              Install dependencies
echo   install                            Check environment
echo   update                             Update to latest version
echo   uninstall                          Remove runtime data
echo   help                               Show this help
echo.
echo Examples:
echo   start.bat                          Start (default ports)
echo   start.bat start 8080 8081          Hub on 8080, progress on 8081
echo   start.bat restart                  Restart both
echo   start.bat stop                     Stop both
echo   start.bat setup                    Install dependencies
goto :eof
