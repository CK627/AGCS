@echo off
setlocal enabledelayedexpansion

REM Dashboard manager (Windows)

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%i in ("%SCRIPT_DIR%\..\..") do set "PROJECT_DIR=%%~fi"

set "DEFAULT_PORT=20001"

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
REM Start (silent, runs via pythonw)
REM ============================================
:do_start
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"

echo === Dashboard start (port: %PORT%) ===

netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
if !errorlevel! equ 0 (
    echo Port %PORT% is already in use. Use "restart" or "stop" first.
    goto :eof
)

if not exist "%PROJECT_DIR%\data" mkdir "%PROJECT_DIR%\data"
if not exist "%PROJECT_DIR%\logs" mkdir "%PROJECT_DIR%\logs"

set PYTHONIOENCODING=utf-8
cd /d "%PROJECT_DIR%\backend"
start "" pythonw app.py --port %PORT%
echo Started. Logs: %PROJECT_DIR%\logs\server.log
echo %PORT%> "%PROJECT_DIR%\data\server.port"
echo URL: http://127.0.0.1:%PORT%
goto :eof

REM ============================================
REM Stop (kill by port; pythonw has no window title)
REM ============================================
:do_stop
echo === Dashboard stop ===
call :kill_by_port
del "%PROJECT_DIR%\data\server.pid" 2>nul
del "%PROJECT_DIR%\data\server.port" 2>nul
echo Stopped.
goto :eof

REM ============================================
REM Restart
REM ============================================
:do_restart
echo === Dashboard restart ===
call :kill_by_port
timeout /t 1 /nobreak >nul
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"
goto :do_start

REM ============================================
REM Status (check by port)
REM ============================================
:do_status
echo === Dashboard status ===
set "PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" set /p PORT=<"%PROJECT_DIR%\data\server.port"
netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
if !errorlevel! equ 0 (
    echo Status: running
    echo Port: %PORT%
    echo URL: http://127.0.0.1:%PORT%
) else (
    echo Status: not running
)
goto :eof

REM ============================================
REM Setup (install dependencies)
REM ============================================
:do_setup
echo === Dashboard setup ===
cd /d "%PROJECT_DIR%\backend"
python -m pip install -r requirements.txt
pause
goto :eof

REM ============================================
REM Install (check environment)
REM ============================================
:do_install
echo === Dashboard environment check ===
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
echo === Dashboard update ===
echo [1/3] Stopping...
call :kill_by_port
echo [2/3] Pulling latest code...
cd /d "%PROJECT_DIR%"
git pull origin main
if !errorlevel! neq 0 echo Pull failed && goto :eof
echo [3/3] Updating dependencies...
cd /d "%PROJECT_DIR%\backend"
python -m pip install -r requirements.txt -q
echo.
echo Updated. Restart with: start.bat start
goto :eof

REM ============================================
REM Uninstall (remove runtime data only)
REM ============================================
:do_uninstall
echo === Dashboard uninstall ===
call :kill_by_port
if exist "%PROJECT_DIR%\data" rmdir /s /q "%PROJECT_DIR%\data"
if exist "%PROJECT_DIR%\logs" rmdir /s /q "%PROJECT_DIR%\logs"
echo Removed runtime data (data/ and logs/). Code is kept.
pause
goto :eof

REM ============================================
REM Kill by port
REM ============================================
:kill_by_port
set "KP_PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" set /p KP_PORT=<"%PROJECT_DIR%\data\server.port"
for /f "tokens=5" %%a in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":%KP_PORT% " 2^>nul') do (
    taskkill /PID %%a /T /F >nul 2>&1
)
goto :eof

REM ============================================
REM Help
REM ============================================
:do_help
echo Dashboard manager (Windows)
echo.
echo Usage: start.bat [command] [args]
echo.
echo Commands:
echo   start [port]    Start the service (default port %DEFAULT_PORT%)
echo   stop            Stop the service
echo   restart [port]  Restart the service
echo   status          Show running status
echo   setup           Install dependencies
echo   install         Check environment
echo   update          Update to latest version
echo   uninstall       Remove runtime data
echo   help            Show this help
echo.
echo Examples:
echo   start.bat                  Start (default port)
echo   start.bat start 8080       Start on port 8080
echo   start.bat restart          Restart
echo   start.bat stop             Stop
echo   start.bat setup            Install dependencies
goto :eof
