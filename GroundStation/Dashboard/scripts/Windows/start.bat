@echo off
chcp 65001 >nul 2>&1
setlocal enabledelayedexpansion

REM 地面站仪表盘 管理入口 (Windows)

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%i in ("%SCRIPT_DIR%\..\..") do set "PROJECT_DIR=%%~fi"

if "%~1"=="" goto :do_start
if "%~1"=="start" goto :do_start
if "%~1"=="stop" goto :do_stop
if "%~1"=="restart" goto :do_restart
if "%~1"=="status" goto :do_status
if "%~1"=="install" goto :do_install
if "%~1"=="update" goto :do_update
if "%~1"=="uninstall" goto :do_uninstall
if "%~1"=="help" goto :do_help

echo 未知命令: %~1
goto :do_help

REM ============================================
REM 启动
REM ============================================
:do_start
set "PORT=%~2"
if "%PORT%"=="" set "PORT=20001"

echo === 地面站仪表盘 启动 (端口: %PORT%) ===

tasklist /FI "WINDOWTITLE eq 地面站仪表盘" 2>nul | find "python.exe" >nul
if !errorlevel! equ 0 (
    echo 服务已在运行中
    goto :eof
)

cd /d "%PROJECT_DIR%\backend"
if not exist "%PROJECT_DIR%\data" mkdir "%PROJECT_DIR%\data"
if not exist "%PROJECT_DIR%\logs" mkdir "%PROJECT_DIR%\logs"

start "地面站仪表盘" python app.py --port %PORT%
echo 服务已启动
echo %PORT%> "%PROJECT_DIR%\data\server.port"
echo 访问: http://localhost:%PORT%
goto :eof

REM ============================================
REM 停止
REM ============================================
:do_stop
echo === 地面站仪表盘 停止 ===
taskkill /FI "WINDOWTITLE eq 地面站仪表盘" /T >nul 2>&1
del /q "%PROJECT_DIR%\data\server.port" 2>nul
echo 已停止
goto :eof

REM ============================================
REM 重启
REM ============================================
:do_restart
echo === 地面站仪表盘 重启 ===
taskkill /FI "WINDOWTITLE eq 地面站仪表盘" /T >nul 2>&1
timeout /t 1 /nobreak >nul
set "PORT=%~2"
if "%PORT%"=="" set "PORT=20001"
goto :do_start

REM ============================================
REM 状态
REM ============================================
:do_status
echo === 地面站仪表盘 状态 ===
tasklist /FI "WINDOWTITLE eq 地面站仪表盘" 2>nul | find "python.exe" >nul
if !errorlevel! equ 0 (
    echo 状态: 运行中
    set "PORT=20001"
    if exist "%PROJECT_DIR%\data\server.port" set /p PORT=<"%PROJECT_DIR%\data\server.port"
    echo 访问: http://localhost:!PORT!
) else (
    echo 状态: 未运行
)
goto :eof

REM ============================================
REM 安装
REM ============================================
:do_install
echo ========================================
echo   地面站仪表盘 环境安装 (Windows)
echo ========================================
echo.

echo [1/1] 安装 Python 依赖...
cd /d "%PROJECT_DIR%\backend"
pip install -r requirements.txt
cd /d "%PROJECT_DIR%"
echo.

echo ========================================
echo   安装完成！
echo ========================================
echo.
echo 启动: scripts\Windows\start.bat start
pause
goto :eof

REM ============================================
REM 更新
REM ============================================
:do_update
echo === 地面站仪表盘 更新 ===

echo [1/3] 停止服务...
taskkill /FI "WINDOWTITLE eq 地面站仪表盘" /T >nul 2>&1

echo [2/3] 拉取最新代码...
cd /d "%PROJECT_DIR%"
git pull origin main
if !errorlevel! neq 0 echo 拉取失败 && goto :eof

echo [3/3] 更新依赖...
pip install -r "%PROJECT_DIR%\backend\requirements.txt" -q

echo.
echo 更新完成！重新启动: scripts\Windows\start.bat start
goto :eof

REM ============================================
REM 卸载
REM ============================================
:do_uninstall
echo ========================================
echo   地面站仪表盘 卸载 (Windows)
echo ========================================
echo.

echo [1/3] 停止服务...
taskkill /FI "WINDOWTITLE eq 地面站仪表盘" /T >nul 2>&1
echo.

echo [2/3] 清理运行时数据...
if exist "%PROJECT_DIR%\data" rmdir /s /q "%PROJECT_DIR%\data"
if exist "%PROJECT_DIR%\logs" rmdir /s /q "%PROJECT_DIR%\logs"
echo 完成
echo.

set /p "CONFIRM=[3/3] 卸载 Python 依赖? (y/N) "
if /i "%CONFIRM%"=="y" (
    pip uninstall -y flask waitress requests pymavlink opencv-python numpy 2>nul
    echo 依赖已卸载
) else (
    echo 跳过
)

echo.
echo 卸载完成
pause
goto :eof

REM ============================================
REM 帮助
REM ============================================
:do_help
echo 地面站仪表盘 管理脚本 (Windows)
echo.
echo 用法:  start.bat [命令] [参数]
echo.
echo 命令:
echo   start [端口]   启动服务（默认端口 20001）
echo   stop           停止服务
echo   restart [端口] 重启服务
echo   status         查看运行状态
echo   install        环境安装（Python 依赖）
echo   update         更新到最新版本
echo   uninstall      卸载
echo   help           帮助
echo.
echo 示例:
echo   start.bat                   启动（默认端口 20001）
echo   start.bat start 8080        启动并指定端口 8080
echo   start.bat restart           重启服务
echo   start.bat stop              停止服务
goto :eof
