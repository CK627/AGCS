@echo off
chcp 65001 >nul 2>&1
setlocal enabledelayedexpansion

REM Robot Dashboard 管理入口 (Windows)

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%i in ("%SCRIPT_DIR%\..\..") do set "PROJECT_DIR=%%~fi"

set "DEFAULT_PORT=20002"

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

echo 未知命令: %~1
goto :do_help

REM ============================================
REM 启动
REM ============================================
:do_start
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"

echo === Robot Dashboard 启动 (端口: %PORT%) ===

REM 检查端口是否被占用
netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
if !errorlevel! equ 0 (
    echo 端口 %PORT% 已被占用
    echo 如需重启: start.bat restart
    goto :eof
)

REM 检查 Python
where python >nul 2>&1
if !errorlevel! neq 0 (
    echo 错误: 未找到 Python
    goto :eof
)

REM 确保必要目录
if not exist "%PROJECT_DIR%\data" mkdir "%PROJECT_DIR%\data"
if not exist "%PROJECT_DIR%\logs" mkdir "%PROJECT_DIR%\logs"

cd /d "%PROJECT_DIR%"

REM 保存端口号
echo %PORT%> "%PROJECT_DIR%\data\server.port"

start "RobotDashboard" python backend\app.py --port %PORT%
echo 服务已启动
echo 访问: http://127.0.0.1:%PORT%
echo.
echo 日志: %PROJECT_DIR%\logs\
goto :eof

REM ============================================
REM 停止
REM ============================================
:do_stop
echo === Robot Dashboard 停止 ===

set "STOPPED=0"

REM 按窗口标题杀
tasklist /FI "WINDOWTITLE eq RobotDashboard" 2>nul | find "python.exe" >nul
if !errorlevel! equ 0 (
    taskkill /FI "WINDOWTITLE eq RobotDashboard" /T >nul 2>&1
    set "STOPPED=1"
)

REM 按端口杀（后备，处理窗口标题不匹配的情况）
set "PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" (
    set /p PORT=<"%PROJECT_DIR%\data\server.port"
)

for /f "tokens=5" %%a in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":%PORT% " 2^>nul') do (
    taskkill /PID %%a /T /F >nul 2>&1
    set "STOPPED=1"
)

del "%PROJECT_DIR%\data\server.pid" 2>nul
del "%PROJECT_DIR%\data\server.port" 2>nul

if "!STOPPED!"=="1" (
    echo 服务已停止
) else (
    echo 服务未在运行
)
goto :eof

REM ============================================
REM 重启
REM ============================================
:do_restart
echo === Robot Dashboard 重启 ===
set "PORT=%~2"
if "%PORT%"=="" set "PORT=%DEFAULT_PORT%"
taskkill /FI "WINDOWTITLE eq RobotDashboard" /T >nul 2>&1
timeout /t 1 /nobreak >nul
goto :do_start

REM ============================================
REM 状态
REM ============================================
:do_status
echo === Robot Dashboard 状态 ===

set "PORT=%DEFAULT_PORT%"
if exist "%PROJECT_DIR%\data\server.port" (
    set /p PORT=<"%PROJECT_DIR%\data\server.port"
)

tasklist /FI "WINDOWTITLE eq RobotDashboard" 2>nul | find "python.exe" >nul
if !errorlevel! equ 0 (
    echo 状态: 运行中
) else (
    REM 后备：按端口检查
    netstat -ano 2>nul | findstr "LISTENING" | findstr ":%PORT% " >nul
    if !errorlevel! equ 0 (
        echo 状态: 运行中
    ) else (
        echo 状态: 未运行
        goto :eof
    )
)
echo 端口: %PORT%
echo 访问: http://127.0.0.1:%PORT%
goto :eof

REM ============================================
REM 依赖安装
REM ============================================
:do_setup
echo === Robot Dashboard 依赖安装 ===
cd /d "%PROJECT_DIR%"
python -m pip install -r requirements.txt
pause
goto :eof

REM ============================================
REM 安装
REM ============================================
:do_install
echo ========================================
echo   Robot Dashboard 环境安装 (Windows)
echo ========================================
echo.

echo [1/2] 检查 Python...
where python >nul 2>&1
if !errorlevel! neq 0 (
    echo   未检测到 Python，请先安装 Python 3.8+
    echo   下载: https://www.python.org/downloads/windows/
    pause
    goto :eof
)
python --version
echo.

echo [2/2] 检查 Python 依赖...
python -c "import flask, waitress, requests" >nul 2>&1
if !errorlevel! equ 0 (
    echo   ✓ 核心依赖已安装
) else (
    echo   ⚠ 缺少依赖，请执行: start.bat setup
)
echo.

echo ========================================
echo   安装完成！
echo ========================================
echo.
echo 首次运行请先安装依赖:
echo   scripts\Windows\start.bat setup
echo.
pause
goto :eof

REM ============================================
REM 更新
REM ============================================
:do_update
echo === Robot Dashboard 更新 ===

echo [1/2] 停止服务...
taskkill /FI "WINDOWTITLE eq RobotDashboard" /T >nul 2>&1

echo [2/2] 拉取最新代码...
cd /d "%PROJECT_DIR%"
git pull origin main
if !errorlevel! neq 0 (
    echo 拉取失败
    goto :eof
)

echo.
echo 更新完成！重新启动: scripts\Windows\start.bat start
goto :eof

REM ============================================
REM 卸载
REM ============================================
:do_uninstall
echo ========================================
echo   Robot Dashboard 卸载 (Windows)
echo ========================================
echo.

echo [1/2] 停止服务...
taskkill /FI "WINDOWTITLE eq RobotDashboard" /T >nul 2>&1

echo [2/2] 清理运行时数据...
if exist "%PROJECT_DIR%\data" rmdir /s /q "%PROJECT_DIR%\data"
if exist "%PROJECT_DIR%\logs" rmdir /s /q "%PROJECT_DIR%\logs"
echo 完成
echo.
echo 卸载完成（代码未删除）
pause
goto :eof

REM ============================================
REM 帮助
REM ============================================
:do_help
echo Robot Dashboard 管理脚本 (Windows)
echo.
echo 用法:  start.bat [命令] [参数]
echo.
echo 命令:
echo   start [端口]   启动服务（默认端口 20002）
echo   stop           停止服务
echo   restart [端口] 重启服务
echo   status         查看运行状态
echo   setup          安装依赖（首次运行）
echo   install        环境检查（Python / 依赖）
echo   update         更新到最新版本
echo   uninstall      卸载
echo   help           帮助
echo.
echo 示例:
echo   start.bat                   启动（默认端口 20002）
echo   start.bat start 8080        启动并指定端口 8080
echo   start.bat restart           重启服务
echo   start.bat stop              停止服务
echo   start.bat setup             安装依赖
pause
goto :eof
