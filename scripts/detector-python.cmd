@echo off
setlocal
set "YOLO_CONFIG_DIR=%~dp0..\runs\yolo-config"
set "PYTHONPATH=%~dp0..\src"
if not exist "%YOLO_CONFIG_DIR%" mkdir "%YOLO_CONFIG_DIR%"
"%~dp0..\.venv-detector\Scripts\python.exe" %*
exit /b %errorlevel%
