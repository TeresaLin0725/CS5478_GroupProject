@echo off
call "%~dp0detector-python.cmd" -m frontier_search.runner.replay run --detector-weights "%~dp0..\weights\yolo11\yolo11s.pt" %*
exit /b %errorlevel%
