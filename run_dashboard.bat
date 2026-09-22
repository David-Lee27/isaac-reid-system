@echo off
REM Launches the live dashboard. Double-click this file, or run it from any
REM directory - it cd's into reporting/ (where dashboard.py actually lives)
REM first, which is the actual fix for "uvicorn dashboard:app doesn't work":
REM uvicorn needs to be run FROM the same folder as dashboard.py to find it,
REM and a plain "python.bat -m uvicorn dashboard:app" typed from wherever the
REM terminal happened to be (e.g. the repo root, or C:\isaacsim) fails with
REM "Could not import module 'dashboard'" for exactly that reason.
cd /d "%~dp0reporting"
echo Starting dashboard at http://127.0.0.1:8000/ (reads event_log.json - run the sim first, or alongside this)
C:\isaacsim\python.bat -m uvicorn dashboard:app --port 8000
pause
