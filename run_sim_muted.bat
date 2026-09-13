@echo off
REM run_sim_muted.bat
REM
REM Runs unified_tracking.py exactly like normal, but hides ONLY lines
REM containing "sequence size exceeds remaining buffer" from the console.
REM
REM The earlier attempt at this (run_sim_filtered.bat) LOOKED like it
REM filtered everything - that wasn't findstr's fault. Piping a Python
REM process's output (python.bat ... | findstr ...) removes its direct
REM connection to a real console, and Python responds by switching stdout
REM from line-buffered to fully block-buffered - so output doesn't appear
REM line-by-line anymore, it appears in big delayed chunks (or only at
REM exit). That looked indistinguishable from "everything got filtered."
REM
REM Fix: PYTHONUNBUFFERED=1 forces Python back to unbuffered stdout even
REM when piped, so findstr actually sees and filters lines live, the same
REM as running it directly.
REM
REM Run this INSTEAD of calling python.bat directly:
REM     C:\isaacsim\projects\surveillance-proj\run_sim_muted.bat

set PYTHONUNBUFFERED=1
C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\unified_tracking.py 2^>^&1 | findstr /v /c:"sequence size exceeds remaining buffer"
