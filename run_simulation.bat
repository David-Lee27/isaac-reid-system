@echo off
REM run_simulation.bat - launches the simulation. Double-click it, or run it
REM from anywhere.
REM
REM Runs unified_tracking.py exactly like normal, but hides ONLY lines
REM containing "sequence size exceeds remaining buffer" from the console.
REM
REM The earlier attempt at this LOOKED like it filtered everything - that
REM wasn't findstr's fault. Piping a Python process's output
REM (python.bat ... | findstr ...) removes its direct connection to a real
REM console, and Python responds by switching stdout from line-buffered to
REM fully block-buffered - so output doesn't appear line-by-line anymore, it
REM appears in big delayed chunks (or only at exit). That looked
REM indistinguishable from "everything got filtered."
REM
REM Fix: PYTHONUNBUFFERED=1 forces Python back to unbuffered stdout even
REM when piped, so findstr actually sees and filters lines live, the same
REM as running it directly.
REM
REM Run this INSTEAD of calling python.bat directly:
REM     C:\isaacsim\projects\surveillance-proj\run_simulation.bat

set PYTHONUNBUFFERED=1
C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\unified_tracking.py 2^>^&1 | findstr /v /c:"sequence size exceeds remaining buffer"
