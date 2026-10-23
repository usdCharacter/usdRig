@echo off
rem Exercise Execution Stack on a cache-owned viewer stage: compiled order,
rem compiler diagnostics, registration and filters.
rem Usage: run_testusdview_execstack.bat [rendererDisplayName]
rem Set RIGEXEC_EXECSTACK_SHOT=path.png to keep a window grab.
call "%~dp0..\_vcvars.bat"
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
if exist "%RIG%\build\CMakeCache.txt" (
    cmake --build "%RIG%\build" >nul
    if errorlevel 1 exit /b 1
)
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewExecStack.py" %RENDERER_ARG% "%RIG%\biped_rigged.usda"
exit /b %errorlevel%
