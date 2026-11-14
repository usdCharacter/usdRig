@echo off
rem Headless end-to-end test of the timeline's frame range fields and the
rem playblast (tests\testUsdviewTimeline.py) on examples\ArmShotAnim.usda.
rem The Windows twin of run_testusdview_timeline.sh. Prints
rem RIGEXEC_TIMELINE_OK.
rem
rem Usage: run_testusdview_timeline.bat [rendererDisplayName]
rem Set RIGEXEC_PLAYBLAST_KEEP=1 to keep the written frames.
call "%~dp0..\_vcvars.bat"
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
if exist "%_RIGEXEC_BUILD%\CMakeCache.txt" (
    cmake --build "%_RIGEXEC_BUILD%" >nul
    if errorlevel 1 exit /b 1
)
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewTimeline.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
