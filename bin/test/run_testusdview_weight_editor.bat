@echo off
rem Headless check of the volume weight editor in a REAL usdview viewport:
rem Hydra preview, queued commit, undo and cancel (tests\testUsdviewWeightEditor.py).
rem Usage: run_testusdview_weight_editor.bat [rendererDisplayName]   (e.g. Embree)
call "%~dp0..\_vcvars.bat"
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
cmake --build "%RIG%\build" >nul
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewWeightEditor.py" %RENDERER_ARG% "%RIG%\examples\11_VolumeWeights.usda"
exit /b %errorlevel%
