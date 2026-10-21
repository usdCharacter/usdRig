@echo off
rem Verify attachment guide rods on the cloth materials example.
rem Requires a current rigExecImaging build. Optional argument: renderer name.
setlocal
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewClothGuides.py" %RENDERER_ARG% "%RIG%\docs\examples\cloth_materials.usda"
exit /b %errorlevel%
