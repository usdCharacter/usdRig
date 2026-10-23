@echo off
rem Exercise Control Picker selection, channel edits and control visibility.
rem Usage: run_testusdview_picker.bat [rendererDisplayName]
rem Set RIGEXEC_PICKER_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewPicker.py" %RENDERER_ARG% "%RIG%\examples\biped\Biped_stack.usda"
exit /b %errorlevel%
