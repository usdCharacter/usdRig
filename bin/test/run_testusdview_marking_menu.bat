@echo off
rem Headless end-to-end test of the right-click marking menu
rem (tests\testUsdviewMarkingMenu.py) on examples\biped\Biped_stack.usda.
rem The Windows twin of run_testusdview_marking_menu.sh.
rem
rem Usage: run_testusdview_marking_menu.bat [rendererDisplayName]  (e.g. Embree)
rem Set RIGEXEC_MARKING_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewMarkingMenu.py" %RENDERER_ARG% "%RIG%\examples\biped\Biped_stack.usda"
exit /b %errorlevel%
