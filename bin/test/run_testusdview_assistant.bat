@echo off
rem Headless end-to-end test of the AI Assistant
rem (tests\testUsdviewAssistant.py) on examples\biped\Biped_anim_ikfk.usda.
rem The Windows twin of run_testusdview_assistant.sh. Prints
rem RIGEXEC_ASSISTANT_OK.
rem
rem Usage: run_testusdview_assistant.bat [rendererDisplayName]
rem Set RIGEXEC_ASSISTANT_SHOT=path\prefix to keep the settings and window
rem grabs.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewAssistant.py" %RENDERER_ARG% "%RIG%\examples\biped\Biped_anim_ikfk.usda"
exit /b %errorlevel%
