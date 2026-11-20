@echo off
rem Headless end-to-end test of RENAMING A PRIM FROM THE NODE GRAPH
rem (tests\testUsdviewNoodlesRename.py) on tests\fixtures\noodles_rename.usda.
rem
rem Why it exists: the USD install can carry an older copy as
rem pxr.UsdNoodles that the launcher runs instead of the in-repo editor,
rem and usdview's prim browser only follows a rename once it is rebuilt.
rem Neither is visible to the unit tests, which never start usdview.
rem
rem So this registers the in-repo editor exactly as launch_usdview.bat does
rem -- both copies present if the install has one -- opens the editor, and
rem drives the double-click, the typing and Return with real Qt events.
rem Prints RIGEXEC_NOODLES_RENAME_OK.
rem
rem Usage: run_testusdview_noodles_rename.bat [rendererDisplayName]
rem Set RIGEXEC_NOODLES_SHOT=path.png to keep a grab of the open editor.
rem
rem No build step: an open usdview holds a lock on build\rigExec.dll. Build
rem first (bin\build_rigexec.bat) so build\python\UsdNoodles is current.
setlocal EnableDelayedExpansion
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1

if not exist "%_RIGEXEC_BUILD%\python\UsdNoodles\plugInfo.json" (
    >&2 echo ERROR: build\python\UsdNoodles is not staged; run bin\build_rigexec.bat
    exit /b 1
)
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%_RIGEXEC_BUILD%\python\UsdNoodles"

set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewNoodlesRename.py" %RENDERER_ARG% "%RIG%\tests\fixtures\noodles_rename.usda"
exit /b %errorlevel%
