@echo off
setlocal EnableExtensions

rem ScanTracking helper aligned with CMakePresets.json / VS Open Folder.
rem Presets: win-msvc2019-qtcore-ninja-debug | win-msvc2019-qtcore-ninja-release

for %%I in ("%~dp0..") do set "ROOT=%%~fI"
set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"
set "PRESET_DEBUG=win-msvc2019-qtcore-ninja-debug"
set "PRESET_RELEASE=win-msvc2019-qtcore-ninja-release"
set "BUILD_DEBUG=%ROOT%\build\%PRESET_DEBUG%"
set "BUILD_RELEASE=%ROOT%\build\%PRESET_RELEASE%"
set "APP_DEBUG=%BUILD_DEBUG%\app"
set "APP_RELEASE=%BUILD_RELEASE%\app"

if not exist "%CMAKE%" (
  echo [scan_tracking_dev] ERROR: cmake not found: %CMAKE%
  exit /b 1
)

if "%~1"=="" goto usage

if /I "%~1"=="configure-debug"  goto configure_debug
if /I "%~1"=="configure-release" goto configure_release
if /I "%~1"=="build-debug"      goto build_debug
if /I "%~1"=="build-release"    goto build_release
if /I "%~1"=="clean-debug"      goto clean_debug
if /I "%~1"=="clean-release"    goto clean_release
if /I "%~1"=="rebuild-debug"    goto rebuild_debug
if /I "%~1"=="rebuild-release"  goto rebuild_release
if /I "%~1"=="run-debug"        goto run_debug
if /I "%~1"=="run-release"      goto run_release
goto usage

:configure_debug
echo [scan_tracking_dev] cmake --preset %PRESET_DEBUG%
"%CMAKE%" --preset %PRESET_DEBUG% -S "%ROOT%"
exit /b %ERRORLEVEL%

:configure_release
echo [scan_tracking_dev] cmake --preset %PRESET_RELEASE%
"%CMAKE%" --preset %PRESET_RELEASE% -S "%ROOT%"
exit /b %ERRORLEVEL%

:ensure_configured_debug
if exist "%BUILD_DEBUG%\build.ninja" exit /b 0
echo [scan_tracking_dev] build.ninja missing - configure Debug first
call :configure_debug
exit /b %ERRORLEVEL%

:ensure_configured_release
if exist "%BUILD_RELEASE%\build.ninja" exit /b 0
echo [scan_tracking_dev] build.ninja missing - configure Release first
call :configure_release
exit /b %ERRORLEVEL%

:build_debug
call :ensure_configured_debug
if errorlevel 1 exit /b %ERRORLEVEL%
echo [scan_tracking_dev] cmake --build --preset %PRESET_DEBUG%
"%CMAKE%" --build --preset %PRESET_DEBUG%
exit /b %ERRORLEVEL%

:build_release
call :ensure_configured_release
if errorlevel 1 exit /b %ERRORLEVEL%
echo [scan_tracking_dev] cmake --build --preset %PRESET_RELEASE%
"%CMAKE%" --build --preset %PRESET_RELEASE%
exit /b %ERRORLEVEL%

:clean_debug
echo [scan_tracking_dev] remove %BUILD_DEBUG%
if exist "%BUILD_DEBUG%" (
  rmdir /s /q "%BUILD_DEBUG%"
  if exist "%BUILD_DEBUG%" (
    echo [scan_tracking_dev] ERROR: cannot delete build dir - close running exe / VS lock and retry
    exit /b 1
  )
)
echo [scan_tracking_dev] Debug build directory cleared
exit /b 0

:clean_release
echo [scan_tracking_dev] remove %BUILD_RELEASE%
if exist "%BUILD_RELEASE%" (
  rmdir /s /q "%BUILD_RELEASE%"
  if exist "%BUILD_RELEASE%" (
    echo [scan_tracking_dev] ERROR: cannot delete build dir - close running exe / VS lock and retry
    exit /b 1
  )
)
echo [scan_tracking_dev] Release build directory cleared
exit /b 0

:rebuild_debug
call :clean_debug
if errorlevel 1 exit /b %ERRORLEVEL%
call :configure_debug
if errorlevel 1 exit /b %ERRORLEVEL%
call :build_debug
exit /b %ERRORLEVEL%

:rebuild_release
call :clean_release
if errorlevel 1 exit /b %ERRORLEVEL%
call :configure_release
if errorlevel 1 exit /b %ERRORLEVEL%
call :build_release
exit /b %ERRORLEVEL%

:run_debug
call :apply_runtime_path "%APP_DEBUG%"
cd /d "%APP_DEBUG%"
if not exist "%APP_DEBUG%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_DEBUG%\scan-tracking.exe - run build-debug first
  exit /b 1
)
echo [scan_tracking_dev] start %APP_DEBUG%\scan-tracking.exe
"%APP_DEBUG%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:run_release
call :apply_runtime_path "%APP_RELEASE%"
cd /d "%APP_RELEASE%"
if not exist "%APP_RELEASE%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_RELEASE%\scan-tracking.exe - run build-release first
  exit /b 1
)
echo [scan_tracking_dev] start %APP_RELEASE%\scan-tracking.exe
"%APP_RELEASE%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:apply_runtime_path
set "APP_DIR=%~1"
set "MVS_RT=C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"
rem CXP GenTL producer path (missing => EnumDevices 0x800000FF)
set "GENICAM_GENTL64_PATH=%MVS_RT%;%APP_DIR%;%APP_DIR%\hik_mvs_runtime"
rem inner_surface_measure_v2_runtime: VTK/OpenNI for pcl_io_release.dll (InnerSurfaceAndVolume)
set "PATH=%APP_DIR%;%APP_DIR%\inner_surface_measure_v2_runtime;%APP_DIR%\mech_eye_api;%APP_DIR%\ThirdParty;%APP_DIR%\hik_mvs_runtime;%MVS_RT%;C:\Qt\5.15.2\msvc2019_64\bin;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll_debug;%PATH%"
echo [scan_tracking_dev] cwd=%APP_DIR%
echo [scan_tracking_dev] GENICAM_GENTL64_PATH=%GENICAM_GENTL64_PATH%
if not exist "%MVS_RT%\MvFGProducerCXP.cti" if not exist "%APP_DIR%\MvFGProducerCXP.cti" (
  echo [scan_tracking_dev] WARN: MvFGProducerCXP.cti not found
)
exit /b 0

:usage
echo.
echo Usage: scan_tracking_dev.cmd ^<command^>
echo.
echo   configure-debug ^| configure-release   cmake --preset ...
echo   build-debug     ^| build-release       cmake --build --preset ... (auto-configure)
echo   clean-debug     ^| clean-release       delete build/^<preset^>
echo   rebuild-debug   ^| rebuild-release     clean + configure + build
echo   run-debug       ^| run-release         run app with runtime PATH/GenTL
echo.
echo Presets: %PRESET_DEBUG%
echo          %PRESET_RELEASE%
echo.
exit /b 1
