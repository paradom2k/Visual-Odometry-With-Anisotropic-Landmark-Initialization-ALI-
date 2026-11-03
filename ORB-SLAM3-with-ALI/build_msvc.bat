@echo off
:: ali-orbslam3 -- build the delivered project with the MSVC and CMake of this machine.
::
:: The compiler must be MSVC: the OpenCV import libraries (opencv_*460.lib) are MSVC-built and MinGW cannot
:: consume them.  Passing the compiler explicitly is also what makes this reproducible, because it does not
:: depend on PATH order at all.
::
:: Two host problems are worked around as in the sibling projects:
::   * vcvars64.bat is not used; the environment is set directly (it aborts while loading its PowerShell
::     helper on this machine);
::   * with the session TMP/TEMP every cl invocation carrying /Zi dies with "D8050 : ... c1.dll", so TMP and
::     TEMP point at build\cltmp.
::
:: Usage:  build_msvc.bat [OpenCV root] [Eigen root] [Eigen parent] [GLFW root] [jobs]
setlocal
set "ALI_MSVC=E:\jo_gvil\visual studio\VC\Tools\MSVC\14.44.35207"
set "ALI_SDK=D:\Windows Kits\10"
set "ALI_SDKVER=10.0.26100.0"
set "ALI_CMAKE=E:\jo_gvil\cmake\bin\cmake.exe"
set "ALI_NINJA=E:\jo_gvil\visual studio\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

set "ALI_OPENCV=%~1"
if "%ALI_OPENCV%"=="" set "ALI_OPENCV=E:/jo_gvil/jo_gvil 1.0/build/opencv-install"
set "ALI_EIGEN=%~2"
if "%ALI_EIGEN%"=="" set "ALI_EIGEN=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/Eigen"
set "ALI_EIGEN_PARENT=%~3"
if "%ALI_EIGEN_PARENT%"=="" set "ALI_EIGEN_PARENT=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres/include"
set "ALI_GLFW=%~4"
if "%ALI_GLFW%"=="" set "ALI_GLFW=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/GLFW"

set "PATH=%ALI_MSVC%\bin\Hostx64\x64;%ALI_SDK%\bin\%ALI_SDKVER%\x64;%PATH%"
set "INCLUDE=%ALI_MSVC%\include;%ALI_SDK%\include\%ALI_SDKVER%\ucrt;%ALI_SDK%\include\%ALI_SDKVER%\shared;%ALI_SDK%\include\%ALI_SDKVER%\um"
set "LIB=%ALI_MSVC%\lib\x64;%ALI_SDK%\lib\%ALI_SDKVER%\ucrt\x64;%ALI_SDK%\lib\%ALI_SDKVER%\um\x64"

set "ALI_BUILD=%~dp0build"
if not exist "%ALI_BUILD%" mkdir "%ALI_BUILD%"
if not exist "%ALI_BUILD%\cltmp" mkdir "%ALI_BUILD%\cltmp"
set "TMP=%ALI_BUILD%\cltmp"
set "TEMP=%ALI_BUILD%\cltmp"

cd /d "%~dp0"
"%ALI_CMAKE%" -G Ninja -S . -B build ^
    -DCMAKE_MAKE_PROGRAM="%ALI_NINJA%" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_C_COMPILER="%ALI_MSVC%\bin\Hostx64\x64\cl.exe" ^
    -DCMAKE_CXX_COMPILER="%ALI_MSVC%\bin\Hostx64\x64\cl.exe" ^
    -DALI_OPENCV_ROOT="%ALI_OPENCV%" ^
    -DALI_EIGEN_ROOT="%ALI_EIGEN%" ^
    -DALI_EIGEN_PARENT="%ALI_EIGEN_PARENT%" ^
    -DALI_GLFW_ROOT="%ALI_GLFW%" ^
    > build\configure.log 2>&1
if errorlevel 1 (
    echo CONFIGURE_FAILED
    echo   see %ALI_BUILD%\configure.log
    exit /b 1
)

set "ALI_JOBS=%~5"
if "%ALI_JOBS%"=="" set "ALI_JOBS=8"
"%ALI_CMAKE%" --build build -j %ALI_JOBS% > build\build.log 2>&1
if errorlevel 1 (
    echo BUILD_FAILED
    echo   see %ALI_BUILD%\build.log
    exit /b 1
)

:: cmake --build has been observed to report success for a run whose link failed, so the check is what
:: makes BUILD_OK mean it.
if not exist "%ALI_BUILD%\..\bin\ali_run.exe" (
    echo BUILD_FAILED
    echo   bin\ali_run.exe was not produced; see %ALI_BUILD%\build.log
    exit /b 1
)
echo BUILD_OK
exit /b 0
