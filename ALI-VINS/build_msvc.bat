@echo off
:: ALI-VINS -- build with the MSVC under E:\jo_gvil\visual studio and the CMake under E:\jo_gvil\cmake.
::
:: Usage: build_msvc.bat [OpenCV root] [Eigen root] [Ceres root] [Boost root] [jobs]
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
set "ALI_CERES=%~3"
if "%ALI_CERES%"=="" set "ALI_CERES=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres"
set "ALI_BOOST=%~4"
if "%ALI_BOOST%"=="" set "ALI_BOOST=E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0"

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
    -DOpenCV_ROOT="%ALI_OPENCV%" ^
    -DEigen3_ROOT="%ALI_EIGEN%" ^
    -DALI_CERES_ROOT="%ALI_CERES%" ^
    -DALI_BOOST_ROOT="%ALI_BOOST%" ^
    > build\configure.log 2>&1
if errorlevel 1 (
    echo CONFIGURE_FAILED
    echo   see %ALI_BUILD%\configure.log
    exit /b 1
)

if "%~5"=="" (set "ALI_JOBS=4") else (set "ALI_JOBS=%~5")
"%ALI_CMAKE%" --build build -j %ALI_JOBS% > build\build.log 2>&1
if errorlevel 1 (
    echo BUILD_FAILED
    echo   see %ALI_BUILD%\build.log
    exit /b 1
)
echo BUILD_OK
exit /b 0
