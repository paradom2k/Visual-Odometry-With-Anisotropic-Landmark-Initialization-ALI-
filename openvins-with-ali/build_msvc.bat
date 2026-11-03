@echo off
:: openvins_ali -- build the delivered project with the same MSVC the workspace uses.
::
:: The compiler must be MSVC, not whatever is first on PATH: the OpenCV import libraries this project links
:: (opencv_*460.lib) are MSVC-built, and MinGW cannot consume them.  Passing -DCMAKE_CXX_COMPILER explicitly is
:: also what keeps this reproducible, because it does not depend on the PATH order at all.
::
:: Two host problems are worked around exactly as in the workspace's own build scripts:
::   * scripts\jo_env.bat calls vcvars64.bat, which aborts here while loading its PowerShell helper, so the
::     environment is set directly;
::   * with the session TMP/TEMP every cl invocation carrying /Zi dies with "D8050 : ... c1.dll".
::
:: Usage:  build_msvc.bat [OpenCV root] [Eigen root]
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
:: Ceres and Boost come from the two roots below.  Ceres must be a built install (include/ + lib/ + bin/), because
:: the kernel links ceres.lib and its runtime DLLs are staged into bin/; a source tree is not enough.  Boost needs
:: libboost_filesystem, libboost_date_time and libboost_thread under stage/lib, and the workspace's Boost build has
:: all three once filesystem has been built (see cmake/openvins.cmake).
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
:: Parallelism.  Measured on a 32-core machine, so that this knob is not guessed at twice:
::
::   clean build, -j 2    971.9 s
::   clean build, -j 4    791.8 s      <- the default, and the best of the three measured
::   clean build, -j 8   1077.9 s      <- more jobs is worse than fewer
::   no-op build            1.4 s
::   relink only            1.2 s      (deleted the 138 MB static lib and rebuilt: archive + link is fast)
::   recompile the driver  37.1 s      (one .cpp, but it pulls in the whole OpenVINS/Eigen/Ceres header stack)
::   recompile one kernel TU  384 s
::
:: So jobs help up to a point and then hurt -- presumably memory, since these translation units are large (32 objects
:: totalling 100 MB, the largest 13.8 MB).  But the compile phase is not the lever it looks like: an application-only
:: change costs about two seconds, and wiping build/ costs thirteen minutes.  Do not wipe build/ to test a change.
set "ALI_JOBS=%~5"
if "%ALI_JOBS%"=="" set "ALI_JOBS=4"
"%ALI_CMAKE%" --build build -j %ALI_JOBS% > build\build.log 2>&1
if errorlevel 1 (
    echo BUILD_FAILED
    echo   see %ALI_BUILD%\build.log
    exit /b 1
)

:: Verify the executables actually exist.  This is not belt-and-braces: cmake --build was observed to report success
:: for a run whose link had failed (LNK1104, an unusable TMP/TEMP), so "errorlevel 0" alone let a build with no
:: driver in bin/ print BUILD_OK -- which cost real time to notice.  The check is what makes BUILD_OK mean it.
if not exist "%ALI_BUILD%\..\bin\openvins_ali_app.exe" (
    echo BUILD_FAILED
    echo   bin\openvins_ali_app.exe was not produced; see %ALI_BUILD%\build.log
    exit /b 1
)
if not exist "%ALI_BUILD%\..\bin\openvins_ali_run.exe" (
    echo BUILD_FAILED
    echo   bin\openvins_ali_run.exe was not produced; see %ALI_BUILD%\build.log
    exit /b 1
)
echo BUILD_OK
exit /b 0
