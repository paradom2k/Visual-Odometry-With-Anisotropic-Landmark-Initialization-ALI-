@echo off
:: jo_vi_corner -- build the delivered project with the same MSVC the workspace uses.
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
set "JO_MSVC=E:\jo_gvil\visual studio\VC\Tools\MSVC\14.44.35207"
set "JO_SDK=D:\Windows Kits\10"
set "JO_SDKVER=10.0.26100.0"
set "JO_CMAKE=E:\jo_gvil\cmake\bin\cmake.exe"
set "JO_NINJA=E:\jo_gvil\visual studio\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

set "JO_OPENCV=%~1"
if "%JO_OPENCV%"=="" set "JO_OPENCV=E:/jo_gvil/jo_gvil 1.0/build/opencv-install"
set "JO_EIGEN=%~2"
if "%JO_EIGEN%"=="" set "JO_EIGEN=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/Eigen"
:: Ceres and Boost come from the two roots below.  Ceres must be a built install (include/ + lib/ + bin/), because
:: the kernel links ceres.lib and its runtime DLLs are staged into bin/; a source tree is not enough.  Boost needs
:: libboost_filesystem, libboost_date_time and libboost_thread under stage/lib, and the workspace's Boost build has
:: all three once filesystem has been built (see cmake/openvins.cmake).
set "JO_CERES=%~3"
if "%JO_CERES%"=="" set "JO_CERES=E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres"
set "JO_BOOST=%~4"
if "%JO_BOOST%"=="" set "JO_BOOST=E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0"

set "PATH=%JO_MSVC%\bin\Hostx64\x64;%JO_SDK%\bin\%JO_SDKVER%\x64;%PATH%"
set "INCLUDE=%JO_MSVC%\include;%JO_SDK%\include\%JO_SDKVER%\ucrt;%JO_SDK%\include\%JO_SDKVER%\shared;%JO_SDK%\include\%JO_SDKVER%\um"
set "LIB=%JO_MSVC%\lib\x64;%JO_SDK%\lib\%JO_SDKVER%\ucrt\x64;%JO_SDK%\lib\%JO_SDKVER%\um\x64"

set "JO_BUILD=%~dp0build"
if not exist "%JO_BUILD%" mkdir "%JO_BUILD%"
if not exist "%JO_BUILD%\cltmp" mkdir "%JO_BUILD%\cltmp"
set "TMP=%JO_BUILD%\cltmp"
set "TEMP=%JO_BUILD%\cltmp"

cd /d "%~dp0"
"%JO_CMAKE%" -G Ninja -S . -B build ^
    -DCMAKE_MAKE_PROGRAM="%JO_NINJA%" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_C_COMPILER="%JO_MSVC%\bin\Hostx64\x64\cl.exe" ^
    -DCMAKE_CXX_COMPILER="%JO_MSVC%\bin\Hostx64\x64\cl.exe" ^
    -DOpenCV_ROOT="%JO_OPENCV%" ^
    -DEigen3_ROOT="%JO_EIGEN%" ^
    -DJO_CERES_ROOT="%JO_CERES%" ^
    -DJO_BOOST_ROOT="%JO_BOOST%" ^
    > build\configure.log 2>&1
if errorlevel 1 (
    echo CONFIGURE_FAILED
    echo   see %JO_BUILD%\configure.log
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
set "JO_JOBS=%~5"
if "%JO_JOBS%"=="" set "JO_JOBS=4"
"%JO_CMAKE%" --build build -j %JO_JOBS% > build\build.log 2>&1
if errorlevel 1 (
    echo BUILD_FAILED
    echo   see %JO_BUILD%\build.log
    exit /b 1
)

:: Verify the executables actually exist.  This is not belt-and-braces: cmake --build was observed to report success
:: for a run whose link had failed (LNK1104, an unusable TMP/TEMP), so "errorlevel 0" alone let a build with no
:: driver in bin/ print BUILD_OK -- which cost real time to notice.  The check is what makes BUILD_OK mean it.
if not exist "%JO_BUILD%\..\bin\jo_vi_corner_app.exe" (
    echo BUILD_FAILED
    echo   bin\jo_vi_corner_app.exe was not produced; see %JO_BUILD%\build.log
    exit /b 1
)
if not exist "%JO_BUILD%\..\bin\jo_vi_corner_openvins_run.exe" (
    echo BUILD_FAILED
    echo   bin\jo_vi_corner_openvins_run.exe was not produced; see %JO_BUILD%\build.log
    exit /b 1
)
echo BUILD_OK
exit /b 0
