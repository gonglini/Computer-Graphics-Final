@echo off
rem Build LidarDrive (VTK 9.6 + Qt 6.10.2, MSVC 2022, Ninja)
rem The build folder is outside the project on purpose: CMake crashed when regenerating
rem inside a folder whose path contains Korean characters.
setlocal
set "VTK_DIR=C:\dev\VTK\vtk-install"
set "QT_DIR=C:\Qt\6.10.2\msvc2022_64"
set "BUILD_DIR=C:\dev\lidar-drive-build"

call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set "CMK=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake"
set "PATH=%CMK%\CMake\bin;%CMK%\Ninja;%VTK_DIR%\bin;%QT_DIR%\bin;%PATH%"

cmake -S "%~dp0." -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=%QT_DIR%;%VTK_DIR%" || exit /b 1
cmake --build "%BUILD_DIR%" || exit /b 1
echo BUILD_OK
