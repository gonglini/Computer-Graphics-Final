@echo off
rem Run LidarDrive with VTK and Qt DLLs on the PATH
set "PATH=C:\dev\VTK\vtk-install\bin;C:\Qt\6.10.2\msvc2022_64\bin;%PATH%"
start "" "C:\dev\lidar-drive-build\LidarDrive.exe"
