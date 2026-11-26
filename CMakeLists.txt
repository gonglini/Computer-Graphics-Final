cmake_minimum_required(VERSION 3.20)

project(LidarDrive LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
  add_compile_options(/utf-8)
endif()

find_package(VTK REQUIRED)

# ------------------------------------------------------------------
# core: simulation without any GUI (world, vehicle, LiDAR, autopilot, crash)
# ------------------------------------------------------------------
add_library(drivecore STATIC
    src/core/RoadModel.h         src/core/RoadModel.cpp
    src/core/Palette.h
    src/core/ThreadPool.h
    src/core/SceneBuilder.h      src/core/SceneBuilder.cpp
    src/core/VehicleShapes.h     src/core/VehicleShapes.cpp
    src/core/World.h             src/core/World.cpp
    src/core/VehicleModel.h      src/core/VehicleModel.cpp
    src/core/LidarSensor.h       src/core/LidarSensor.cpp
    src/core/AutoPilot.h         src/core/AutoPilot.cpp
    src/core/CrashDeformer.h     src/core/CrashDeformer.cpp
    src/core/DriveSim.h          src/core/DriveSim.cpp
    src/core/DataRecorder.h      src/core/DataRecorder.cpp
)
target_include_directories(drivecore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(drivecore PUBLIC ${VTK_LIBRARIES})

# headless test: autopilot must finish the course without crashing
add_executable(autopilot_sim src/sim_main.cpp)
target_link_libraries(autopilot_sim PRIVATE drivecore)
vtk_module_autoinit(TARGETS autopilot_sim MODULES ${VTK_LIBRARIES})

# ------------------------------------------------------------------
# Qt application
# ------------------------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL)
set(CMAKE_AUTOMOC ON)

add_executable(LidarDrive WIN32
    src/app/main.cpp
    src/app/Quaternion.h
    src/app/MainWindow.h         src/app/MainWindow.cpp
    src/app/PidActorManager.h    src/app/PidActorManager.cpp
)
target_link_libraries(LidarDrive PRIVATE drivecore Qt6::Widgets Qt6::OpenGL)
vtk_module_autoinit(TARGETS LidarDrive MODULES ${VTK_LIBRARIES})
