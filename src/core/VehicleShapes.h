#pragma once

#include "SceneBuilder.h"

// Vehicle meshes in the vehicle local frame
// (front face at x = 0, rear at x = 4, width y = 0..1, forward = -x, ground at z = -0.1).
// Every player vehicle keeps the footprint of the original drive-vehicle example (4 x 1) and the
// same four PIDs, so collision, crash deformation, LiDAR and the UI work for all of them.
namespace VehicleShapes
{
struct Parts
{
    SceneBuilder::Grid cab;      // PID 1: front body / cab + bumper + lights + mirrors (deformable)
    SceneBuilder::Grid cargo;    // PID 2: rear body / cargo box + chassis (deformable)
    SceneBuilder::Grid wheels;   // PID 3
    SceneBuilder::Grid sensors;  // PID 4: roof LiDAR + front camera
    std::vector<double> axles;   // axle positions (local x) of the wheels in 'wheels'
    double wheelZ = 0.2;         // hub height
    double wheelR = 0.3;         // tire radius (mesh units)
};
using Truck = Parts;

enum Type { TypeTruck = 0, TypeSedan, TypeSuv, TypeSports, TypeCount };

// physical and sensor data of a player vehicle type
struct Spec
{
    const char* name;      // English (HUD)
    const char* nameKo;    // Korean (UI)
    double mass;           // [kg]
    double power;          // [W]
    double maxAccel;       // traction limit [m/s^2]
    double dragArea;       // Cd * A [m^2]
    double maxSpeed;       // [m/s]
    double maxBrake;       // [m/s^2]
    double wheelBase;      // [m]
    double lidarMount[3];  // LiDAR origin (vehicle frame)
    double frontCam[3];    // front camera origin
    double eye[3];         // driver's eye (first person view)
    // engine / 8-speed automatic
    double idleRpm;
    double peakPowerRpm;   // rpm of maximum power
    double redlineRpm;
    double wheelRadius;    // [m] (real size; the mesh is scaled to the 4 x 1 frame)
    // chassis (dynamics)
    double cgHeight;       // [m]
    double frontShare;     // lf / wheelbase
};
const Spec& spec(int type);

Parts buildVehicle(int type);
Truck buildTruck();

// Sedan for traffic (not deformable). paint = Palette index.
SceneBuilder::Grid buildSedan(int paint);
}
