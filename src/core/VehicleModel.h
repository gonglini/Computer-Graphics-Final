#pragma once

#include <cmath>

class vtkTransform;

// Kinematic bicycle model.
//
// Vehicle local frame = the original drive-vehicle mesh coordinates:
//   front face at x = 0, rear at x = 4, width y = 0..1, height z = 0..1, forward = -x.
// World pose (x, y, heading) is the pose of the rear-axle center (local kPivot).
// heading = world angle of the forward direction (pi = driving toward -x).
class VehicleModel
{
public:
    static constexpr double kPivotX = 3.0;   // rear axle center (between tires at x=2.5 and 3.5)
    static constexpr double kPivotY = 0.5;
    static constexpr double kLength = 4.0;
    static constexpr double kWidth  = 1.0;

    struct Command
    {
        double throttle = 0.0;  // -1..1 (negative = reverse when stopped)
        double brake    = 0.0;  // 0..1
        double steer    = 0.0;  // target steering angle [rad], + = left
        bool handbrake  = false; // locks the rear wheels (drift / hand-brake turn)
    };

    void reset(double x, double y, double heading);
    void step(double dt, const Command& cmd);

    // pose
    double x() const { return m_x; }
    double y() const { return m_y; }
    double heading() const { return m_heading; }
    double speed() const { return m_speed; }
    double steer() const { return m_steer; }
    void setSpeed(double v) { m_speed = v; }
    void setPose(double x, double y, double heading) { m_x = x; m_y = y; m_heading = heading; }
    void moveBy(double dx, double dy) { m_x += dx; m_y += dy; }

    // height of the road surface under the car (bridges, ramps, underpasses) and the pitch
    // (nose up > 0) of the car on a slope. Set by the simulation every step.
    double z() const { return m_z; }
    double pitch() const { return m_pitch; }
    void setElevation(double z, double pitch) { m_z = z; m_pitch = pitch; }

    // impact: velocity change [m/s] (world) of the center of gravity and yaw-rate change [rad/s]
    void applyImpulse(double dvx, double dvy, double dyaw);
    void velocity(double& vx, double& vy) const;   // world velocity of the center of gravity
    double slideSpeed() const { return std::abs(m_vy); }
    void clearMotion() { m_speed = 0.0; m_vy = 0.0; m_r = 0.0; }

    // ---- dynamics (read-outs) ----
    double lateralSpeed() const { return m_vy; }         // body frame, + = left [m/s]
    double yawRate() const { return m_r; }               // [rad/s]
    double longAccel() const { return m_ax; }            // [m/s^2]
    double latAccel() const { return m_ay; }             // [m/s^2] (+ = toward the left)
    double slipAngleFront() const { return m_alphaF; }   // [rad]
    double slipAngleRear() const { return m_alphaR; }
    bool   escActive() const { return m_escActive; }
    bool   handbrakeOn() const { return m_handbrake; }
    // landing / bump: kicks the body on its springs (pitch and roll rate) [rad/s]
    void   kickBody(double pitchRate, double rollRate) { m_bodyPitchRate += pitchRate; m_rollRate += rollRate; }
    bool tractionLimited() const { return m_tractionLimited; }   // drive force cut to the tire grip
    double bodyRoll() const { return m_roll; }           // visual body roll / pitch [rad]
    double bodyPitch() const { return m_bodyPitch; }

    // frame conversions (with the elevation and the pitch)
    void localToWorld(const double local[3], double world[3]) const;
    void worldToLocal(const double world[3], double local[3]) const;
    void localDirToWorld(const double local[3], double world[3]) const;

    // actor transform: world = T(x, y, z) * Rz(heading - pi) * Ry(pitch) * [body roll / pitch] * T(-pivot)
    void toTransform(vtkTransform* t) const;

    // parameters
    double wheelBase   = 2.5;
    double maxSteer    = 0.55;   // [rad] ~31 deg
    double steerRate   = 1.6;    // [rad/s]
    // 3.5 t light truck, 190 kW: 0 -> 100 km/h in about 12 s, top speed about 140 km/h
    double mass        = 3500.0; // [kg]
    double enginePower = 190e3;  // [W]
    double maxAccel    = 3.0;    // [m/s^2] traction limit
    double dragArea    = 4.5;    // Cd * A [m^2]
    double maxBrake    = 8.0;    // [m/s^2]
    double maxSpeed    = 40.0;   // [m/s] = 144 km/h
    double maxReverse  = -5.0;   // [m/s]

    // ---- dynamic single-track ("bicycle") model ----
    // states: forward speed vx, lateral speed vy, yaw rate r at the center of gravity.
    // Tire side force from the slip angle (simplified Pacejka "magic formula"), normal loads with
    // longitudinal weight transfer, friction circle per axle (drive / brake force and side force share
    // mu * Fz), rear-wheel drive. Below ~3 m/s it blends into the kinematic model (no singularity).
    double cgHeight    = 1.0;    // [m]
    bool   driverAids  = true;    // ABS + EBD + TCS + ESC (as in every modern car)
    // damaged wheels / axles (set by the crash model): grip of the front / rear axle (1 = intact),
    // steering pull [rad] (bent toe / dragging wheel), extra rolling drag [N], body sag of a lost wheel [rad]
    bool   airborne    = false;   // all four wheels off the ground: no tire forces
    double wheelGrip[2] = { 1.0, 1.0 };
    double wheelPull   = 0.0;
    double wheelDrag   = 0.0;
    double sagRoll     = 0.0;
    double sagPitch    = 0.0;
    double frontShare  = 0.45;   // lf / wheelBase (static load on the rear = frontShare)
    double surfaceMu   = 1.0;    // friction coefficient (dry asphalt 1.0, grass ~0.6), set by the simulation

    // ---- engine + 8-speed automatic transmission (ZF 8HP-like ratios) ----
    // Engine power curve P(n) = Pmax (x + x^2 - x^3), x = n / peakPowerRpm (classic polynomial fit):
    // torque peaks at half the peak-power rpm. Drive force = T * gear * final / r_wheel.
    // At low speed a torque converter lets the engine run above the wheel speed and multiplies torque.
    static constexpr int kGears = 8;
    static constexpr double kGearRatio[kGears] = { 4.71, 3.14, 2.11, 1.67, 1.29, 1.00, 0.84, 0.67 };
    double idleRpm      = 650.0;
    double peakPowerRpm = 3200.0;
    double redlineRpm   = 3800.0;
    double wheelRadius  = 0.38;   // [m]
    double finalDrive   = 4.7;    // set by configureDrivetrain()
    // final drive so that 8th gear reaches maxSpeed at the peak-power rpm
    void configureDrivetrain();

    bool manualShift = false;     // false: automatic (D), true: manual (M) with shiftUp/shiftDown
    void stopEngine();             // after a crash: engine off, gearbox back to 1st
    void shiftUp();
    void shiftDown();
    int gear() const { return m_gear; }               // 1..8
    bool reverse() const { return m_speed < -0.05; }
    double rpm() const { return m_rpm; }              // engine speed (smoothed for display)
    double engineTorque() const { return m_torque; }  // [Nm] current output
    bool shifting() const { return m_shiftTimer > 0.0; }
    double maxTorque() const;                         // [Nm] peak of the curve
    double torqueAt(double rpm) const;                // full-load torque [Nm]

private:
    double wheelRpm(int gear) const;   // engine rpm if gear 'gear' were coupled at the current speed
    void autoShift(double throttle, double dt);
    void beginShift(int newGear);
    double m_z = 0.0, m_pitch = 0.0;
    double m_vy = 0.0, m_r = 0.0;                 // lateral speed, yaw rate
    double m_ax = 0.0, m_ay = 0.0;
    double m_alphaF = 0.0, m_alphaR = 0.0;
    double m_axF = 0.0;          // filtered longitudinal acceleration (weight transfer)
    bool   m_escActive = false;
    bool   m_handbrake = false;
    bool m_tractionLimited = false;
    double m_roll = 0.0, m_rollRate = 0.0, m_bodyPitch = 0.0, m_bodyPitchRate = 0.0;
    int m_gear = 1;
    double m_rpm = 650.0;
    double m_torque = 0.0;
    double m_shiftTimer = 0.0;
    double m_sinceShift = 10.0;

    double m_x = 0.0;
    double m_y = 0.0;
    double m_heading = 3.14159265358979323846;
    double m_speed = 0.0;
    double m_steer = 0.0;
};
