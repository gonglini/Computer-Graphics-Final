#include "VehicleModel.h"

#include <algorithm>
#include <cmath>

#include <vtkTransform.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

void VehicleModel::reset(double x, double y, double heading)
{
    m_x = x;
    m_y = y;
    m_heading = heading;
    m_speed = 0.0;
    m_steer = 0.0;
    m_gear = 1;
    m_rpm = idleRpm;
    m_torque = 0.0;
    m_shiftTimer = 0.0;
    m_sinceShift = 10.0;
    m_vy = m_r = m_ax = m_ay = m_axF = 0.0;
    m_roll = m_rollRate = m_bodyPitch = m_bodyPitchRate = 0.0;
    m_pitch = 0.0;
}

void VehicleModel::applyImpulse(double dvx, double dvy, double dyaw)
{
    // world velocity change -> body frame (forward / left)
    const double cx = std::cos(m_heading), cy = std::sin(m_heading);
    m_speed += dvx * cx + dvy * cy;
    m_vy += -dvx * cy + dvy * cx;
    m_r += dyaw;
}

void VehicleModel::velocity(double& vx, double& vy) const
{
    const double cx = std::cos(m_heading), cy = std::sin(m_heading);
    vx = m_speed * cx - m_vy * cy;
    vy = m_speed * cy + m_vy * cx;
}

// ---------------------------------------------------------------- drivetrain
void VehicleModel::configureDrivetrain()
{
    const double wheelOmega = maxSpeed / wheelRadius;
    const double engineOmega = peakPowerRpm * 2.0 * kPi / 60.0;
    finalDrive = engineOmega / (wheelOmega * kGearRatio[kGears - 1]);
    m_rpm = idleRpm;
}

double VehicleModel::torqueAt(double rpm) const
{
    // P = Pmax (x + x^2 - x^3)  ->  T = P / omega = Pmax / omega_p * (1 + x - x^2)
    const double x = std::max(rpm, idleRpm) / peakPowerRpm;
    const double omegaP = peakPowerRpm * 2.0 * kPi / 60.0;
    return enginePower / omegaP * std::max(0.0, 1.0 + x - x * x);
}

double VehicleModel::maxTorque() const
{
    return torqueAt(0.5 * peakPowerRpm);
}

double VehicleModel::wheelRpm(int g) const
{
    const double omega = std::abs(m_speed) / wheelRadius * kGearRatio[std::clamp(g, 1, kGears) - 1] * finalDrive;
    return omega * 60.0 / (2.0 * kPi);
}

void VehicleModel::beginShift(int newGear)
{
    newGear = std::clamp(newGear, 1, kGears);
    if (newGear == m_gear)
        return;
    m_gear = newGear;
    m_shiftTimer = 0.22;   // torque interruption while the clutches swap
    m_sinceShift = 0.0;
}

void VehicleModel::stopEngine()
{
    m_gear = 1;
    m_rpm = 0.0;
    m_torque = 0.0;
    m_shiftTimer = 0.0;
}

void VehicleModel::shiftUp()
{
    // refuse an upshift that would lug the engine below idle
    if (m_gear < kGears && wheelRpm(m_gear + 1) > idleRpm * 1.05)
        beginShift(m_gear + 1);
}

void VehicleModel::shiftDown()
{
    // refuse a downshift that would over-rev the engine
    if (m_gear > 1 && wheelRpm(m_gear - 1) < redlineRpm * 0.98)
        beginShift(m_gear - 1);
}

void VehicleModel::autoShift(double throttle, double dt)
{
    m_sinceShift += dt;
    if (std::abs(m_speed) < 1.0)
    {
        if (m_gear != 1 && m_shiftTimer <= 0.0) beginShift(1);
        return;
    }
    if (m_sinceShift < 0.6 || m_shiftTimer > 0.0)
        return;

    const double n = wheelRpm(m_gear);
    if (manualShift)
    {
        // manual mode still protects the engine: downshift instead of stalling
        if (n < idleRpm * 1.1 && m_gear > 1)
            beginShift(m_gear - 1);
        return;
    }

    const double th = std::clamp(throttle, 0.0, 1.0);
    if (th > 0.85)
    {
        // full load: the gear with the most engine power (= most wheel force at this speed)
        int best = m_gear;
        double bestP = -1.0;
        for (int g = 1; g <= kGears; ++g)
        {
            const double r = wheelRpm(g);
            if (r > redlineRpm * 0.97 || r < idleRpm)
                continue;
            const double p = torqueAt(r) * r;
            if (p > bestP) { bestP = p; best = g; }
        }
        const double curP = n <= redlineRpm * 0.97 ? torqueAt(n) * n : 0.0;
        if (best != m_gear && bestP > 1.03 * curP)
            beginShift(m_gear + (best > m_gear ? 1 : -1));
        return;
    }

    // part load: shift points rise with the pedal (early upshifts when cruising, kickdown when pressed)
    const double up = std::min(redlineRpm * 0.95, peakPowerRpm * (0.42 + 0.60 * th));
    const double down = peakPowerRpm * (0.24 + 0.35 * th);
    if (n > up && m_gear < kGears && wheelRpm(m_gear + 1) > down * 1.1)
        beginShift(m_gear + 1);
    else if (n < down && m_gear > 1 && wheelRpm(m_gear - 1) < up * 0.95)
        beginShift(m_gear - 1);
}

void VehicleModel::step(double dt, const Command& cmd)
{
    // --- steering actuator (rate limited) ---
    const double target = std::clamp(cmd.steer + wheelPull, -maxSteer, maxSteer);
    m_handbrake = cmd.handbrake;
    const double maxDelta = steerRate * dt;
    m_steer += std::clamp(target - m_steer, -maxDelta, maxDelta);

    // --- powertrain: engine -> torque converter -> 8-speed gearbox -> final drive -> rear wheels ---
    const double th = std::max(0.0, cmd.throttle);
    autoShift(th, dt);
    if (m_shiftTimer > 0.0)
        m_shiftTimer -= dt;
    const double nWheel = wheelRpm(m_gear);
    // torque converter: with the pedal down the engine can spin up to the stall speed while the
    // wheels are slow (it slips), and the slip multiplies torque (up to 2x at stall)
    const double stallRpm = idleRpm + (0.55 * peakPowerRpm - idleRpm) * th;
    const double nEngine = std::max(nWheel, std::max(idleRpm, stallRpm));
    const double slip = nEngine > 1.0 ? std::clamp(1.0 - nWheel / nEngine, 0.0, 1.0) : 0.0;
    const double tcMult = 1.0 + 1.0 * std::clamp((slip - 0.1) / 0.9, 0.0, 1.0);
    const double ratio = kGearRatio[m_gear - 1] * finalDrive;

    double Fdrive = 0.0;   // at the rear wheels [N]
    m_torque = 0.0;
    if (th > 0.0 && m_speed > -0.1)
    {
        double T = th * torqueAt(nEngine);
        if (nEngine >= redlineRpm) T = 0.0;          // rev limiter
        if (m_shiftTimer > 0.0) T *= 0.35;           // shift torque cut
        if (driverAids && m_escActive) T *= 0.25;    // ESC also cuts the engine torque while it intervenes
        m_torque = T;
        // the gear ratio multiplies the torque: low gears push much harder (up to the tire grip)
        Fdrive = T * tcMult * ratio / wheelRadius;
    }
    else if (cmd.throttle < 0.0 && m_speed <= 0.1)
        Fdrive = 1.5 * mass * cmd.throttle;          // reverse
    else if (th <= 0.0 && m_speed > 2.0)
    {
        // engine braking (fuel cut): stronger in the low gears (the ratio again)
        Fdrive = -0.06 * maxTorque() * std::min(1.0, nWheel / peakPowerRpm) * ratio / wheelRadius;
    }
    const double targetRpm = m_speed < -0.05 ? std::max(idleRpm, wheelRpm(1)) : nEngine;
    m_rpm += (targetRpm - m_rpm) * std::min(1.0, dt * (m_shiftTimer > 0.0 ? 25.0 : 12.0));
    const double brake = std::clamp(cmd.brake, 0.0, 1.0);
    const double Fbrake = brake * maxBrake * mass;   // total brake force request

    // --- chassis: dynamic single-track model, a few sub-steps ---
    const double g = 9.81;
    const double L = wheelBase, lf = frontShare * L, lr = L - lf;
    const double Iz = 1.1 * mass * lf * lr;          // yaw inertia (typical cars: m * a * b)
    const double mu = airborne ? 0.0 : std::max(0.1, surfaceMu);
    const int nSub = 4;
    const double h = dt / nSub;
    bool limited = false;
    m_escActive = false;
    for (int k = 0; k < nSub; ++k)
    {
        const double vx = m_speed, vy = m_vy, r = m_r;
        // normal loads with longitudinal weight transfer
        const double dW = mass * m_axF * cgHeight / L;   // (filtered: the suspension takes time)
        const double Fzf = std::max(0.15 * mass * g, mass * g * lr / L - dW);
        const double Fzr = std::max(0.15 * mass * g, mass * g * lf / L + dW);
        // longitudinal tire forces: drive at the rear; brakes distributed by the axle loads (EBD)
        const double sgn = vx > 0.05 ? 1.0 : (vx < -0.05 ? -1.0 : 0.0);
        const double rearShare = std::clamp(Fzr / (Fzf + Fzr), 0.2, 0.45);
        double Fxr = Fdrive - sgn * rearShare * Fbrake;
        double Fxf = -sgn * (1.0 - rearShare) * Fbrake;
        // friction circle: longitudinal force first, the rest is available for cornering.
        // ABS / TCS keep the wheels rolling (at most ~88 % of the grip used longitudinally,
        // so the tire can still build side force); without them a locked / spinning wheel uses all of it
        const double lim = driverAids ? 0.88 : 1.0;
        const double muF = mu * wheelGrip[0], muR = mu * wheelGrip[1];   // damaged wheels grip less
        // TCS: straight ahead almost all the rear grip may drive; in a corner only ~60 %, the rest keeps the rear in line
        const double corner = std::min(1.0, std::abs(m_ay) / (mu * g));   // how much grip the corner already uses
        const double limR = driverAids && Fxr > 0.0 ? std::clamp(0.92 - 0.45 * corner, 0.6, 0.92) : lim;
        if (std::abs(Fxr) > limR * muR * Fzr) { Fxr = std::copysign(limR * muR * Fzr, Fxr); limited = limited || Fdrive > 0.0; }
        if (std::abs(Fxf) > lim * muF * Fzf) Fxf = std::copysign(lim * muF * Fzf, Fxf);
        // hand brake: the rear wheels lock and slide -> all their grip is spent along the motion,
        // nothing is left for side force and the rear steps out (no ABS on the hand brake)
        if (m_handbrake && std::abs(vx) > 0.5)
            Fxr = -sgn * muR * Fzr;
        const double capF = std::sqrt(std::max(0.0, (muF * Fzf) * (muF * Fzf) - Fxf * Fxf));
        const double capR = std::sqrt(std::max(0.0, (muR * Fzr) * (muR * Fzr) - Fxr * Fxr));
        // slip angles and side forces (magic formula: B from a cornering stiffness of ~16 x Fz per rad)
        const double avx = std::max(std::abs(vx), 0.5);
        m_alphaF = std::atan2(vy + lf * r, avx) * (vx >= 0 ? 1.0 : -1.0) - m_steer * (vx >= 0 ? 1.0 : -1.0);
        m_alphaR = std::atan2(vy - lr * r, avx) * (vx >= 0 ? 1.0 : -1.0);
        const double C = 1.35;
        auto side = [&](double alpha, double cap, double Fz) {
            if (cap <= 1.0) return 0.0;
            const double B = 16.0 * Fz / (C * cap);
            return -cap * std::sin(C * std::atan(B * alpha));
        };
        const double Fyf = side(m_alphaF, capF, Fzf), Fyr = side(m_alphaR, capR, Fzr);
        // air drag + rolling resistance
        const double Fres = 0.5 * 1.2 * dragArea * vx * std::abs(vx) + (std::abs(vx) > 0.05 ? sgn * (0.0122 * mass * g + wheelDrag) : 0.0);
        const double cd = std::cos(m_steer), sd = std::sin(m_steer);
        const double fx = Fxr + Fxf * cd - Fyf * sd - Fres;
        const double fy = Fyr + Fyf * cd + Fxf * sd;
        double mz = lf * (Fyf * cd + Fxf * sd) - lr * Fyr;
        // ESC: when the car yaws faster than the tires can carry (oversteer / spin), brake one wheel
        // -> a counter yaw moment proportional to the excess yaw rate
        if (driverAids && !m_handbrake && std::abs(vx) > 3.0)
        {
            const double rMax = (mu * g * 0.95) / std::abs(vx);
            const double rRef = std::clamp(vx * std::tan(m_steer) / L, -rMax, rMax);
            const double excess = r - rRef;
            const double beta = std::atan2(vy, std::abs(vx));
            if (std::abs(excess) > 0.05 || std::abs(beta) > 0.06)
            {
                // brake the outer front / inner rear wheel: yaw moment against the excess yaw rate and the
                // body slip (beta < 0 while turning left = the rear is stepping out)
                mz -= Iz * (8.0 * excess - 5.0 * beta * (beta * r < 0.0 ? 1.0 : 0.0));
                m_escActive = true;
            }
        }
        double nvx = vx + h * (fx / mass + vy * r);
        double nvy = vy + h * (fy / mass - vx * r);
        double nr = r + h * (mz / Iz);
        // brakes cannot push the car backwards: stop
        if (Fbrake > 0.0 && Fdrive <= 0.0 && std::abs(nvx) < 0.3 && std::abs(vx) < 0.6)
            nvx = 0.0;
        // low speed: blend into the kinematic model (the slip angles are meaningless near standstill)
        const double w = airborne ? 1.0 : std::clamp((std::abs(nvx) - 1.0) / 3.0, 0.0, 1.0);
        const double rk = nvx * std::tan(m_steer) / L;
        nr = w * nr + (1.0 - w) * rk;
        nvy = w * nvy + (1.0 - w) * (lr * rk);
        m_ax = (nvx - vx) / h - vy * r;                 // longitudinal acceleration (for the weight transfer)
        m_axF += (m_ax - m_axF) * std::min(1.0, h * 8.0);
        m_ay = fy / mass * w + (1.0 - w) * nvx * rk;    // lateral acceleration
        m_speed = std::clamp(nvx, maxReverse, maxSpeed);
        m_vy = nvy;
        m_r = nr;
        // pose of the rear axle: velocity of that point = (vx, vy - lr r) in the body frame
        const double c = std::cos(m_heading), sn = std::sin(m_heading);
        const double ux = m_speed, uy = m_vy - lr * m_r;
        m_x += (ux * c - uy * sn) * h;
        m_y += (ux * sn + uy * c) * h;
        m_heading += m_r * h;
    }
    m_tractionLimited = limited;

    // --- visual body motion: roll in corners, squat / dive (spring-damper, ~1.3 Hz) ---
    const double wn = 8.0, zeta = 0.45;
    const double rollTarget = -0.010 * cgHeight * m_ay + sagRoll;  // leans to the outside of the corner
    const double pitchTarget = 0.007 * cgHeight * std::clamp(m_ax, -12.0, 12.0) + sagPitch;   // nose up when accelerating
    m_rollRate += dt * (wn * wn * (rollTarget - m_roll) - 2.0 * zeta * wn * m_rollRate);
    m_roll += dt * m_rollRate;
    m_bodyPitchRate += dt * (wn * wn * (pitchTarget - m_bodyPitch) - 2.0 * zeta * wn * m_bodyPitchRate);
    m_bodyPitch += dt * m_bodyPitchRate;
}

void VehicleModel::localToWorld(const double l[3], double w[3]) const
{
    const double phi = m_heading - kPi;
    const double c = std::cos(phi), s = std::sin(phi);
    const double cp = std::cos(m_pitch), sp = std::sin(m_pitch);
    const double dx = l[0] - kPivotX, dy = l[1] - kPivotY, dz = l[2];
    // pitch about the rear axle (local y axis), then heading
    const double rx = dx * cp + dz * sp, rz = -dx * sp + dz * cp;
    w[0] = m_x + c * rx - s * dy;
    w[1] = m_y + s * rx + c * dy;
    w[2] = m_z + rz;
}

void VehicleModel::worldToLocal(const double w[3], double l[3]) const
{
    const double phi = m_heading - kPi;
    const double c = std::cos(phi), s = std::sin(phi);
    const double cp = std::cos(m_pitch), sp = std::sin(m_pitch);
    const double dx = w[0] - m_x, dy = w[1] - m_y;
    const double rx = c * dx + s * dy, ry = -s * dx + c * dy, rz = w[2] - m_z;
    l[0] = kPivotX + rx * cp - rz * sp;
    l[1] = kPivotY + ry;
    l[2] = rx * sp + rz * cp;
}

void VehicleModel::localDirToWorld(const double l[3], double w[3]) const
{
    const double phi = m_heading - kPi;
    const double c = std::cos(phi), s = std::sin(phi);
    const double cp = std::cos(m_pitch), sp = std::sin(m_pitch);
    const double rx = l[0] * cp + l[2] * sp, rz = -l[0] * sp + l[2] * cp;
    w[0] = c * rx - s * l[1];
    w[1] = s * rx + c * l[1];
    w[2] = rz;
}

void VehicleModel::toTransform(vtkTransform* t) const
{
    t->Identity();
    t->PostMultiply();
    // body roll / pitch about the middle of the car (springs), then the road pitch about the rear axle
    t->Translate(-2.0, -0.5, -0.6);
    t->RotateX(m_roll * 180.0 / kPi);
    t->RotateY(m_bodyPitch * 180.0 / kPi);
    t->Translate(2.0 - kPivotX, 0.5 - kPivotY, 0.6);
    t->RotateY(m_pitch * 180.0 / kPi);
    t->RotateZ((m_heading - kPi) * 180.0 / kPi);
    t->Translate(m_x, m_y, m_z);
}
