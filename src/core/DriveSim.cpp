#include "DriveSim.h"

#include <algorithm>
#include <cmath>

#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>

#include "VehicleShapes.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

void DriveSim::Npc::toTransform(vtkTransform* t) const
{
    // same local frame as the player vehicle; (2, 0.5) is the center of the car
    t->Identity();
    t->PostMultiply();
    t->Translate(-2.0, -0.5, -0.55);                // body roll / pitch about the center of gravity
    t->RotateX(roll * 180.0 / kPi);
    t->RotateY(bodyPitch * 180.0 / kPi);
    t->Translate(0.0, 0.0, 0.55);
    t->RotateY(pitch * 180.0 / kPi);
    t->RotateZ((heading - kPi) * 180.0 / kPi);
    t->Translate(cx, cy, z);
}

void DriveSim::Npc::velocity(double& wx, double& wy) const
{
    const double c = std::cos(heading), s = std::sin(heading);
    if (free)
    {
        wx = vx * c - vy * s;
        wy = vx * s + vy * c;
    }
    else
    {
        wx = speed * c;
        wy = speed * s;
    }
}

namespace
{
// traffic car: 1450 kg, 110 kW. Uphill / at speed the engine runs out of power: a = P / (m v)
double npcAccel(double v) { return std::min(2.6, 0.55 * 110e3 / (1450.0 * std::max(v, 3.0))); }

// suspension: the body leans out of corners and dives / squats (same spring as the player car)
void npcBody(DriveSim::Npc& n, double dt)
{
    if (dt <= 0.0)
        return;
    const double wn = 8.0, zeta = 0.45, h = 0.55;
    const double rollT = -0.010 * h * std::clamp(n.ay, -12.0, 12.0);
    const double pitchT = 0.007 * h * std::clamp(n.ax, -12.0, 12.0);
    n.rollRate += dt * (wn * wn * (rollT - n.roll) - 2.0 * zeta * wn * n.rollRate);
    n.roll += dt * n.rollRate;
    n.bodyPitchRate += dt * (wn * wn * (pitchT - n.bodyPitch) - 2.0 * zeta * wn * n.bodyPitchRate);
    n.bodyPitch += dt * n.bodyPitchRate;
}

// a car that was hit: single-track model, wheels straight, the driver stands on the brake (ABS).
// The tires stop the sideways slide and the spin with Coulomb friction, like the player's car.
void npcFreeStep(DriveSim::Npc& n, double dt)
{
    const double m = 1450.0, g = 9.81, L = 2.7, lf = 1.2, lr = 1.5, Iz = 1.1 * m * lf * lr, hcg = 0.55;
    const double mu = 0.9;
    const int nSub = 4;
    const double h = dt / nSub;
    for (int k = 0; k < nSub; ++k)
    {
        const double vx = n.vx, vy = n.vy, r = n.r;
        if (std::hypot(vx, vy) < 0.25 && std::abs(r) < 0.3)
        {
            n.vx = n.vy = 0.0;
            n.r *= 0.8;
            n.ax = n.ay = 0.0;
            break;
        }
        const double dW = m * std::clamp(n.ax, -9.0, 9.0) * hcg / L;
        const double Fzf = std::max(0.2 * m * g, m * g * lr / L - dW);
        const double Fzr = std::max(0.2 * m * g, m * g * lf / L + dW);
        const double sgn = vx > 0.05 ? 1.0 : (vx < -0.05 ? -1.0 : 0.0);
        const double Fxf = -sgn * 0.8 * mu * Fzf, Fxr = -sgn * 0.8 * mu * Fzr;   // ABS braking
        const double capF = std::sqrt(std::max(0.0, mu * Fzf * mu * Fzf - Fxf * Fxf));
        const double capR = std::sqrt(std::max(0.0, mu * Fzr * mu * Fzr - Fxr * Fxr));
        const double avx = std::max(std::abs(vx), 0.5);
        const double aF = std::atan2(vy + lf * r, avx), aR = std::atan2(vy - lr * r, avx);
        auto side = [](double alpha, double cap, double Fz) {
            const double C = 1.35, B = 16.0 * Fz / (C * std::max(cap, 1.0));
            return -cap * std::sin(C * std::atan(B * alpha));
        };
        const double Fyf = side(aF, capF, Fzf), Fyr = side(aR, capR, Fzr);
        const double fx = Fxf + Fxr, fy = Fyf + Fyr, mz = lf * Fyf - lr * Fyr;
        n.vx = vx + h * (fx / m + vy * r);
        n.vy = vy + h * (fy / m - vx * r);
        n.r = r + h * (mz / Iz);
        if (vx * n.vx < 0.0 && std::abs(vx) < 1.0)
            n.vx = 0.0;   // friction stops, it does not reverse
        n.ax = fx / m;
        n.ay = fy / m;
        const double c = std::cos(n.heading), s = std::sin(n.heading);
        n.cx += (n.vx * c - n.vy * s) * h;
        n.cy += (n.vx * s + n.vy * c) * h;
        n.heading += n.r * h;
    }
    n.speed = std::hypot(n.vx, n.vy);
    npcBody(n, dt);
}
}

void DriveSim::init()
{
    world.build();
    startS = world.startS();
    startLane = world.startLane();
    lidar.setObstacleSurface(world.obstacleSurface());
    lidar.setGroundZ(world.road().groundZ);
    lidar.setGroundMaterial([this](double x, double y) { return world.groundMaterial(x, y); });
    lidar.setGroundHole([this](double x, double y) { return world.groundHole(x, y); });
    lidar.setRoadOnHills([this](double x, double y) { const int m = world.netMask(x, y); return m >= 2; });
    lidar.setHeightField(world.heightField());
    lidar.setRoadProjector([this](double x, double y, double& s, double& lat) {
        return world.road().project(x, y, s, lat);
    });
    pilot.sensorRange = lidar.config.range;

    deformer.addPart(world.grid(World::PidCab));
    deformer.addPart(world.grid(World::PidCargo));
    vehicle.configureDrivetrain();
    setupWheels();

    reset();
}

void DriveSim::setVehicleType(int type)
{
    lidar.sync();
    world.setVehicleType(type);
    const VehicleShapes::Spec& sp = VehicleShapes::spec(type);
    vehicle.mass = sp.mass;
    vehicle.enginePower = sp.power;
    vehicle.maxAccel = sp.maxAccel;
    vehicle.dragArea = sp.dragArea;
    vehicle.maxSpeed = sp.maxSpeed;
    vehicle.maxBrake = sp.maxBrake;
    vehicle.wheelBase = sp.wheelBase;
    vehicle.idleRpm = sp.idleRpm;
    vehicle.peakPowerRpm = sp.peakPowerRpm;
    vehicle.redlineRpm = sp.redlineRpm;
    vehicle.wheelRadius = sp.wheelRadius;
    vehicle.cgHeight = sp.cgHeight;
    vehicle.frontShare = sp.frontShare;
    vehicle.configureDrivetrain();
    for (int k = 0; k < 3; ++k)
        lidar.config.mount[k] = sp.lidarMount[k];
    // the deformer keeps a rest copy of the meshes: start over with the new ones
    deformer = CrashDeformer();
    deformer.addPart(world.grid(World::PidCab));
    deformer.addPart(world.grid(World::PidCargo));
    setupWheels();
    reset();
}

void DriveSim::updateSignalAhead()
{
    SignalAhead& sa = signalAhead;
    const double frontLocal[3] = { 0.0, 0.5, 0.0 };
    double f[3], sF, latF;
    vehicle.localToWorld(frontLocal, f);
    sa = SignalAhead{};
    if (!world.road().project(f[0], f[1], sF, latF) || latF < -1.0)
    {
        m_prevSignalJ = -1;
        return;
    }
    double d = 1e9;
    const int j = world.nextStopLine(sF, 1, 150.0, d, -3.0);
    if (j < 0)
    {
        m_prevSignalJ = -1;
        return;
    }
    sa.junction = j;
    sa.distance = d;
    sa.light = world.signalAt(j, 1, time);
    sa.remaining = world.signalRemaining(j, 1, time);
    const double v = std::max(0.0, vehicle.speed());
    // red (also with the left arrow: we go straight) -> stop; yellow -> stop only if that is comfortable
    sa.mustStop = d > -0.5 && (sa.light == Light::Red || sa.light == Light::RedArrow ||
                               (sa.light == Light::Yellow && d > v * v / (2.0 * 3.0) + 0.5));

    // crossing the stop line on red = violation
    if (j == m_prevSignalJ && m_prevSignalD > 0.0 && d <= 0.0 && v > 0.5 &&
        (sa.light == Light::Red || sa.light == Light::RedArrow))
    {
        ++redLightViolations;
        lastViolationTime = time;
    }
    m_prevSignalJ = j;
    m_prevSignalD = d;
}

void DriveSim::reset()
{
    {
        const RoadModel& r = world.road();
        // the vehicle pose is the rear axle (local x = 3): 1 m behind the car's center
        double x, y;
        if (startNetRoad >= 0)
        {
            const RoadModel& rr = world.net()[startNetRoad].road;
            const int lane = std::min(startLane, rr.lanes - 1);
            rr.point(startNetS - 1.0, rr.laneOffset(lane), x, y);
            vehicle.reset(x, y, rr.heading(startNetS));
        }
        else
        {
            r.point(startS - 1.0, r.laneOffset(startLane), x, y);
            vehicle.reset(x, y, r.heading(startS));
        }
    }
    pilot.reset();
    pilot.setTargetLane(startLane);
    deformer.reset();
    resetWheels();
    airborne = false;
    vehicle.airborne = false;
    m_groundValid = false;
    m_vz = 0.0;
    airTime = 0.0;
    world.resetProps();
    lidar.clear();
    crash = CrashInfo{};
    report = CrashReport{};
    lastImpact = ImpactInfo{};
    impactLog.clear();
    damagePct = 0.0;
    impacts = 0;
    time = 0.0;
    distance = 0.0;
    redLightViolations = 0;
    lastViolationTime = -100.0;
    m_prevSignalJ = -1;
    m_prevSignalD = 1e9;
    signalAhead = SignalAhead{};
    path.clear();
    path.push_back({ vehicle.x(), vehicle.y(), vehicle.heading() });
    lastCommand = {};

    npcs.clear();
    for (const auto& s : world.npcSpawns())
    {
        Npc n;
        n.dir = s.dir;
        n.lane = s.lane;
        n.cruise = s.cruise;
        n.road = s.road;
        // personal speed factor 0.85 .. 1.12 of the limit
        n.factor = 0.85 + 0.27 * std::fmod(s.s * 0.618 + s.lane * 0.37 + (s.dir > 0 ? 0.0 : 0.5), 1.0);
        placeNpc(n, s.s);
        npcs.push_back(n);
    }
    for (auto& n : npcs)
        if (n.road >= 1)
        {
            n.active = true;   // the street network has no respawning: its cars drive from junction to junction
            n.speed = 0.8 * world.net()[n.road - 1].limit * n.factor;
        }
    vehicle.setElevation(0.0, 0.0);
    updateSurface();
    updateTraffic(0.0);

    // start with one complete revolution so the first frame already has a full cloud
    lidar.fullScan(vehicle, dynamicBoxes());
}

void DriveSim::placeNpc(Npc& n, double s) const
{
    const RoadModel& road = n.road ? world.net()[n.road - 1].road : world.road();
    n.s = road.wrap(s);
    road.point(n.s, n.dir * road.laneOffset(n.lane), n.cx, n.cy);
    n.heading = road.heading(n.s, n.dir);
    n.z = road.elevation(n.s);
    n.pitch = std::atan(road.grade(n.s) * n.dir);
}

const char* DriveSim::surfaceNameKo() const
{
    switch (surface)
    {
    case Surface::Net:
    {
        if (netRoad < 0)
            return netNode >= 0 ? world.nodes()[netNode].nameKo : u8"교차로";
        switch (world.netStructure(netRoad, netS))
        {
        case World::NetViaduct: return u8"북부 고가도로 (고속도로 위)";
        case World::NetTrench:  return u8"동부 지하차도";
        case World::NetCovered: return u8"동부 지하차도 (교외 도로 아래)";
        default: break;
        }
        return world.net()[netRoad].nameKo;
    }
    case Surface::Connector: return u8"도심 → 공원 연결 도로";
    case Surface::Offroad:   return u8"도로 밖";
    default: break;
    }
    return "";
}

void DriveSim::updateSurface()
{
    // which road is under the car: the main loop (always at ground level), a street of the network
    // (viaduct, ramps, underpass, junctions), a connector street, or none. Where two overlap (the
    // crossings) the one closest to the current height wins: no jumping from the viaduct onto the highway.
    const double c[3] = { 2.0, 0.5, 0.0 };
    double w[3];
    vehicle.localToWorld(c, w);
    const RoadModel& main = world.road();
    const double zPrev = vehicle.z();
    double ms = 0, ml = 1e9;
    const bool mainOk = main.project(w[0], w[1], ms, ml);
    const bool onMain = mainOk && std::abs(ml) < main.halfWidth + 3.0;
    const bool onConn = mainOk && World::inConnector(ms, ml);
    World::NetPos np;
    const bool onNet = world.netLocate(w[0], w[1], zPrev, np);

    bool useNet = onNet;
    if (onNet && (onMain || onConn))
        useNet = std::abs(np.z - zPrev) < std::abs(0.0 - zPrev);

    double z = 0.0, pitch = 0.0;
    netRoad = -1;
    netNode = -1;
    if (useNet)
    {
        surface = Surface::Net;
        z = np.z;
        netRoad = np.road;
        netNode = np.node;
        netS = np.s;
        if (np.road >= 0)
        {
            const RoadModel& rr = world.net()[np.road].road;
            pitch = std::atan(rr.grade(np.s) * std::cos(vehicle.heading() - rr.heading(np.s)));
        }
    }
    else if (onMain)
        surface = Surface::Main;
    else if (onConn)
        surface = Surface::Connector;
    else
    {
        surface = Surface::Offroad;
        z = std::max(0.0, world.terrainHeight(w[0], w[1]));
        const double hx = std::cos(vehicle.heading()), hy = std::sin(vehicle.heading());
        const double ahead = std::max(0.0, world.terrainHeight(w[0] + hx, w[1] + hy));
        const double back = std::max(0.0, world.terrainHeight(w[0] - hx, w[1] - hy));
        pitch = std::atan(0.5 * (ahead - back));
    }
    m_groundZ = z;
    m_groundPitch = pitch;
    if (airborne)
        vehicle.setElevation(m_airZ, m_airPitch);
    else
        vehicle.setElevation(z, pitch);
    // grip: roads and junctions dry asphalt, grass / dirt off the road
    vehicle.surfaceMu = surface == Surface::Offroad ? 0.6 : 1.0;
}

void DriveSim::egoRoadPosition(double& s, double& lat) const
{
    const double c[3] = { 2.0, 0.5, 0.0 };
    double w[3];
    vehicle.localToWorld(c, w);
    if (!world.road().project(w[0], w[1], s, lat))
    {
        s = 0.0;
        lat = 1e9;
    }
}

bool DriveSim::respawnAhead(size_t i, double egoS)
{
    Npc& n = npcs[i];
    const RoadModel& road = world.road();
    for (double d = 260.0; d <= 380.0; d += 20.0)
    {
        const double s = road.wrap(egoS + d);   // ahead of the player
        if (!world.trafficAllowed(s, n.dir))
            continue;
        bool free = true;
        for (size_t j = 0; j < npcs.size(); ++j)
            if (j != i && npcs[j].alive && npcs[j].road == 0 && npcs[j].dir == n.dir && npcs[j].lane == n.lane &&
                std::abs(road.diff(npcs[j].s, s)) < 15.0)
                free = false;
        if (!free)
            continue;
        placeNpc(n, s);
        n.speed = 0.8 * world.speedLimit(s) * n.factor;
        n.lastSpeed = n.speed;
        n.crashedFor = 0.0;
        n.free = false;
        n.vx = n.vy = n.r = n.ax = n.ay = 0.0;
        n.roll = n.rollRate = n.bodyPitch = n.bodyPitchRate = 0.0;
        n.active = true;
        n.alive = true;
        return true;
    }
    n.alive = false;
    n.active = false;
    n.cx = 1.0e4 + 10.0 * i;   // out of the world (no collision, no LiDAR returns)
    n.cy = 1.0e4;
    n.speed = 0.0;
    return false;
}

void DriveSim::updateTraffic(double dt)
{
    const RoadModel& road = world.road();
    double egoS, egoLat;
    egoRoadPosition(egoS, egoLat);

    // is the player inside lane 'lane' of direction 'dir'? (only when down on the main road)
    const bool egoOnMain = surface == Surface::Main;
    auto egoInLane = [&](int dir, int lane) {
        return egoOnMain && std::abs(egoLat - dir * road.laneOffset(lane)) < 1.6;
    };

    for (size_t i = 0; i < npcs.size(); ++i)
    {
        Npc& n = npcs[i];
        if (!trafficEnabled)
        {
            n.alive = false;
            n.cx = 1.0e4 + 10.0 * i;   // parked out of the world
            n.cy = 1.0e4;
            continue;
        }
        if (n.road >= 1)
        {
            updateNetNpc(n, i, dt);
            continue;
        }

        // recycle cars that are out of play: left far behind by the player, passed (oncoming),
        // or about to enter the obstacle course (player direction)
        if (!crash.happened && dt > 0.0)
        {
            const double rel = road.diff(n.s, egoS);   // > 0: ahead of the player along the loop
            // recycled only far out of sight (the top / cinematic cameras see a few hundred meters)
            const bool behind = n.dir > 0 ? rel < -260.0 : rel < -200.0;
            const bool course = n.dir > 0 && !world.trafficAllowed(n.s + 12.0, 1);
            if (!n.alive || behind || course)
            {
                respawnAhead(i, egoS);
                if (!n.alive)
                    continue;
            }
        }
        if (!n.alive)
            continue;

        // hit by the player: slides / spins as a free body until it stops, then waits with the hazard
        // lights on; it rejoins its lane only when the player is far away (no visible jump)
        if (n.free)
        {
            npcFreeStep(n, dt);
            double ps, pl;
            if (road.project(n.cx, n.cy, ps, pl))
                n.s = ps;
            n.crashedFor = std::max(0.0, n.crashedFor - dt);
            if (n.crashedFor <= 0.0 && n.speed < 0.1 && std::hypot(n.cx - vehicle.x(), n.cy - vehicle.y()) > 90.0)
            {
                n.free = false;
                n.speed = n.lastSpeed = 0.0;
                n.roll = n.bodyPitch = n.rollRate = n.bodyPitchRate = 0.0;
                placeNpc(n, n.s);
            }
            continue;
        }

        // start driving when the player gets close
        if (!n.active && !crash.happened && std::abs(road.diff(n.s, egoS)) < 200.0)
            n.active = true;

        // gap to whatever is ahead in the same lane: other cars, the player
        double gap = 1e9;
        for (size_t j = 0; j < npcs.size(); ++j)
        {
            const Npc& o = npcs[j];
            if (j == i || !o.alive || o.road != 0 || o.dir != n.dir || o.lane != n.lane)
                continue;
            const double ahead = n.dir * road.diff(o.s, n.s);   // > 0 if o is ahead of n
            if (ahead > 0.0)
                gap = std::min(gap, ahead - 2.0 * kNpcHalfL);
        }
        // traffic light ahead: stop about 1 m before the stop line
        {
            double d;
            const int j = world.nextStopLine(n.s + n.dir * kNpcHalfL, n.dir, 60.0, d, -0.3);
            if (j >= 0)
            {
                const Light L = world.signalAt(j, n.dir, time);
                const bool stop = L == Light::Red || L == Light::RedArrow ||
                                  (L == Light::Yellow && d > n.speed * n.speed / (2.0 * 3.5) + 1.0);
                if (stop)
                    gap = std::min(gap, d + 2.0);
            }
        }
        if (egoInLane(n.dir, n.lane))
        {
            const double ahead = n.dir * road.diff(egoS, n.s);
            if (ahead > 0.0)
                gap = std::min(gap, ahead - kNpcHalfL - 2.1);
        }

        // cruise at the speed limit of the section (each driver a little faster or slower),
        // slower in the corners (comfortable lateral acceleration 2 m/s^2)
        double vmax = world.speedLimit(n.s) * n.factor;
        if (n.crashedFor > 0.0)
        {
            n.crashedFor -= dt;
            vmax = 0.0;   // a car that was hit stays where it is for a while
        }
        for (double d = 0.0; d <= 40.0; d += 10.0)
        {
            const double k = std::abs(road.curvature(n.s + n.dir * d));
            if (k > 1e-6)
                vmax = std::min(vmax, std::sqrt(2.0 / k + 2.0 * 2.0 * d));
        }
        const double target = n.active ? std::clamp(0.9 * (gap - 3.0), 0.0, vmax) : 0.0;
        n.speed += std::clamp(target - n.speed, -6.0 * dt, npcAccel(n.speed) * dt);
        if (gap < 0.5)
            n.speed = 0.0;

        // follow the lane center
        placeNpc(n, n.s + n.dir * n.speed * dt);
        if (dt > 0.0)
        {
            n.ax = (n.speed - n.lastSpeed) / dt;
            n.ay = n.speed * n.speed * road.curvature(n.s) * n.dir;
            n.lastSpeed = n.speed;
            npcBody(n, dt);
        }
    }
}

void DriveSim::updateNetNpc(Npc& n, size_t i, double dt)
{
    const auto& net = world.net();
    const auto& nodes = world.nodes();

    if (n.free)
    {
        npcFreeStep(n, dt);
        const RoadModel& rd = net[n.road - 1].road;
        double ps, pl;
        if (rd.project(n.cx, n.cy, ps, pl))
            n.s = std::clamp(ps, 0.0, rd.length());
        n.crashedFor = std::max(0.0, n.crashedFor - dt);
        if (n.crashedFor <= 0.0 && n.speed < 0.1 && std::hypot(n.cx - vehicle.x(), n.cy - vehicle.y()) > 90.0)
        {
            n.free = false;
            n.speed = n.lastSpeed = 0.0;
            n.roll = n.bodyPitch = n.rollRate = n.bodyPitchRate = 0.0;
            placeNpc(n, n.s);
        }
        return;
    }

    // ---- turning through a junction along a cubic Bezier curve
    if (n.inJunction)
    {
        const double headingBefore = n.heading;
        // wait while another car is turning through the same junction too close ahead
        double target = 7.0;
        for (size_t j = 0; j < npcs.size(); ++j)
        {
            const Npc& o = npcs[j];
            if (j == i || !o.alive || std::abs(o.z - n.z) > 2.0)
                continue;
            const double dx = o.cx - n.cx, dy = o.cy - n.cy;
            const double ahead = dx * std::cos(n.heading) + dy * std::sin(n.heading);
            if (ahead > 0.0 && ahead < 8.0 && std::abs(-dx * std::sin(n.heading) + dy * std::cos(n.heading)) < 1.6)
                target = 0.0;
        }
        n.speed += std::clamp(target - n.speed, -6.0 * dt, npcAccel(n.speed) * dt);
        n.jt = std::min(1.0, n.jt + n.speed * dt / std::max(1.0, n.jLen));
        const double t = n.jt, u = 1.0 - t;
        const double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
        const double d0 = -3 * u * u, d1 = 3 * u * u - 6 * u * t, d2 = 6 * u * t - 3 * t * t, d3 = 3 * t * t;
        n.cx = b0 * n.jp[0][0] + b1 * n.jp[1][0] + b2 * n.jp[2][0] + b3 * n.jp[3][0];
        n.cy = b0 * n.jp[0][1] + b1 * n.jp[1][1] + b2 * n.jp[2][1] + b3 * n.jp[3][1];
        const double tx = d0 * n.jp[0][0] + d1 * n.jp[1][0] + d2 * n.jp[2][0] + d3 * n.jp[3][0];
        const double ty = d0 * n.jp[0][1] + d1 * n.jp[1][1] + d2 * n.jp[2][1] + d3 * n.jp[3][1];
        if (std::hypot(tx, ty) > 1e-6)
            n.heading = std::atan2(ty, tx);
        n.z = n.jz[0] + (n.jz[1] - n.jz[0]) * t;
        n.pitch = 0.0;
        if (dt > 0.0)
        {
            double dh = n.heading - headingBefore;
            while (dh > kPi) dh -= 2.0 * kPi;
            while (dh < -kPi) dh += 2.0 * kPi;
            n.ax = (n.speed - n.lastSpeed) / dt;
            n.ay = n.speed * dh / dt;
            n.lastSpeed = n.speed;
            npcBody(n, dt);
        }
        if (n.jt >= 1.0)
        {
            n.inJunction = false;
            n.road = n.nextRoad + 1;
            n.dir = n.nextDir;
            n.lane = n.nextLane;
            placeNpc(n, n.nextS);
        }
        return;
    }

    const World::NetRoad& nr = net[n.road - 1];
    const RoadModel& road = nr.road;
    const double L = road.length();

    // gap to the car ahead in the same lane (other cars, the player when on this street)
    double gap = 1e9;
    for (size_t j = 0; j < npcs.size(); ++j)
    {
        const Npc& o = npcs[j];
        if (j == i || !o.alive || o.inJunction || o.road != n.road || o.dir != n.dir || o.lane != n.lane)
            continue;
        const double ahead = n.dir * (o.s - n.s);
        if (ahead > 0.0)
            gap = std::min(gap, ahead - 2.0 * kNpcHalfL);
    }
    if (surface == Surface::Net && netRoad == n.road - 1)
    {
        const double c[3] = { 2.0, 0.5, 0.0 };
        double w[3], es, el;
        vehicle.localToWorld(c, w);
        if (road.project(w[0], w[1], es, el) && std::abs(el - n.dir * road.laneOffset(n.lane)) < 1.6)
        {
            const double ahead = n.dir * (es - n.s);
            if (ahead > 0.0)
                gap = std::min(gap, ahead - kNpcHalfL - 2.1);
        }
    }
    double vmax = nr.limit * n.factor;
    if (n.crashedFor > 0.0)
    {
        n.crashedFor -= dt;
        vmax = 0.0;
    }
    for (double d = 0.0; d <= 50.0; d += 10.0)
    {
        const double k = std::abs(road.curvature(n.s + n.dir * d));
        if (k > 1e-6)
            vmax = std::min(vmax, std::sqrt(2.5 / k + 2.0 * 2.0 * d));
    }
    // slow down for the junction at the end
    const int endNode = n.dir > 0 ? nr.nodeB : nr.nodeA;
    const double exitS = n.dir > 0 ? L - nodes[endNode].radius : nodes[endNode].radius;
    const double remain = n.dir * (exitS - n.s);
    vmax = std::min(vmax, std::sqrt(7.0 * 7.0 + 2.0 * 2.5 * std::max(0.0, remain)));
    const double target = std::clamp(0.9 * (gap - 3.0), 0.0, vmax);
    n.speed += std::clamp(target - n.speed, -6.0 * dt, npcAccel(n.speed) * dt);
    if (gap < 0.5)
        n.speed = 0.0;
    if (dt > 0.0)
    {
        n.ax = (n.speed - n.lastSpeed) / dt;
        n.ay = n.speed * n.speed * road.curvature(n.s) * n.dir;
        n.lastSpeed = n.speed;
        npcBody(n, dt);
    }

    if (remain > 0.0)
    {
        placeNpc(n, n.s + n.dir * n.speed * dt);
        return;
    }

    // ---- reached the junction: pick the next street (never straight back, unless a dead end)
    const World::NetNode& nd = nodes[endNode];
    const int here = n.road - 1, hereEnd = n.dir > 0 ? 1 : 0;
    std::vector<std::pair<int, int>> options;
    for (const auto& e : nd.ends)
        if (!(e.first == here && e.second == hereEnd))
            options.push_back(e);
    if (options.empty())
        options.push_back({ here, hereEnd });   // dead end: turn around
    // deterministic "random" choice
    const unsigned h = static_cast<unsigned>(i * 2654435761u + static_cast<unsigned>(time * 10.0) * 40503u);
    const auto pick = options[h % options.size()];
    const RoadModel& next = net[pick.first].road;
    n.nextRoad = pick.first;
    n.nextDir = pick.second == 0 ? 1 : -1;
    n.nextLane = std::min(n.lane, next.lanes - 1);
    n.nextS = pick.second == 0 ? nd.radius : next.length() - nd.radius;

    double p3x, p3y;
    next.point(n.nextS, n.nextDir * next.laneOffset(n.nextLane), p3x, p3y);
    const double h3 = next.heading(n.nextS, n.nextDir);
    const double chord = std::hypot(p3x - n.cx, p3y - n.cy);
    const double c = std::max(2.0, 0.45 * chord);
    n.jp[0][0] = n.cx;
    n.jp[0][1] = n.cy;
    n.jp[1][0] = n.cx + c * std::cos(n.heading);
    n.jp[1][1] = n.cy + c * std::sin(n.heading);
    n.jp[2][0] = p3x - c * std::cos(h3);
    n.jp[2][1] = p3y - c * std::sin(h3);
    n.jp[3][0] = p3x;
    n.jp[3][1] = p3y;
    // curve length (sampled)
    double len = 0.0, px = n.cx, py = n.cy;
    for (int k = 1; k <= 16; ++k)
    {
        const double t = k / 16.0, u = 1.0 - t;
        const double x = u * u * u * n.jp[0][0] + 3 * u * u * t * n.jp[1][0] + 3 * u * t * t * n.jp[2][0] + t * t * t * n.jp[3][0];
        const double y = u * u * u * n.jp[0][1] + 3 * u * u * t * n.jp[1][1] + 3 * u * t * t * n.jp[2][1] + t * t * t * n.jp[3][1];
        len += std::hypot(x - px, y - py);
        px = x;
        py = y;
    }
    n.jLen = len;
    n.jt = 0.0;
    n.jz[0] = n.z;
    n.jz[1] = next.elevation(n.nextS);
    n.inJunction = true;
    ++junctionTurns;
}

std::vector<LidarSensor::DynamicBox> DriveSim::dynamicBoxes() const
{
    std::vector<LidarSensor::DynamicBox> boxes;
    const auto& spawns = world.npcSpawns();
    for (size_t i = 0; i < npcs.size(); ++i)
    {
        const auto& n = npcs[i];
        if (!n.alive)
            continue;
        boxes.push_back({ n.cx, n.cy, n.heading, kNpcHalfL, kNpcHalfW, world.road().groundZ + n.z, n.z + 1.22,
                          spawns[i].paint, static_cast<int>(i) });
    }
    for (const auto& pr : world.props())
    {
        // standing: its own size; lying on the side: long axis horizontal
        const bool lying = std::abs(std::sin(pr.tilt)) > 0.7;
        const double hz = lying ? pr.halfW : 0.5 * pr.height;
        const double hl = lying ? 0.5 * pr.height : pr.halfL;
        boxes.push_back({ pr.pos[0], pr.pos[1], pr.yaw0 + pr.spin + (lying ? pr.tiltDir - pr.yaw0 : 0.0), hl,
                          pr.halfW, pr.pos[2] - hz, pr.pos[2] + hz, pr.material, -1 });
    }
    return boxes;
}

void DriveSim::step(double dt, const VehicleModel::Command& manual)
{
    scannedThisStep = false;
    m_dt = dt;
    if (crash.happened)
        return;

    animate(dt);
    updateTraffic(dt);
    updateSignalAhead();
    pilot.setSignal(signalAhead.mustStop ? std::max(0.0, signalAhead.distance) : 1e9,
                    signalAhead.mustStop && signalAhead.distance < 80.0);

    // the LiDAR head keeps spinning: fire the columns swept during this step
    scannedThisStep = lidar.update(vehicle, dynamicBoxes(), dt);

    if (autopilotEnabled && !canAutopilot())
        autopilotEnabled = false;   // off the main loop: the driver takes over
    lastCommand = autopilotEnabled
        ? pilot.update(vehicle, lidar.hits(), world.road(), dt)
        : manual;

    const Pose before{ vehicle.x(), vehicle.y(), vehicle.heading() };
    if (autopilotEnabled)
        vehicle.manualShift = false;   // the autopilot drives in D
    vehicle.driverAids = autopilotEnabled || userDriverAids;
    vehicle.step(dt, lastCommand);
    updateSurface();
    updateVertical(dt);
    time += dt;
    distance += std::abs(vehicle.speed()) * dt;

    // any collision at the current pose? (static world, loose objects, traffic)
    const Footprint fp = footprint();
    auto collides = [&](double c[3], ObstacleKind& k, int& npc, int& prop) {
        npc = -1;
        if (world.checkCollision(vehicle, c, &k, &fp, &prop, static_cast<int>(surface), netRoad))
            return true;
        for (size_t i = 0; i < npcs.size(); ++i)
        {
            const Npc& n = npcs[i];
            if (!n.alive || std::abs(n.cx - vehicle.x()) > 8.0 || std::abs(n.cy - vehicle.y()) > 8.0 ||
                std::abs(n.z - vehicle.z()) > 2.0)
                continue;
            if (World::footprintHitsBox(vehicle, n.cx, n.cy, n.heading, kNpcHalfL, kNpcHalfW, c, &fp))
            {
                k = ObstacleKind::Car;
                npc = static_cast<int>(i);
                return true;
            }
        }
        return false;
    };

    double contact[3];
    ObstacleKind kind = ObstacleKind::Wall;
    int npcHit = -1, propHit = -1;
    if (collides(contact, kind, npcHit, propHit))
    {
        // At high speed the car moves ~0.8 m per step and is already inside the obstacle.
        // Binary search on this step for the moment of first contact.
        const Pose after{ vehicle.x(), vehicle.y(), vehicle.heading() };
        auto poseAt = [&](double m) {
            vehicle.setPose(before.x + m * (after.x - before.x), before.y + m * (after.y - before.y),
                            before.heading + m * (after.heading - before.heading));
            updateSurface();
        };
        double lo = 0.0, hi = 1.0;
        for (int it = 0; it < 12; ++it)
        {
            const double m = 0.5 * (lo + hi);
            poseAt(m);
            double c[3];
            ObstacleKind k;
            int n = -1, pr = -1;
            if (collides(c, k, n, pr)) hi = m; else lo = m;
        }
        poseAt(hi);
        collides(contact, kind, npcHit, propHit);
        poseAt(lo);   // last pose without overlap

        resolveImpact(contact, kind, npcHit, propHit);

        // still overlapping (pushed by traffic, pressed against a wall)? move out along the contact normal
        for (int it = 0; it < 10; ++it)
        {
            double c[3];
            ObstacleKind k;
            int n = -1, pr = -1;
            if (!collides(c, k, n, pr))
                break;
            double cw[3];
            const double centerLocal[3] = { 2.0, 0.5, 0.0 };
            vehicle.localToWorld(centerLocal, cw);
            const double dx = cw[0] - c[0], dy = cw[1] - c[1], dl = std::max(1e-6, std::hypot(dx, dy));
            vehicle.moveBy(0.05 * dx / dl, 0.05 * dy / dl);
        }
    }
    updateSurface();
    path.push_back({ vehicle.x(), vehicle.y(), vehicle.heading(), vehicle.z(), vehicle.pitch() });
}

namespace
{
// driver space of the cab: behind the dashboard, above the floor (set per vehicle type)
double g_cabinX = 0.55;
bool cabinRegion(const double p[3]) { return p[0] > g_cabinX && p[2] > 0.35; }
}

double DriveSim::wreckEnergy() const
{
    // the body is a write-off after absorbing as much energy as a 48 km/h frontal hit into a wall
    const double v = 48.0 / 3.6;
    return 0.5 * vehicle.mass * v * v;
}

Footprint DriveSim::footprint() const
{
    // the crushed faces move in: the car can get that much closer to an obstacle
    Footprint fp;
    fp.x0 += 0.9 * deformer.faceCrush(0);
    fp.x1 -= 0.9 * deformer.faceCrush(1);
    fp.y0 += 0.9 * deformer.faceCrush(2);
    fp.y1 -= 0.9 * deformer.faceCrush(3);
    return fp;
}

void DriveSim::animate(double dt)
{
    deformer.update(dt);
    updateWheels(dt);
    if (crash.happened)
    {
        for (auto& n : npcs)
            if (n.alive && n.free)
                npcFreeStep(n, dt);
        if (dt > 0.0 && (std::abs(vehicle.speed()) > 0.2 || vehicle.slideSpeed() > 0.2 || std::abs(vehicle.yawRate()) > 0.1))
        {
            const VehicleModel saved = vehicle;
            VehicleModel::Command c;
            c.brake = 0.7;   // engine dead, the driver on the brake
            vehicle.driverAids = false;
            vehicle.step(dt, c);
            updateSurface();
            double ct[3];
            ObstacleKind k;
            int pr = -1;
            const Footprint fp = footprint();
            if (world.checkCollision(vehicle, ct, &k, &fp, &pr, static_cast<int>(surface), netRoad))
            {
                // grinding against the wall / rail: back to the last free pose, lose speed fast
                vehicle = saved;
                double vx, vy;
                vehicle.velocity(vx, vy);
                vehicle.applyImpulse(-0.35 * vx, -0.35 * vy, -0.35 * vehicle.yawRate());
                if (std::hypot(vx, vy) < 1.0)
                    vehicle.clearMotion();
                updateSurface();
            }
        }
    }
    world.updateProps(dt);
}

void DriveSim::buildReport()
{
    CrashReport& r = report;
    const ImpactInfo& im = lastImpact;
    r.speedKmh = im.speedKmh;
    r.relSpeedKmh = im.relSpeedKmh;
    r.severity = World::obstacleProps(im.kind).carShare;
    // same damage as hitting a rigid wall at this speed: E_car = 1/2 m v^2
    const double veq = std::sqrt(2.0 * im.carKJ * 1000.0 / vehicle.mass);
    r.effectiveKmh = veq * 3.6;
    r.energyKJ = im.energyKJ;
    r.carKJ = im.carKJ;
    r.objectMassKg = im.objectMassKg;
    r.dropHeightM = veq * veq / (2.0 * 9.81);
    r.peakG = im.peakG;
    r.avgG = im.avgG;
    r.pulseMs = im.pulseMs;

    const VehicleShapes::Spec& sp = VehicleShapes::spec(vehicleType());
    g_cabinX = sp.eye[0] - 0.45;
    r.maxCrushM = deformer.maxDisplacement();
    r.intrusionM = deformer.maxDisplacementIn(0, cabinRegion);
    r.damagedPct = 100.0 * deformer.damagedFraction(0.03);
    r.maxStrainPct = deformer.finalStrainPct();
    r.objectKo = World::kindNameKo(im.kind);
    r.damagePct = damagePct;
    r.impacts = impacts;

    const double* n = im.normalLocal;
    r.sideKo = n[0] > 0.5 ? u8"정면" : n[0] < -0.5 ? u8"후면" : n[1] > 0.5 ? u8"왼쪽 측면" : u8"오른쪽 측면";

    // grade from the accumulated damage; the driver space pushed in counts extra
    int g = damagePct < 15.0 ? 1 : damagePct < 40.0 ? 2 : damagePct < kWreckPct ? 3 : 4;
    if (r.intrusionM > kIntrusionWreck) g = 4;
    else if (r.intrusionM > 0.20) g = std::max(g, 3);
    r.grade = g;
    r.wrecked = g == 4;

    static const char* gradeKo[] = { "", u8"경미", u8"보통", u8"심각", u8"전파 (폐차 수준)" };
    static const char* gradeEn[] = { "", "MINOR", "MODERATE", "SEVERE", "TOTAL LOSS" };
    static const char* repairKo[] = {
        "",
        u8"범퍼 긁힘 정도. 도색·간단한 수리로 해결",
        u8"범퍼·보닛·라이트 교체가 필요한 수준",
        u8"차체 프레임까지 휘어서 큰 수리가 필요 (조금만 더 부딪히면 폐차)",
        u8"차체가 더 버티지 못함 — 수리비가 차값을 넘는 수준 (폐차)" };
    r.gradeKo = gradeKo[g];
    r.gradeEn = gradeEn[g];
    r.repairKo = repairKo[g];

    // injury risk from the deceleration of this impact (and the driver space)
    int inj = r.peakG < 20.0 ? 0 : r.peakG < 40.0 ? 1 : r.peakG < 60.0 ? 2 : 3;
    if (r.intrusionM > 0.30) inj = std::max(inj, 3);
    static const char* injKo[] = { u8"낮음 (안전벨트 착용 시 거의 무사)", u8"보통 (타박상·목 부상 가능)",
                                   u8"높음 (골절 등 중상 가능)", u8"매우 높음 (생명 위험)" };
    r.injuryKo = injKo[inj];
}

void DriveSim::resolveImpact(const double contactWorld[3], ObstacleKind kind, int npc, int prop)
{
    const ObstacleProps op = World::obstacleProps(kind);

    // which face was hit? -> inward normal (vehicle local) and the impact center on the body.
    // Driving forward into something near the front counts as a frontal impact even when the
    // first touch is on a front corner; likewise for reversing. Otherwise it is a side impact
    // (doors / cargo box at the point of contact).
    double l[3];
    vehicle.worldToLocal(contactWorld, l);
    double n[3] = { 1.0, 0.0, 0.0 };
    double c[3] = { std::clamp(l[0], 0.0, VehicleModel::kLength), std::clamp(l[1], 0.0, VehicleModel::kWidth), 0.6 };
    const double v = vehicle.speed();
    const Footprint fp = footprint();
    if (v > 0.5 && l[0] < fp.x0 + 0.8)
    {
        n[0] = 1.0; n[1] = 0.0; c[0] = 0.0;
    }
    else if (v < -0.5 && l[0] > fp.x1 - 0.8)
    {
        n[0] = -1.0; n[1] = 0.0; c[0] = VehicleModel::kLength;
    }
    else
    {
        const double dFront = std::abs(l[0] - fp.x0);
        const double dRear  = std::abs(l[0] - fp.x1);
        const double dSide0 = std::abs(l[1] - fp.y0);
        const double dSide1 = std::abs(l[1] - fp.y1);
        const double dMin = std::min({ dFront, dRear, dSide0, dSide1 });
        if (dMin == dFront)      { n[0] = 1.0;  n[1] = 0.0;  c[0] = 0.0; }
        else if (dMin == dRear)  { n[0] = -1.0; n[1] = 0.0;  c[0] = VehicleModel::kLength; }
        else if (dMin == dSide0) { n[0] = 0.0;  n[1] = 1.0;  c[1] = 0.0; }
        else                     { n[0] = 0.0;  n[1] = -1.0; c[1] = VehicleModel::kWidth; }
    }
    double nW[3];
    vehicle.localDirToWorld(n, nW);
    {
        const double nl = std::hypot(nW[0], nW[1]);
        nW[0] /= nl;
        nW[1] /= nl;
        nW[2] = 0.0;
    }
    // a guard rail / median / wall is hit at an angle: only the velocity across the barrier is lost
    // in the impact; along it the car keeps sliding (and scrapes). The crush goes along that normal too.
    {
        double nb[2];
        if (barrierNormal(contactWorld, kind, nb))
        {
            nW[0] = nb[0];
            nW[1] = nb[1];
            const double phi = vehicle.heading() - kPi, cp = std::cos(phi), sp = std::sin(phi);
            n[0] = cp * nb[0] + sp * nb[1];
            n[1] = -sp * nb[0] + cp * nb[1];
            n[2] = 0.0;
        }
    }

    // relative velocity along the normal (< 0: closing)
    double vcx, vcy;
    vehicle.velocity(vcx, vcy);
    double vox = 0.0, voy = 0.0;
    if (npc >= 0)
        npcs[npc].velocity(vox, voy);
    const double vrx = vcx - vox, vry = vcy - voy;
    const double vn = vrx * nW[0] + vry * nW[1];
    const double mc = vehicle.mass;
    const double mo = op.massKg > 0.0 ? op.massKg : 1e9;
    const double mu = mc * mo / (mc + mo);

    if (vn > -0.8)
    {
        // touching / pushing gently: stop the closing motion, no crash
        if (vn < 0.0)
        {
            const double dv = -vn * mo / (mc + mo);
            vehicle.applyImpulse(dv * nW[0], dv * nW[1], 0.0);
        }
        vehicle.moveBy(0.02 * nW[0], 0.02 * nW[1]);
        // sliding along it: friction slows the car down gradually and grinds the body
        const double tx = vrx - vn * nW[0], ty = vry - vn * nW[1], tl = std::hypot(tx, ty);
        if (tl > 1.0 && op.massKg <= 0.0)
            scrape(contactWorld, kind, nW, tx, ty, tl);
        return;
    }

    // ---- impact: momentum exchange along the normal + friction along the contact
    const double e = op.restitution;
    const double J = -(1.0 + e) * mu * vn;                      // normal impulse on the car
    const double tx = vrx - vn * nW[0], ty = vry - vn * nW[1];
    const double tl = std::hypot(tx, ty);
    const double Jt = tl > 1e-6 ? std::min(op.friction * J, mu * tl) : 0.0;
    const double Px = J * nW[0] - (tl > 1e-6 ? Jt * tx / tl : 0.0);
    const double Py = J * nW[1] - (tl > 1e-6 ? Jt * ty / tl : 0.0);
    // an off-center hit spins the car (inertia of a 4.5 x 1.8 m box)
    const double centerLocal[3] = { 2.0, 0.5, 0.0 };
    double cw[3];
    vehicle.localToWorld(centerLocal, cw);
    const double rx = contactWorld[0] - cw[0], ry = contactWorld[1] - cw[1];
    const double inertia = mc * (4.5 * 4.5 + 1.8 * 1.8) / 12.0;
    const double dyaw = std::clamp((rx * Py - ry * Px) / inertia * 2.0, -2.5, 2.5);
    const double speedBefore = std::hypot(vcx, vcy);
    vehicle.applyImpulse(Px / mc, Py / mc, dyaw);

    // the other object gets the opposite impulse
    bool knocked = false;
    if (npc >= 0)
    {
        Npc& o = npcs[npc];
        const double dvx = -Px / mo, dvy = -Py / mo;
        if (std::hypot(dvx, dvy) > 1.5 || o.free)
        {
            // a real hit: the car leaves its lane and slides / spins (momentum + angular momentum about
            // its center), then the tires bring it to rest
            if (!o.free)
            {
                o.free = true;
                o.vx = o.speed;
                o.vy = 0.0;
                o.r = 0.0;
                o.inJunction = false;
            }
            const double ch = std::cos(o.heading), sh = std::sin(o.heading);
            o.vx += dvx * ch + dvy * sh;
            o.vy += -dvx * sh + dvy * ch;
            const double rx = contactWorld[0] - o.cx, ry = contactWorld[1] - o.cy;
            const double Io = mo * (4.5 * 4.5 + 1.8 * 1.8) / 12.0;
            o.r += std::clamp(0.6 * (rx * -Py - ry * -Px) / Io, -2.5, 2.5);   // part of the impulse spreads over the contact
            o.speed = std::hypot(o.vx, o.vy);
            o.crashedFor = 15.0;   // the hit car stops with its hazard lights on
        }
        else
            o.speed = std::max(0.0, o.speed + dvx * std::cos(o.heading) + dvy * std::sin(o.heading));
    }
    if (prop >= 0)
    {
        // a light object leaves faster than the car (bounce), lifts off and is thrown a little sideways
        const double dv = std::hypot(Px, Py) / mo;
        const double side = std::fmod(contactWorld[0] * 7.31 + contactWorld[1] * 3.17, 1.0) - 0.5;
        const double lat = 0.35 * dv * side;
        const double vel[3] = { -Px / mo - lat * nW[1], -Py / mo + lat * nW[0], 1.5 + 0.35 * dv };
        world.knockProp(prop, vel);
        knocked = true;
    }

    // ---- energy: what the collision dissipates, the car's share crumples its body
    const double E = 0.5 * mu * vn * vn * (1.0 - e * e);
    const double Ecar = E * op.carShare;
    const bool wide = kind == ObstacleKind::Wall || kind == ObstacleKind::Building || kind == ObstacleKind::Guardrail ||
                      kind == ObstacleKind::TunnelWall || kind == ObstacleKind::Crate || kind == ObstacleKind::Car ||
                      kind == ObstacleKind::ParkedCar;
    const double added = deformer.addImpact(c, n, Ecar, wide ? 1.4 : 0.3, mc);
    damageWheels(c, n, Ecar);
    const double damageBefore = damagePct;
    damagePct += 100.0 * Ecar / wreckEnergy();
    ++impacts;
    ++impactSerial;

    // crash pulse: the car's velocity change over the crush time (half sine)
    ImpactInfo& im = lastImpact;
    im.kind = kind;
    im.npc = npc;
    im.knocked = knocked;
    im.speedKmh = speedBefore * 3.6;
    im.relSpeedKmh = -vn * 3.6;
    im.objectMassKg = op.massKg;
    im.energyKJ = E / 1000.0;
    im.carKJ = Ecar / 1000.0;
    im.crushAdded = added;
    {
        const double x = std::max(0.03, std::max(added, 0.05 * std::sqrt(E / 1000.0)));
        const double T = 2.0 * x / -vn;
        const double dv = std::hypot(Px, Py) / mc;
        im.avgG = dv / T / 9.81;
        im.peakG = 1.57 * im.avgG;
        im.pulseMs = 1000.0 * T;
    }
    std::copy(contactWorld, contactWorld + 3, im.contactWorld);
    std::copy(n, n + 3, im.normalLocal);
    im.time = time;
    im.damageBefore = damageBefore;
    im.damageAfter = damagePct;
    impactLog.push_back(im);

    // legacy crash info (UI camera, recorder): the last impact
    crash.speed = speedBefore;
    crash.kind = kind;
    crash.npc = npc;
    std::copy(contactWorld, contactWorld + 3, crash.contactWorld);
    std::copy(c, c + 3, crash.impactLocal);
    std::copy(n, n + 3, crash.normalLocal);

    buildReport();
    if (report.wrecked)
    {
        // too much damage: the run ends here
        crash.happened = true;
        vehicle.stopEngine();   // it keeps its momentum and slides / scrapes to rest (animate)
    }
    // separate a little so the next step does not register the same contact again
    vehicle.moveBy(0.02 * nW[0], 0.02 * nW[1]);
}


// =====================================================================
// wheels and axles
// =====================================================================
void DriveSim::setupWheels()
{
    vtkUnstructuredGrid* g = world.grid(World::PidTires);
    const auto& hubs = world.wheelHubs();
    wheels.assign(hubs.size(), WheelState{});
    for (size_t k = 0; k < hubs.size(); ++k)
        std::copy(hubs[k].begin(), hubs[k].end(), wheels[k].hub);
    m_wheelRest.clear();
    m_wheelOf.clear();
    if (!g || hubs.empty())
        return;
    const vtkIdType np = g->GetNumberOfPoints();
    m_wheelRest.resize(3 * np);
    m_wheelOf.resize(np);
    for (vtkIdType i = 0; i < np; ++i)
    {
        double p[3];
        g->GetPoint(i, p);
        std::copy(p, p + 3, &m_wheelRest[3 * i]);
        // nearest hub (the side decides first: tandem axles of the truck touch each other)
        int best = 0;
        double bd = 1e9;
        for (size_t k = 0; k < hubs.size(); ++k)
        {
            if ((p[1] < 0.5) != (hubs[k][1] < 0.5))
                continue;
            const double d = std::hypot(p[0] - hubs[k][0], p[2] - hubs[k][2]);
            if (d < bd)
            {
                bd = d;
                best = static_cast<int>(k);
            }
        }
        m_wheelOf[i] = best;
    }
}

void DriveSim::resetWheels()
{
    for (auto& w : wheels)
    {
        const WheelState fresh{};
        double hub[3];
        std::copy(w.hub, w.hub + 3, hub);
        w = fresh;
        std::copy(hub, hub + 3, w.hub);
    }
    vehicle.wheelGrip[0] = vehicle.wheelGrip[1] = 1.0;
    vehicle.wheelPull = vehicle.wheelDrag = vehicle.sagRoll = vehicle.sagPitch = 0.0;
    m_wheelsMoving = true;
    updateWheels(0.0);
}

void DriveSim::damageWheels(const double c[3], const double n[3], double energyJ)
{
    // energy that bends a suspension arm / axle: ~30 kJ for a 1.5 t car (more for heavier cars);
    // a frontal hit is mostly taken by the crumple zone in front of the wheels
    const double eRef = 30000.0 * std::pow(vehicle.mass / 1500.0, 0.8);
    const bool sideHit = std::abs(n[1]) > std::abs(n[0]);
    for (auto& w : wheels)
    {
        const double dx = w.hub[0] - c[0], dy = w.hub[1] - c[1];
        const double d = std::hypot(dx, dy);
        // the wheel nearest the contact takes most of it; a side hit right on the wheel is the worst
        const double reach = sideHit ? 0.8 : 1.1;
        const double share = std::exp(-(d * d) / (reach * reach)) * (sideHit ? 1.0 : 0.35);
        if (share < 0.03)
            continue;
        const double add = energyJ * share / eRef;
        if (add < 0.01)
            continue;
        w.damage += add;
        const double D = std::min(1.5, w.damage);
        // pushed in along the impact, the wheel tilts (camber: the bottom kicks out) and turns (toe)
        const double push = std::min(0.22, 0.16 * D);
        w.push[0] = push * n[0];
        w.push[1] = push * n[1];
        const double side = w.left() ? -1.0 : 1.0;   // outward direction of this wheel (local y)
        w.camber = std::min(0.45, 0.30 * D) * side * (sideHit ? 1.0 : 0.5);
        w.toe = std::min(0.35, 0.22 * D) * (sideHit ? side : -side) * (w.front() ? 1.0 : 0.6);
        if (w.damage > 1.3)
            w.detached = true;
    }
    m_wheelsMoving = true;

    // what the driver feels: grip, pull to one side, drag, a corner sags on the ground
    double gripF = 1.0, gripR = 1.0, pull = 0.0, drag = 0.0, sagR = 0.0, sagP = 0.0;
    for (const auto& w : wheels)
    {
        const double D = std::min(1.0, w.damage);
        double g = 1.0 - 0.30 * D;
        if (w.detached)
            g = 0.5;
        if (w.front()) gripF = std::min(gripF, g);
        else gripR = std::min(gripR, g);
        // + steer = left; a dragging / toed-in wheel pulls the car to its own side
        const double toward = w.left() ? 1.0 : -1.0;
        pull += toward * (0.035 * D + (w.detached ? 0.05 : 0.0));
        drag += vehicle.mass * 9.81 * (0.025 * D + (w.detached ? 0.22 : 0.0));
        if (w.detached)
        {
            sagR += w.left() ? 0.06 : -0.06;
            sagP += w.front() ? -0.04 : 0.04;
        }
    }
    vehicle.wheelGrip[0] = gripF;
    vehicle.wheelGrip[1] = gripR;
    vehicle.wheelPull = std::clamp(pull, -0.10, 0.10);
    vehicle.wheelDrag = drag;
    vehicle.sagRoll = std::clamp(sagR, -0.09, 0.09);
    vehicle.sagPitch = std::clamp(sagP, -0.06, 0.06);
}

void DriveSim::updateWheels(double dt)
{
    vtkUnstructuredGrid* g = world.grid(World::PidTires);
    if (!m_wheelsMoving || !g || m_wheelRest.empty() || g->GetNumberOfPoints() * 3 != static_cast<vtkIdType>(m_wheelRest.size()))
        return;
    bool moving = false;
    const double k = dt > 0.0 ? std::min(1.0, dt * 10.0) : 1.0;
    for (auto& w : wheels)
    {
        const double tgt[4] = { w.push[0], w.push[1], w.camber, w.toe };
        for (int i = 0; i < 4; ++i)
        {
            w.cur[i] += (tgt[i] - w.cur[i]) * k;
            if (std::abs(tgt[i] - w.cur[i]) > 1e-4)
                moving = true;
        }
        if (w.detached && w.fall < 1.0)
        {
            w.fall = dt > 0.0 ? std::min(1.0, w.fall + dt / 0.6) : 1.0;
            moving = true;
        }
    }
    const double R = world.wheelRadius();
    vtkPoints* pts = g->GetPoints();
    const vtkIdType np = g->GetNumberOfPoints();
    for (vtkIdType i = 0; i < np; ++i)
    {
        const WheelState& w = wheels[m_wheelOf[i]];
        const double* p0 = &m_wheelRest[3 * i];
        double q[3] = { p0[0] - w.hub[0], p0[1] - w.hub[1], p0[2] - w.hub[2] };
        const double out = w.left() ? -1.0 : 1.0;
        // camber: rotation about the local x axis; toe: about z
        double cam = w.cur[2], toe = w.cur[3], off[3] = { w.cur[0], w.cur[1], 0.0 };
        if (w.detached)
        {
            // the wheel breaks off, kicks outward and falls flat, then it is gone (left behind)
            const double f = w.fall;
            cam += out * 1.5 * f;
            off[1] += out * 0.8 * f;
            off[2] -= (w.hub[2] - (-0.1 + 0.1)) * f;
            if (f >= 1.0)
                q[0] = q[1] = q[2] = 0.0;
        }
        const double cc = std::cos(cam), sc = std::sin(cam);
        const double y1 = q[1] * cc - q[2] * sc, z1 = q[1] * sc + q[2] * cc;
        const double ct = std::cos(toe), st = std::sin(toe);
        const double x2 = q[0] * ct - y1 * st, y2 = q[0] * st + y1 * ct;
        pts->SetPoint(i, w.hub[0] + off[0] + x2, w.hub[1] + off[1] + y2, w.hub[2] + off[2] + z1);
    }
    (void)R;
    pts->Modified();
    g->Modified();
    m_wheelsMoving = moving;
}


// =====================================================================
// barriers: normal of the rail at the contact, scraping along it
// =====================================================================
bool DriveSim::barrierNormal(const double contactWorld[3], ObstacleKind kind, double nb[2]) const
{
    if (kind != ObstacleKind::Guardrail && kind != ObstacleKind::Wall && kind != ObstacleKind::TunnelWall)
        return false;
    const RoadModel* rd = &world.road();
    if (surface == Surface::Net && netRoad >= 0)
        rd = &world.net()[netRoad].road;
    double s, lat;
    if (!rd->project(contactWorld[0], contactWorld[1], s, lat))
        return false;
    const double h = rd->heading(s, 1);
    nb[0] = -std::sin(h);
    nb[1] = std::cos(h);
    // toward the car
    const double cl[3] = { 2.0, 0.5, 0.0 };
    double cw[3];
    vehicle.localToWorld(cl, cw);
    if ((cw[0] - contactWorld[0]) * nb[0] + (cw[1] - contactWorld[1]) * nb[1] < 0.0)
    {
        nb[0] = -nb[0];
        nb[1] = -nb[1];
    }
    return true;
}

void DriveSim::scrape(const double contactWorld[3], ObstacleKind kind, const double nW[3], double tx, double ty, double tl)
{
    // steel / concrete against the body panel: kinetic friction ~0.45, pressed on by the car's
    // own sideways push -> about 0.45 g of deceleration along the barrier
    const double a = (kind == ObstacleKind::Guardrail ? 0.40 : 0.50) * 9.81;
    const double dv = std::min(tl, a * m_dt);
    vehicle.applyImpulse(-dv * tx / tl, -dv * ty / tl, 0.0);
    lastScrapeTime = time;
    scrapeSpeed = tl;

    // the friction work grinds the side of the body: dents and scratches, a bit at a time
    m_scrapeJ += vehicle.mass * dv * tl * 0.5;   // about half of it goes into the panels
    m_scrapeClock += m_dt;
    if (m_scrapeClock < 0.2 || m_scrapeJ < 500.0)
        return;
    double l[3];
    vehicle.worldToLocal(contactWorld, l);
    const double phi = vehicle.heading() - kPi, cp = std::cos(phi), sp = std::sin(phi);
    double n[3] = { cp * nW[0] + sp * nW[1], -sp * nW[0] + cp * nW[1], 0.0 };
    double c[3] = { std::clamp(l[0], 0.0, VehicleModel::kLength), std::clamp(l[1], 0.0, VehicleModel::kWidth), 0.45 };
    deformer.addImpact(c, n, m_scrapeJ, 0.6, vehicle.mass);
    damageWheels(c, n, 0.3 * m_scrapeJ);
    damagePct += 100.0 * m_scrapeJ / wreckEnergy();
    m_scrapeJ = 0.0;
    m_scrapeClock = 0.0;
}


// =====================================================================
// vertical motion
// =====================================================================
void DriveSim::updateVertical(double dt)
{
    if (dt <= 0.0)
        return;
    const double g = 9.81;
    if (!m_groundValid)
    {
        m_prevGroundZ = m_groundZ;
        m_groundValid = true;
        m_vz = 0.0;
        return;
    }
    const double prevZ = airborne ? m_airZ : m_prevGroundZ;
    const double dzGround = m_groundZ - m_prevGroundZ;
    const double vzRoad = std::clamp(dzGround / dt, -30.0, 30.0);
    m_prevGroundZ = m_groundZ;

    if (!airborne)
    {
        // the road falls away faster than gravity can pull the car down (crest at speed, an edge):
        // the tires unload and the car takes off with its current vertical speed
        const bool crest = vzRoad < m_vz - g * dt - 0.12 && std::abs(vehicle.speed()) > 6.0;
        const bool edge = dzGround < -0.35;   // drove off a step / ledge
        if (crest || edge)
        {
            airborne = true;
            vehicle.airborne = true;
            airTime = 0.0;
            lastAirHeight = 0.0;
            m_airZ = prevZ + m_vz * dt;
            m_airPitch = vehicle.pitch();
        }
        else
        {
            m_vz = vzRoad;
            return;
        }
    }

    // ballistic flight: gravity, the nose slowly drops (the heavier engine in front)
    airTime += dt;
    m_vz -= g * dt;
    m_airZ += m_vz * dt;
    m_airPitch = std::max(m_airPitch - 0.12 * dt, m_groundPitch - 0.35);
    lastAirHeight = std::max(lastAirHeight, m_airZ - m_groundZ);
    if (m_airZ <= m_groundZ)
    {
        // touch-down: the suspension takes the vertical speed relative to the road
        const double vRel = m_vz - vzRoad;   // < 0: coming down
        airborne = false;
        vehicle.airborne = false;
        lastLandingSpeed = vRel;
        ++landingSerial;
        if (vRel < -1.5)
        {
            // nose / tail lands first: the body pitches and bounces on its springs
            vehicle.kickBody(0.25 * vRel * (m_airPitch < m_groundPitch ? 1.0 : -1.0), 0.05 * vRel);
        }
        m_vz = vzRoad;
        vehicle.setElevation(m_groundZ, m_groundPitch);
        if (vRel < -6.0)
            hardLanding(-vRel);
        return;
    }
    vehicle.setElevation(m_airZ, m_airPitch);
}


void DriveSim::hardLanding(double vDown)
{
    // harder than the suspension travel can absorb (> 6 m/s, about a 2 m drop): the rest of the
    // vertical energy goes into the wheels / axles and the underbody
    const double E = 0.5 * vehicle.mass * (vDown * vDown - 36.0);
    const double eRef = 30000.0 * std::pow(vehicle.mass / 1500.0, 0.8);
    for (auto& w : wheels)
    {
        w.damage += 0.6 * E / std::max<size_t>(1, wheels.size()) / eRef;
        if (w.damage > 1.3)
            w.detached = true;
    }
    const double mid[3] = { 2.0, 0.5, 0.6 }, up[3] = { 0.0, 0.0, 1.0 };
    damageWheels(mid, up, 0.0);   // re-evaluates grip / pull / drag / sag from the wheel states
    m_wheelsMoving = true;

    const double damageBefore = damagePct;
    const double Ecar = 0.4 * E;   // the floor pan and the sills
    damagePct += 100.0 * Ecar / wreckEnergy();
    ++impacts;
    ++impactSerial;
    ImpactInfo& im = lastImpact;
    im = ImpactInfo{};
    im.kind = ObstacleKind::Ground;
    im.npc = -1;
    im.speedKmh = std::abs(vehicle.speed()) * 3.6;
    im.relSpeedKmh = vDown * 3.6;
    im.objectMassKg = 0.0;
    im.energyKJ = E / 1000.0;
    im.carKJ = Ecar / 1000.0;
    im.crushAdded = 0.0;
    im.avgG = vDown / 0.08 / 9.81;   // stopped over ~0.08 s by the bottomed-out suspension
    im.peakG = 1.57 * im.avgG;
    im.pulseMs = 80.0;
    const double cl[3] = { 2.0, 0.5, 0.0 };
    vehicle.localToWorld(cl, im.contactWorld);
    im.normalLocal[0] = 0.0;
    im.normalLocal[1] = 0.0;
    im.normalLocal[2] = 1.0;
    im.time = time;
    im.damageBefore = damageBefore;
    im.damageAfter = damagePct;
    impactLog.push_back(im);
    crash.speed = std::abs(vehicle.speed());
    crash.kind = ObstacleKind::Ground;
    crash.npc = -1;
    std::copy(im.contactWorld, im.contactWorld + 3, crash.contactWorld);
    std::copy(cl, cl + 3, crash.impactLocal);
    std::copy(im.normalLocal, im.normalLocal + 3, crash.normalLocal);
    buildReport();
    if (report.wrecked)
    {
        crash.happened = true;
        vehicle.stopEngine();
    }
}
