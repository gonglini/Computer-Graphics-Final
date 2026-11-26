// Headless test: runs the simulation without any window.
//   1) autopilot drives the whole course -> must not crash, must stop before the end wall
//   2) manual full-throttle run straight ahead -> must crash into the first obstacle
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include <vtkCellData.h>
#include <vtkDataArray.h>

#include "core/DataRecorder.h"
#include "core/DriveSim.h"
#include "core/VehicleShapes.h"
#include "app/Quaternion.h"

namespace
{
// quaternion math used by the camera
int testQuaternion()
{
    int fails = 0;
    auto near = [&](double a, double b, const char* what) {
        if (std::abs(a - b) > 1e-9) { std::printf("  FAIL %s: %.12f vs %.12f\n", what, a, b); ++fails; }
    };
    const double pi = 3.14159265358979323846;

    // 90 deg about z: x -> y
    const double xAxis[3] = { 1, 0, 0 }, yAxis[3] = { 0, 1, 0 };
    double r[3];
    Quat::aboutZ(pi / 2).rotate(xAxis, r);
    near(r[0], 0, "rotZ x"); near(r[1], 1, "rotZ y"); near(r[2], 0, "rotZ z");

    // composition: (a*b) v == a(b v)
    const double axis[3] = { 0.6, 0.0, 0.8 };
    const Quat a = Quat::axisAngle(axis, 0.7), b = Quat::aboutZ(1.3);
    const double v[3] = { 0.3, -1.2, 2.0 };
    double r1[3], t[3], r2[3];
    (a * b).rotate(v, r1);
    b.rotate(v, t);
    a.rotate(t, r2);
    for (int i = 0; i < 3; ++i) near(r1[i], r2[i], "compose");

    // fromTo maps a onto b
    const double u[3] = { 0.48, 0.6, 0.64 };   // unit
    Quat::fromTo(u, yAxis).rotate(u, r);
    near(r[0], 0, "fromTo x"); near(r[1], 1, "fromTo y"); near(r[2], 0, "fromTo z");

    // slerp halfway between identity and 120 deg about z = 60 deg about z
    Quat::slerp(Quat::identity(), Quat::aboutZ(2 * pi / 3), 0.5).rotate(xAxis, r);
    near(r[0], std::cos(pi / 3), "slerp x"); near(r[1], std::sin(pi / 3), "slerp y");

    // conj is the inverse
    (a.conj() * a).rotate(v, r);
    for (int i = 0; i < 3; ++i) near(r[i], v[i], "conj");

    std::printf("quaternion tests: %s\n", fails ? "FAILED" : "ok");
    return fails;
}

// road geometry: the loop closes, and project() inverts point()
int testRoad(const RoadModel& r)
{
    int fails = 0;
    double x0, y0, h0, x1, y1, h1;
    r.pose(0.0, x0, y0, h0);
    r.pose(r.length() - 1e-9, x1, y1, h1);
    const double gap = std::hypot(x1 - x0, y1 - y0);
    if (gap > 1e-6) { std::printf("  FAIL loop does not close: gap %.6f m\n", gap); ++fails; }
    double worst = 0.0;
    for (double s = 0.0; s < r.length(); s += 7.3)
        for (double lat : { -40.0, -7.5, 0.0, 4.5, 25.0 })
        {
            double x, y, ps, pl;
            r.point(s, lat, x, y);
            if (!r.project(x, y, ps, pl)) { ++fails; continue; }
            worst = std::max({ worst, std::abs(r.diff(ps, s)), std::abs(pl - lat) });
        }
    if (worst > 0.05) { std::printf("  FAIL projection error %.3f m\n", worst); ++fails; }
    std::printf("road: loop %.1f m, closure gap %.2e m, max projection error %.4f m -> %s\n", r.length(), gap, worst,
                fails ? "FAILED" : "ok");
    return fails;
}

// the autopilot must drive one full lap (city, corners, obstacle course, highway, suburb) without crashing
int runAutopilot(DriveSim& sim, double cruise)
{
    sim.reset();
    sim.autopilotEnabled = true;
    sim.pilot.cruiseSpeed = cruise;

    const double dt = 0.02;
    const double lap = sim.world.road().length();
    AutoPilot::State last = sim.pilot.state();
    double scanMsSum = 0.0, maxLat = 0.0, maxSpeedKmh = 0.0;
    int scans = 0;
    double blockedFor = 0.0;
    int dbgTick = 0;

    std::printf("\n=== autopilot lap, cruise %.1f m/s ===\n", cruise);
    while (sim.time < 800.0 && sim.distance < lap)
    {
        sim.step(dt, {});
        if (sim.scannedThisStep) { scanMsSum += sim.lidar.lastScanMs(); ++scans; }
        double es, el;
        sim.egoRoadPosition(es, el);
        maxSpeedKmh = std::max(maxSpeedKmh, sim.vehicle.speed() * 3.6);
        if (const char* dbg = std::getenv("LAPDBG"))   // LAPDBG=s0,s1 : trace a stretch of the lap
        {
            double s0 = 0, s1 = 0;
            if (std::sscanf(dbg, "%lf,%lf", &s0, &s1) == 2 && es > s0 && es < s1 && (++dbgTick % 10) == 0)
            {
                std::printf("  dbg t=%6.2f s=%7.1f lat=%5.2f v=%5.2f st=%-12s lane %d->%d goal free=[%5.1f %5.1f %5.1f] ego=%5.1f",
                            sim.time, es, el, sim.vehicle.speed(), sim.pilot.stateName(), sim.pilot.currentLane(),
                            sim.pilot.targetLane(), std::min(99.9, sim.pilot.laneFreeDistance(0)),
                            std::min(99.9, sim.pilot.laneFreeDistance(1)), std::min(99.9, sim.pilot.laneFreeDistance(2)),
                            std::min(99.9, sim.pilot.egoFreeDistance()));
                for (const auto& n : sim.npcs)
                    if (n.alive && n.dir > 0 && std::abs(sim.world.road().diff(n.s, es)) < 40.0)
                        std::printf("  npc(ds=%5.1f L%d v=%4.1f)", sim.world.road().diff(n.s, es), n.lane, n.speed);
                std::printf("\n");
            }
        }
        if (el < 0.0 || el > sim.world.road().halfWidth) maxLat = std::max(maxLat, std::abs(el));

        if (sim.pilot.state() != last)
        {
            last = sim.pilot.state();
            if (last != AutoPilot::State::Curve && last != AutoPilot::State::Cruise)
                std::printf("t=%6.2f s=%7.1f lat=%5.2f v=%5.2f lane=%d->%d free=[%5.1f %5.1f %5.1f] %-12s %s\n",
                            sim.time, es, el, sim.vehicle.speed(), sim.pilot.currentLane(), sim.pilot.targetLane(),
                            std::min(99.9, sim.pilot.laneFreeDistance(0)),
                            std::min(99.9, sim.pilot.laneFreeDistance(1)),
                            std::min(99.9, sim.pilot.laneFreeDistance(2)), sim.pilot.stateName(),
                            sim.world.sectionNameKo(es));
        }
        if (sim.impacts > 0)
        {
            std::printf("IMPACT at t=%.2f s=%.1f lat=%.2f speed=%.2f into %s (damage %.0f%%)\n", sim.time, es, el,
                        sim.crash.speed, World::kindNameKo(sim.crash.kind), sim.damagePct);
            return 1;
        }
        if (sim.pilot.state() == AutoPilot::State::Blocked)
        {
            blockedFor += dt;
            if (blockedFor > 8.0)
                break;
        }
        else
        {
            blockedFor = 0.0;
        }
    }
    std::printf("END t=%.1f dist=%.1f / %.1f m, top %.0f km/h, avg LiDAR scan %.2f ms, outside own lanes %.2f m, "
                "red-light violations %d\n",
                sim.time, sim.distance, lap, maxSpeedKmh, scans ? scanMsSum / scans : 0.0, maxLat,
                sim.redLightViolations);
    if (sim.redLightViolations > 0)
        return 3;
    return sim.distance >= lap ? 0 : 2;
}

int runManualCrash(DriveSim& sim)
{
    sim.reset();
    sim.autopilotEnabled = false;
    VehicleModel::Command cmd;
    cmd.throttle = 1.0;

    // full throttle straight ahead: hits traffic / whatever is in the way, keeps going while the
    // damage is below 100 %, until the car is wrecked
    std::printf("\n=== manual full throttle ===\n");
    unsigned seen = sim.impactSerial;
    while (sim.time < 90.0 && !sim.crash.happened)
    {
        sim.step(0.02, cmd);
        if (sim.impactSerial != seen)
        {
            seen = sim.impactSerial;
            const auto& im = sim.lastImpact;
            std::printf("  impact %d: %-12s at %5.1f km/h (closing %5.1f), object %5.0f kg, car absorbs %6.1f kJ, "
                        "+%.2f m crush, %4.0f g -> damage %5.1f%%\n",
                        sim.impacts, World::kindNameKo(im.kind), im.speedKmh, im.relSpeedKmh, im.objectMassKg,
                        im.carKJ, im.crushAdded, im.peakG, sim.damagePct);
        }
    }
    if (!sim.crash.happened)
    {
        std::printf("not wrecked after %.0f s?! (damage %.0f%%, %d impacts)\n", sim.time, sim.damagePct, sim.impacts);
        return 1;
    }
    sim.deformer.settle();
    const auto& r = sim.report;
    std::printf("CRASH into %s at x=%.2f speed=%.0f km/h, normal=(%.0f %.0f), car share %.2f -> wall-equivalent %.0f km/h\n"
                "  WRECKED after %d impacts, damage %.0f%%, grade %d (%s), max crush %.2f m, cabin intrusion %.2f m, "
                "damaged %.0f%%, max strain %.0f%%, peak %.0f g\n",
                r.objectKo, sim.vehicle.x(), r.speedKmh, sim.crash.normalLocal[0], sim.crash.normalLocal[1],
                r.severity, r.effectiveKmh, r.impacts, r.damagePct, r.grade, r.gradeEn, r.maxCrushM, r.intrusionM,
                r.damagedPct, r.maxStrainPct, r.peakG);
    if (sim.crash.npc >= 0)
    {
        // the car that was hit leaves its lane: slides and spins to rest
        auto& o = sim.npcs[sim.crash.npc];
        const double x0 = o.cx, y0 = o.cy, h0 = o.heading;
        for (int k = 0; k < 600; ++k)
            sim.animate(0.01);
        std::printf("  hit car: free %d, moved %.1f m, turned %.0f deg, speed now %.1f m/s\n", o.free ? 1 : 0,
                    std::hypot(o.cx - x0, o.cy - y0), (o.heading - h0) * 57.3, o.speed);
    }
    for (const auto& w : sim.wheels)
        std::printf("  wheel %s%s x=%.2f damage %.2f camber %.1f toe %.1f deg%s\n", w.front() ? "F" : "R", w.left() ? "L" : "R",
                    w.hub[0], w.damage, w.camber * 57.3, w.toe * 57.3, w.detached ? "  DETACHED" : "");
    std::printf("  wheel effects: grip F %.2f R %.2f, pull %.1f deg, drag %.1f kN\n", sim.vehicle.wheelGrip[0],
                sim.vehicle.wheelGrip[1], sim.vehicle.wheelPull * 57.3, sim.vehicle.wheelDrag / 1000.0);
    return r.impacts > 0 && r.maxCrushM > 0.05 ? 0 : 1;
}

// drive into a cone and a barrel of the obstacle course: the car must keep going, the object must fly away
int testKnockProps(DriveSim& sim)
{
    const RoadModel& r = sim.world.road();
    int fails = 0;
    bool done[2] = { false, false };
    for (size_t i = 0; i < sim.world.props().size(); ++i)
    {
        const ObstacleKind kind = sim.world.props()[i].kind;
        const int which = kind == ObstacleKind::Cone ? 0 : kind == ObstacleKind::Barrel ? 1 : -1;
        if (which < 0 || done[which])
            continue;
        done[which] = true;
        sim.reset();
        const auto& pr = sim.world.props()[i];
        double ps, pl;
        r.project(pr.x0, pr.y0, ps, pl);
        // the vehicle pose is the rear axle (1 m behind the center), center line on the object
        double x, y;
        r.point(ps - 25.0, pl - 0.0, x, y);
        sim.vehicle.reset(x, y, r.heading(ps));
        {
            // shift so the car's center line (local y = 0.5) runs through the object
            double c[3], l[3] = { 2.0, 0.5, 0.0 };
            sim.vehicle.localToWorld(l, c);
            double cs, cl;
            r.project(c[0], c[1], cs, cl);
            double tx, ty;
            r.point(cs, pl, tx, ty);
            sim.vehicle.moveBy(tx - c[0], ty - c[1]);
        }
        sim.vehicle.setSpeed(12.0);
        VehicleModel::Command cmd;
        cmd.throttle = 0.4;
        while (sim.time < 6.0 && !sim.world.props()[i].knocked && !sim.crash.happened)
            sim.step(0.01, cmd);
        for (int k = 0; k < 60; ++k)   // 0.6 s after the hit (the next gate is further on)
            sim.step(0.01, cmd);
        const auto& p2 = sim.world.props()[i];
        const double moved = std::hypot(p2.pos[0] - p2.x0, p2.pos[1] - p2.y0);
        std::printf("KNOCK %-6s: knocked %d, moved %.1f m, lying %d, car %.0f km/h after, damage %.1f%%\n",
                    which == 0 ? "cone" : "barrel", p2.knocked ? 1 : 0, moved, std::abs(std::sin(p2.tilt)) > 0.7 ? 1 : 0,
                    sim.vehicle.speed() * 3.6, sim.damagePct);
        if (!p2.knocked || moved < 1.0 || sim.crash.happened || sim.vehicle.speed() < 3.0)
        {
            std::printf("  FAIL knock test\n");
            ++fails;
        }
    }
    return fails;
}
}

// LiDAR cost per simulated second at a few places along the course
void benchLidar(DriveSim& sim)
{
    const char* names[] = { "city", "forest", "highway", "suburb" };
    const RoadModel& r = sim.world.road();
    for (int i = 0; i < 4; ++i)
    {
        const double s = sim.world.placeS(i) + 40.0;
        double x, y;
        r.point(s, r.laneOffset(1), x, y);
        sim.vehicle.reset(x, y, r.heading(s));
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < 100; ++k)   // 100 x 10 ms = 1 simulated second = 10 revolutions
            sim.lidar.update(sim.vehicle, {}, 0.01);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("bench %-8s %.0f ms per simulated second (%zu pts/rev)\n", names[i], ms,
                    sim.lidar.totalPoints());
    }
}

// Headless dataset generation: the autopilot drives, every LiDAR revolution is saved.
//   autopilot_sim --record <dir> [seconds=60] [start place 0 city / 1 forest / 2 highway / 3 suburb] [cruise=12]
int recordDataset(DriveSim& sim, int argc, char** argv)
{
    // argv is in the ANSI code page on Windows: go through std::filesystem::path to get UTF-8
    const std::string dir = std::filesystem::path(argv[2]).u8string();
    const double seconds = argc > 3 ? std::atof(argv[3]) : 60.0;
    sim.startS = sim.world.placeS(argc > 4 ? std::atoi(argv[4]) : 0);
    sim.reset();
    sim.autopilotEnabled = true;
    sim.pilot.cruiseSpeed = argc > 5 ? std::atof(argv[5]) : 12.0;

    DataRecorder rec;
    DataRecorder::Options opt;
    opt.maxQueue = 1000000;   // headless: never drop, wait for the disk instead
    if (!rec.start(dir, sim, opt, nullptr))
    {
        std::printf("cannot record: %s\n", rec.lastError().c_str());
        return 1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    while (sim.time < seconds && !sim.crash.happened)
    {
        sim.step(0.01, {});
        rec.poll(sim);
    }
    if (sim.crash.happened)
        rec.logCrash(sim);
    const int frames = rec.frames();
    rec.stop();
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("recorded %d frames (%.1f s simulated, %.1f s wall, %.0f MB) into %s%s\n", frames, sim.time, wall,
                rec.megabytesWritten(), dir.c_str(), rec.lastError().empty() ? "" : " WITH ERRORS");
    return rec.lastError().empty() ? 0 : 1;
}

// Drive single streets of the network end to end with a simple lane follower (pure pursuit on the
// lane): the viaduct over the highway, the underpass, the mountain road, the dead end. Must stay on
// the network, follow the height and not hit anything. Then the traffic: every network car must keep
// moving through the junctions.
int testNet(DriveSim& sim, double cruise)
{
    int fails = 0;
    const auto& net = sim.world.net();
    std::printf("\n=== street network: %zu streets, %zu junctions (viaduct on street %d at s=%.0f, underpass on %d at s=%.0f) ===\n",
                net.size(), sim.world.nodes().size(), sim.world.viaductRoad(), sim.world.viaductS(),
                sim.world.underpassRoad(), sim.world.underpassS());
    for (size_t r = 0; r < net.size(); ++r)
    {
        const RoadModel& road = net[r].road;
        const double L = road.length();
        const double s0 = sim.world.nodes()[net[r].nodeA].radius + 4.0;
        const double s1 = L - sim.world.nodes()[net[r].nodeB].radius - 2.0;
        sim.startNetRoad = static_cast<int>(r);
        sim.startNetS = s0;
        sim.reset();
        sim.autopilotEnabled = false;
        const double lane = road.laneOffset(std::min(sim.startLane, road.lanes - 1));
        double zMin = 1e9, zMax = -1e9, maxPitch = 0.0, maxLat = 0.0, s = s0;
        int off = 0;
        int seen[4] = { 0, 0, 0, 0 };
        const unsigned landings0 = sim.landingSerial;
        double hitSum[4] = { 0, 0, 0, 0 };
        int hitN[4] = { 0, 0, 0, 0 };
        while (s < s1 && sim.time < 200.0)
        {
            const double c[3] = { 2.0, 0.5, 0.0 };
            double w[3], lat;
            sim.vehicle.localToWorld(c, w);
            road.project(w[0], w[1], s, lat);
            double tx, ty;
            road.point(std::min(L, s + 9.0), lane, tx, ty);
            const double dx = tx - sim.vehicle.x(), dy = ty - sim.vehicle.y();
            const double alpha = std::atan2(dy, dx) - sim.vehicle.heading();
            VehicleModel::Command cmd;
            cmd.steer = std::atan2(2.0 * sim.vehicle.wheelBase * std::sin(alpha), std::hypot(dx, dy));
            const double err = (std::getenv("NETFAST") ? cruise : std::min(cruise, net[r].limit)) - sim.vehicle.speed();
            cmd.throttle = std::clamp(0.5 * err, 0.0, 1.0);
            cmd.brake = std::clamp(-0.4 * err, 0.0, 1.0);
            sim.step(0.01, cmd);
            zMin = std::min(zMin, sim.vehicle.z());
            zMax = std::max(zMax, sim.vehicle.z());
            maxPitch = std::max(maxPitch, std::abs(sim.vehicle.pitch()));
            maxLat = std::max(maxLat, std::abs(lat - lane));
            if (sim.surface != DriveSim::Surface::Net)
                ++off;
            const int stNow = sim.world.netStructure(static_cast<int>(r), s);
            seen[stNow]++;
            if (sim.scannedThisStep)
            {
                // only real returns: farther than 4 m from the car (phantom hits collapse onto the sensor)
                int far = 0;
                for (const auto& h : sim.lidar.hits())
                    if (std::hypot(h.p[0] - sim.vehicle.x(), h.p[1] - sim.vehicle.y()) > 4.0)
                        ++far;
                hitSum[stNow] += far;
                ++hitN[stNow];
            }
            if (sim.impacts > 0)
                break;
        }
        if (sim.landingSerial != landings0)
            std::printf("   flights: %u (last %.2f s, %.2f m high, touch-down %.1f m/s)\n", sim.landingSerial - landings0,
                        sim.airTime, sim.lastAirHeight, sim.lastLandingSpeed);
        if (hitN[2] + hitN[3] > 0)
            std::printf("   LiDAR points per scan: graded %.0f, trench %.0f, covered %.0f\n",
                        hitN[0] ? hitSum[0] / hitN[0] : 0.0, hitN[2] ? hitSum[2] / hitN[2] : 0.0,
                        hitN[3] ? hitSum[3] / hitN[3] : 0.0);
        const bool ok = s >= s1 - 1.0 && sim.impacts == 0 && off == 0 && maxLat < 1.5;
        std::printf("STREET %2zu %-16s %5.0f m: reached %5.0f, z %6.2f .. %6.2f m, max grade %4.1f %%, lane error %.2f, "
                    "off %d, impacts %d, viaduct/trench/covered %d/%d/%d %s\n",
                    r, net[r].nameKo, L, s, zMin, zMax, 100.0 * std::tan(maxPitch), maxLat, off, sim.impacts, seen[1],
                    seen[2], seen[3], ok ? "" : "FAIL");
        if (sim.impacts > 0)
            std::printf("   hit %s at (%.1f, %.1f) z %.2f, npc %d\n", World::kindNameKo(sim.lastImpact.kind),
                        sim.lastImpact.contactWorld[0], sim.lastImpact.contactWorld[1], sim.vehicle.z(), sim.lastImpact.npc);
        if (!ok)
            ++fails;
    }
    sim.startNetRoad = -1;

    // traffic on the network: 120 s, every car must move, turn at junctions, never jump
    sim.reset();
    sim.autopilotEnabled = false;
    std::vector<double> travelled(sim.npcs.size(), 0.0);
    std::vector<std::array<double, 2>> last(sim.npcs.size());
    for (size_t i = 0; i < sim.npcs.size(); ++i)
        last[i] = { sim.npcs[i].cx, sim.npcs[i].cy };
    double maxJump = 0.0;
    const int turns0 = sim.junctionTurns;
    for (int k = 0; k < 12000; ++k)
    {
        VehicleModel::Command cmd;
        cmd.brake = 1.0;
        sim.step(0.01, cmd);
        for (size_t i = 0; i < sim.npcs.size(); ++i)
        {
            if (sim.npcs[i].road == 0)
                continue;
            const double d = std::hypot(sim.npcs[i].cx - last[i][0], sim.npcs[i].cy - last[i][1]);
            maxJump = std::max(maxJump, d);
            travelled[i] += d;
            last[i] = { sim.npcs[i].cx, sim.npcs[i].cy };
        }
    }
    int stuck = 0, netCars = 0;
    for (size_t i = 0; i < sim.npcs.size(); ++i)
        if (sim.npcs[i].road != 0)
        {
            ++netCars;
            if (travelled[i] < 300.0)
                ++stuck;
        }
    const bool trafficOk = stuck == 0 && maxJump < 0.5 && sim.junctionTurns - turns0 > 2 * netCars;
    std::printf("NETWORK TRAFFIC %d cars, 120 s: junction turns %d, cars that drove < 300 m: %d, largest step %.2f m %s\n",
                netCars, sim.junctionTurns - turns0, stuck, maxJump, trafficOk ? "" : "FAIL");
    if (!trafficOk)
        ++fails;
    return fails;
}

// full-throttle acceleration of the bare vehicle model (no world): 0->100 km/h, shift points, top speed
int testAcceleration(bool verbose)
{
    int fails = 0;
    for (int type = 0; type < VehicleShapes::TypeCount; ++type)
    {
        const auto& sp = VehicleShapes::spec(type);
        VehicleModel v;
        v.mass = sp.mass; v.enginePower = sp.power; v.maxAccel = sp.maxAccel; v.dragArea = sp.dragArea;
        v.maxSpeed = sp.maxSpeed; v.maxBrake = sp.maxBrake; v.wheelBase = sp.wheelBase;
        v.idleRpm = sp.idleRpm; v.peakPowerRpm = sp.peakPowerRpm; v.redlineRpm = sp.redlineRpm;
        v.wheelRadius = sp.wheelRadius;
        v.configureDrivetrain();
        v.reset(0, 0, 0);
        VehicleModel::Command cmd;
        cmd.throttle = 1.0;
        const double dt = 0.01;
        double t100 = -1.0, top = 0.0;
        int gear = 1, maxGear = 1;
        std::string shifts;
        for (double t = 0.0; t < 90.0; t += dt)
        {
            v.step(dt, cmd);
            if (t100 < 0.0 && v.speed() * 3.6 >= 100.0) t100 = t;
            top = std::max(top, v.speed());
            if (v.gear() != gear)
            {
                char buf[64];
                std::snprintf(buf, sizeof buf, " %d->%d@%.0fkm/h/%.0frpm", gear, v.gear(), v.speed() * 3.6, v.rpm());
                shifts += buf;
                gear = v.gear();
                maxGear = std::max(maxGear, gear);
            }
        }
        std::printf("ACCEL %-6s final %.2f  0-100 %.1f s  top %.0f km/h  peak torque %.0f Nm  gears used %d\n",
                    sp.name, v.finalDrive, t100, top * 3.6, v.maxTorque(), maxGear);
        if (verbose) std::printf("   shifts:%s\n", shifts.c_str());
        if (t100 < 0.0 || maxGear < 7) { std::printf("  FAIL acceleration %s\n", sp.name); ++fails; }

        // coast down from top speed: must downshift back to 1st when stopped
        cmd.throttle = 0.0; cmd.brake = 0.6;
        for (int i = 0; i < 3000; ++i) v.step(dt, cmd);
        if (v.gear() != 1 || std::abs(v.speed()) > 0.01) { std::printf("  FAIL stop gear %d\n", v.gear()); ++fails; }
    }
    return fails;
}

// =====================================================================
// SLAM dataset export (for FAST-LIO): a mapping drive over the whole map without traffic.
//   scans.bin  per revolution: double t0, uint32 n, n x { float x, y, z, intensity, time; uint16 ring }
//              points in the SENSOR frame (x forward, y left, z up), time = seconds after t0
//   imu.csv    200 Hz: t, gyro (rad/s), specific force (m/s^2, gravity included), sensor frame
//   gt.csv     ground truth sensor pose: t, x, y, z, qx, qy, qz, qw (world)
// =====================================================================
namespace
{
struct RoutePt { double x, y, v; };

void appendBezier(std::vector<RoutePt>& r, double x0, double y0, double h0, double x3, double y3, double h3, double v)
{
    const double chord = std::hypot(x3 - x0, y3 - y0);
    const double c = std::max(3.0, 0.45 * chord);
    const double p[4][2] = { { x0, y0 }, { x0 + c * std::cos(h0), y0 + c * std::sin(h0) },
                             { x3 - c * std::cos(h3), y3 - c * std::sin(h3) }, { x3, y3 } };
    const int n = std::max(4, static_cast<int>(chord * 1.3));
    for (int i = 1; i < n; ++i)
    {
        const double t = static_cast<double>(i) / n, u = 1.0 - t;
        r.push_back({ u * u * u * p[0][0] + 3 * u * u * t * p[1][0] + 3 * u * t * t * p[2][0] + t * t * t * p[3][0],
                      u * u * u * p[0][1] + 3 * u * u * t * p[1][1] + 3 * u * t * t * p[2][1] + t * t * t * p[3][1], v });
    }
}

std::vector<RoutePt> mappingRoute(const World& w, double vMain, double vNet)
{
    std::vector<RoutePt> r;
    const RoadModel& m = w.road();
    const double L = m.length();
    // 1) one lap of the main loop in the right lane, then on to the first cross street (s = 55)
    for (double s = 17.0; s <= L + 47.0; s += 1.0)
    {
        double x, y;
        m.point(s, m.laneOffset(2), x, y);
        r.push_back({ x, y, vMain });
    }
    // 2) right turn into the connector street (northbound lanes east of the center line: s = 51.5)
    {
        double x0, y0, x3, y3;
        // start the turn after the traffic light pole on the corner (s = 44.3, lat 10)
        m.point(L + 47.0, m.laneOffset(2), x0, y0);
        m.point(53.5, 20.0, x3, y3);   // inner northbound lane (parked cars on the curb lane)
        appendBezier(r, x0, y0, m.heading(47.0), x3, y3, m.heading(53.5) - 0.5 * RoadModel::kPi, 5.0);
        for (double lat = 20.0; lat <= 136.0; lat += 1.0)
        {
            double x, y;
            m.point(53.5, lat, x, y);
            r.push_back({ x, y, vNet });
        }
    }
    // 3) every street of the network: (street, direction) in driving order from junction 0
    const int order[][2] = { { 0, 1 }, { 7, 1 }, { 9, 1 }, { 8, -1 }, { 6, 1 }, { 4, -1 },
                             { 3, -1 }, { 2, -1 }, { 1, -1 }, { 5, 1 } };
    const auto& net = w.net();
    const auto& nodes = w.nodes();
    for (const auto& o : order)
    {
        const auto& nr = net[o[0]];
        const RoadModel& rd = nr.road;
        const int dir = o[1];
        const double Ls = rd.length();
        const double sA = nodes[nr.nodeA].radius + 1.0, sB = Ls - nodes[nr.nodeB].radius - 1.0;
        const double s0 = dir > 0 ? sA : sB, s1 = dir > 0 ? sB : sA;
        const double lat = dir * rd.laneOffset(0);
        double x0 = r.back().x, y0 = r.back().y;
        const double h0 = std::atan2(r.back().y - r[r.size() - 2].y, r.back().x - r[r.size() - 2].x);
        double x3, y3;
        rd.point(s0, lat, x3, y3);
        appendBezier(r, x0, y0, h0, x3, y3, rd.heading(s0, dir), 6.0);
        const int n = static_cast<int>(std::abs(s1 - s0));
        for (int i = 0; i <= n; ++i)
        {
            const double s = s0 + (s1 - s0) * i / std::max(1, n);
            double x, y;
            rd.point(s, lat, x, y);
            r.push_back({ x, y, std::min(vNet, nr.limit) });
        }
    }
    return r;
}

void rotToQuat(const double X[3], const double Y[3], const double Z[3], double q[4])
{
    // columns X, Y, Z = sensor axes in world
    const double m00 = X[0], m11 = Y[1], m22 = Z[2];
    const double tr = m00 + m11 + m22;
    if (tr > 0)
    {
        const double S = std::sqrt(tr + 1.0) * 2;
        q[3] = 0.25 * S;
        q[0] = (Y[2] - Z[1]) / S;
        q[1] = (Z[0] - X[2]) / S;
        q[2] = (X[1] - Y[0]) / S;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double S = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        q[3] = (Y[2] - Z[1]) / S;
        q[0] = 0.25 * S;
        q[1] = (Y[0] + X[1]) / S;
        q[2] = (Z[0] + X[2]) / S;
    }
    else if (m11 > m22)
    {
        const double S = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        q[3] = (Z[0] - X[2]) / S;
        q[0] = (Y[0] + X[1]) / S;
        q[1] = 0.25 * S;
        q[2] = (Z[1] + Y[2]) / S;
    }
    else
    {
        const double S = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        q[3] = (X[1] - Y[0]) / S;
        q[0] = (Z[0] + X[2]) / S;
        q[1] = (Z[1] + Y[2]) / S;
        q[2] = 0.25 * S;
    }
}
}

int exportSlam(DriveSim& sim, const std::string& dir, double vMain)
{
    std::filesystem::create_directories(std::filesystem::u8path(dir));
    sim.trafficEnabled = false;
    sim.autopilotEnabled = false;
    sim.lidar.config.azimuthStepDeg = 0.8;   // 450 columns x 32 lines, 10 Hz (keeps the bag ~1 GB)
    sim.startNetRoad = -1;
    sim.reset();
    sim.world.hideProps();   // the obstacle course objects would block the mapping route
    const auto route = mappingRoute(sim.world, vMain, 12.0);
    std::printf("mapping route: %zu points (~%.1f km)\n", route.size(), route.size() / 1000.0);

    FILE* fs = std::fopen((dir + "/scans.bin").c_str(), "wb");
    FILE* fi = std::fopen((dir + "/imu.csv").c_str(), "w");
    FILE* fg = std::fopen((dir + "/gt.csv").c_str(), "w");
    if (!fs || !fi || !fg)
    {
        std::printf("cannot write into %s\n", dir.c_str());
        return 1;
    }
    std::fprintf(fi, "t,wx,wy,wz,ax,ay,az\n");
    std::fprintf(fg, "t,x,y,z,qx,qy,qz,qw\n");

    const double dt = 0.005;
    const double* mnt = sim.lidar.config.mount;
    struct Sample { double t, p[3], X[3], Y[3], Z[3]; };
    std::vector<Sample> samples;
    size_t idx = 0;
    unsigned lastRev = sim.lidar.revolution();
    long long nScans = 0, nPts = 0;
    std::vector<float> buf;
    while (idx + 10 < route.size() && sim.time < 900.0)
    {
        // FAST-LIO estimates gravity and the IMU biases from the first samples: stand still for 3 s
        if (sim.time < 3.0)
        {
            VehicleModel::Command hold;
            hold.brake = 1.0;
            sim.step(dt, hold);
            Sample sm;
            sm.t = sim.time;
            sim.vehicle.localToWorld(mnt, sm.p);
            const double ex[3] = { -1, 0, 0 }, ey[3] = { 0, -1, 0 }, ez[3] = { 0, 0, 1 };
            sim.vehicle.localDirToWorld(ex, sm.X);
            sim.vehicle.localDirToWorld(ey, sm.Y);
            sim.vehicle.localDirToWorld(ez, sm.Z);
            samples.push_back(sm);
            lastRev = sim.lidar.revolution();
            continue;
        }
        // pure pursuit on the route polyline
        const double c[3] = { 2.0, 0.5, 0.0 };
        double w[3];
        sim.vehicle.localToWorld(c, w);
        double best = 1e18;
        for (size_t k = idx; k < std::min(route.size(), idx + 40); ++k)
        {
            const double d = std::hypot(route[k].x - w[0], route[k].y - w[1]);
            if (d < best)
            {
                best = d;
                idx = k;
            }
        }
        const size_t look = std::min(route.size() - 1, idx + 9);
        const double dx = route[look].x - sim.vehicle.x(), dy = route[look].y - sim.vehicle.y();
        const double alpha = std::atan2(dy, dx) - sim.vehicle.heading();
        VehicleModel::Command cmd;
        cmd.steer = std::atan2(2.0 * sim.vehicle.wheelBase * std::sin(alpha), std::hypot(dx, dy));
        double vT = route[idx].v;
        for (size_t k = idx; k < std::min(route.size(), idx + 25); ++k)
            vT = std::min(vT, route[k].v + 0.4 * (k - idx));
        const double err = vT - sim.vehicle.speed();
        cmd.throttle = std::clamp(0.5 * err, 0.0, 1.0);
        cmd.brake = std::clamp(-0.4 * err, 0.0, 1.0);
        sim.step(dt, cmd);
        if (sim.impacts > 0)
        {
            std::printf("IMPACT during the mapping drive at t=%.1f (%s)\n", sim.time, World::kindNameKo(sim.lastImpact.kind));
            break;
        }

        // sensor pose (ground truth) for the IMU
        Sample sm;
        sm.t = sim.time;
        sim.vehicle.localToWorld(mnt, sm.p);
        const double ex[3] = { -1, 0, 0 }, ey[3] = { 0, -1, 0 }, ez[3] = { 0, 0, 1 };
        sim.vehicle.localDirToWorld(ex, sm.X);
        sim.vehicle.localDirToWorld(ey, sm.Y);
        sim.vehicle.localDirToWorld(ez, sm.Z);
        samples.push_back(sm);

        // a revolution is complete: the column buffer holds 0.1 s of returns, column c fired at c / n
        if (sim.lidar.revolution() != lastRev)
        {
            lastRev = sim.lidar.revolution();
            const auto& cols = sim.lidar.columnBuffer();
            const double period = 1.0 / sim.lidar.config.rotationHz;
            const double t0 = sim.time - period;
            buf.clear();
            std::vector<unsigned short> rings;
            for (size_t col = 0; col < cols.size(); ++col)
                for (const auto& h : cols[col])
                {
                    if (h.instance >= 0)
                        continue;
                    buf.push_back(static_cast<float>(-(h.local[0] - mnt[0])));
                    buf.push_back(static_cast<float>(-(h.local[1] - mnt[1])));
                    buf.push_back(static_cast<float>(h.local[2] - mnt[2]));
                    buf.push_back(255.0f * h.intensity);
                    buf.push_back(static_cast<float>(period * col / cols.size()));
                    rings.push_back(h.ring);
                }
            const unsigned n = static_cast<unsigned>(rings.size());
            std::fwrite(&t0, sizeof t0, 1, fs);
            std::fwrite(&n, sizeof n, 1, fs);
            for (unsigned k = 0; k < n; ++k)
            {
                std::fwrite(&buf[5 * k], sizeof(float), 5, fs);
                std::fwrite(&rings[k], sizeof(unsigned short), 1, fs);
            }
            ++nScans;
            nPts += n;
        }
    }
    std::fclose(fs);

    // IMU from the ground-truth trajectory: central differences, smoothed, plus sensor noise
    const size_t N = samples.size();
    unsigned seed = 1234567u;
    auto noise = [&]() {
        seed = seed * 1664525u + 1013904223u;
        const double a = (seed >> 8) / 16777216.0;
        seed = seed * 1664525u + 1013904223u;
        const double b = (seed >> 8) / 16777216.0;
        return std::sqrt(-2.0 * std::log(std::max(1e-12, a))) * std::cos(6.283185307 * b);
    };
    for (size_t k = 2; k + 2 < N; ++k)
    {
        const Sample& s = samples[k];
        double acc[3];
        for (int i = 0; i < 3; ++i)
        {
            // second difference over +-2 samples (10 ms): a = (p+2 - 2p + p-2) / (2dt)^2
            acc[i] = (samples[k + 2].p[i] - 2.0 * s.p[i] + samples[k - 2].p[i]) / (4.0 * dt * dt);
        }
        acc[2] += 9.81;   // specific force: gravity included
        // angular velocity: dR = R_k^T R_k+1 ~ I + [w]x dt
        const Sample& a = samples[k - 1];
        const Sample& b = samples[k + 1];
        auto dot = [](const double u[3], const double v[3]) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };
        const double wx = (dot(s.Z, b.Y) - dot(s.Z, a.Y)) / (2.0 * dt);
        const double wy = (dot(s.X, b.Z) - dot(s.X, a.Z)) / (2.0 * dt);
        const double wz = (dot(s.Y, b.X) - dot(s.Y, a.X)) / (2.0 * dt);
        const double fx = dot(acc, s.X), fy = dot(acc, s.Y), fz = dot(acc, s.Z);
        std::fprintf(fi, "%.4f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n", s.t, wx + 0.002 * noise(), wy + 0.002 * noise(),
                     wz + 0.002 * noise(), fx + 0.02 * noise(), fy + 0.02 * noise(), fz + 0.02 * noise());
        double q[4];
        rotToQuat(s.X, s.Y, s.Z, q);
        std::fprintf(fg, "%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n", s.t, s.p[0], s.p[1], s.p[2], q[0], q[1], q[2], q[3]);
    }
    std::fclose(fi);
    std::fclose(fg);
    std::printf("SLAM export: %lld scans, %.1f M points, %zu IMU samples, %.0f s, %.1f km -> %s\n", nScans, nPts / 1e6,
                N, sim.time, sim.distance / 1000.0, dir.c_str());
    return 0;
}

// glancing hit on the highway guard rail at 120 km/h: the car must scrape along, slow down gradually,
// take side damage, and not be stopped dead
int testScrape(DriveSim& sim)
{
    const RoadModel& road = sim.world.road();
    // longest open-air highway stretch (guard rails on both sides)
    double s0 = -1.0, best = 0.0, runStart = -1.0;
    for (double s = 0.0; s < road.length(); s += 5.0)
    {
        const bool ok = road.zoneAt(s) == RoadModel::Zone::Highway && !sim.world.inTunnel(s) && sim.world.guardrailAt(s, 1);
        if (ok && runStart < 0.0)
            runStart = s;
        if ((!ok || s + 5.0 >= road.length()) && runStart >= 0.0)
        {
            if (s - runStart > best)
            {
                best = s - runStart;
                s0 = runStart + 10.0;
            }
            runStart = -1.0;
        }
    }
    std::printf("SCRAPE test stretch: s=%.0f, %.0f m of highway\n", s0, best);
    if (s0 < 0.0)
    {
        std::printf("SCRAPE: no straight highway stretch with a guard rail found\n");
        return 1;
    }
    sim.setVehicleType(VehicleShapes::TypeSports);
    const double oldS = sim.startS;
    const int oldLane = sim.startLane;
    sim.startS = s0;
    sim.startLane = 2;
    sim.trafficEnabled = false;
    sim.reset();
    sim.autopilotEnabled = false;
    int fails = 0;
    // 120 km/h in the outer lane, pointing 8 degrees toward the right-hand guard rail
    {
        const double h = sim.vehicle.heading() - 0.14;
        sim.vehicle.setPose(sim.vehicle.x(), sim.vehicle.y(), h);
        sim.vehicle.applyImpulse(33.3 * std::cos(h), 33.3 * std::sin(h), 0.0);
    }
    VehicleModel::Command c;
    c.throttle = 0.25;
    c.steer = 0.0;
    const double vStart = sim.vehicle.speed();
    double tFirst = -1.0, xFirst = 0.0, yFirst = 0.0, vAt1s = -1.0;
    for (int k = 0; k < 600 && !sim.crash.happened; ++k)
    {
        if (tFirst >= 0.0)
            c.steer = -0.01;   // keep leaning on it
        sim.step(0.02, c);
        if (tFirst < 0.0 && sim.impacts > 0)
        {
            tFirst = sim.time;
            xFirst = sim.vehicle.x();
            yFirst = sim.vehicle.y();
            std::printf("SCRAPE first contact: %s at %.0f km/h, closing %.1f km/h, damage %.1f%%\n",
                        World::kindNameKo(sim.lastImpact.kind), sim.lastImpact.speedKmh, sim.lastImpact.relSpeedKmh,
                        sim.damagePct);
        }
        if (tFirst >= 0.0 && vAt1s < 0.0 && sim.time - tFirst >= 1.0)
            vAt1s = sim.vehicle.speed();
        if (tFirst >= 0.0 && sim.time - tFirst > 3.0)
            break;
    }
    const double slid = tFirst >= 0.0 ? std::hypot(sim.vehicle.x() - xFirst, sim.vehicle.y() - yFirst) : 0.0;
    std::printf("SCRAPE start %.0f km/h, 1 s after contact %.0f km/h, now %.0f km/h, moved %.0f m along the rail, "
                "impacts %d, damage %.1f%%, scraping %s, wrecked %d\n",
                vStart * 3.6, vAt1s * 3.6, sim.vehicle.speed() * 3.6, slid, sim.impacts, sim.damagePct,
                sim.time - sim.lastScrapeTime < 1.0 ? "yes" : "no", sim.crash.happened ? 1 : 0);
    if (tFirst < 0.0 || sim.crash.happened || vAt1s < 0.5 * vStart || vAt1s > vStart - 1.0 || slid < 20.0 || sim.damagePct < 1.0)
    {
        std::printf("  FAIL: expected a gradual slow-down along the rail with some damage\n");
        ++fails;
    }
    sim.startS = oldS;
    sim.startLane = oldLane;
    sim.trafficEnabled = true;
    sim.setVehicleType(0);
    return fails;
}

// off-road over the hills at speed: the car must leave the ground over sharp crests and land again
int testJump(DriveSim& sim)
{
    sim.setVehicleType(VehicleShapes::TypeSports);
    sim.trafficEnabled = false;
    int flights = 0;
    double maxAir = 0.0, maxH = 0.0, worstLanding = 0.0;
    unsigned seed = 12345;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xFFFF) / 65535.0; };
    for (int trial = 0; trial < 40; ++trial)
    {
        // a random off-road start on a slope
        double x = 0, y = 0, h = 0;
        for (int k = 0; k < 200; ++k)
        {
            x = -600.0 + 1200.0 * rnd();
            y = -600.0 + 1200.0 * rnd();
            h = 6.2832 * rnd();
            if (sim.world.terrainHeight(x, y) > 3.0)
                break;
        }
        sim.reset();
        sim.vehicle.setPose(x, y, h);
        sim.vehicle.applyImpulse(38.0 * std::cos(h), 38.0 * std::sin(h), 0.0);
        VehicleModel::Command c;
        c.throttle = 1.0;
        const unsigned l0 = sim.landingSerial;
        for (int k = 0; k < 300 && !sim.crash.happened && sim.impacts == 0; ++k)
        {
            sim.step(0.01, c);
            if (sim.landingSerial != l0 || sim.airborne)
            {
                maxAir = std::max(maxAir, sim.airTime);
                maxH = std::max(maxH, sim.lastAirHeight);
            }
        }
        if (sim.landingSerial != l0)
        {
            flights += static_cast<int>(sim.landingSerial - l0);
            worstLanding = std::min(worstLanding, sim.lastLandingSpeed);
        }
    }
    std::printf("JUMP off-road 40 runs at 137 km/h: %d flights, longest %.2f s, highest %.2f m, hardest touch-down %.1f m/s\n",
                flights, maxAir, maxH, worstLanding);
    sim.trafficEnabled = true;
    sim.setVehicleType(0);
    return flights > 0 ? 0 : 1;
}

// vehicle dynamics: hand-brake turn and power oversteer with / without the driver aids
int testDynamics()
{
    int fails = 0;
    const auto& sp = VehicleShapes::spec(VehicleShapes::TypeSports);
    auto make = [&](bool aids) {
        VehicleModel v;
        v.mass = sp.mass; v.enginePower = sp.power; v.maxAccel = sp.maxAccel; v.dragArea = sp.dragArea;
        v.maxSpeed = sp.maxSpeed; v.maxBrake = sp.maxBrake; v.wheelBase = sp.wheelBase;
        v.idleRpm = sp.idleRpm; v.peakPowerRpm = sp.peakPowerRpm; v.redlineRpm = sp.redlineRpm;
        v.wheelRadius = sp.wheelRadius; v.cgHeight = sp.cgHeight; v.frontShare = sp.frontShare;
        v.driverAids = aids;
        v.configureDrivetrain();
        v.reset(0, 0, 0);
        return v;
    };
    const double dt = 0.01;
    double betaHb[2] = { 0, 0 }, betaPw[2] = { 0, 0 };
    for (int aids = 0; aids < 2; ++aids)
    {
        // 1) hand-brake turn at 60 km/h
        VehicleModel v = make(aids == 1);
        VehicleModel::Command c;
        c.throttle = 1.0;
        while (v.speed() < 60.0 / 3.6)
            v.step(dt, c);
        for (double t = 0.0; t < 3.0; t += dt)
        {
            c = VehicleModel::Command{};
            c.steer = 0.5 * v.maxSteer;
            c.handbrake = t < 0.7;
            c.throttle = t >= 0.7 ? 0.6 : 0.0;
            v.step(dt, c);
            if (std::hypot(v.speed(), v.lateralSpeed()) > 4.0)
                betaHb[aids] = std::max(betaHb[aids], std::abs(std::atan2(v.lateralSpeed(), std::abs(v.speed()))));
        }
        // 2) power oversteer: 45 km/h, full steer + full throttle
        v = make(aids == 1);
        c = VehicleModel::Command{};
        c.throttle = 0.3;
        while (v.speed() < 45.0 / 3.6)
            v.step(dt, c);
        for (double t = 0.0; t < 2.5; t += dt)
        {
            c = VehicleModel::Command{};
            c.steer = 0.6 * v.maxSteer;
            c.throttle = 1.0;
            v.step(dt, c);
            if (std::getenv("DYNDBG") && (static_cast<int>(t * 100) % 10) == 0)
                std::printf("  aids %d t=%.1f vx %.1f vy %.2f r %.2f steer %.2f aF %.2f aR %.2f gear %d esc %d tl %d\n", aids, t,
                            v.speed(), v.lateralSpeed(), v.yawRate(), v.steer(), v.slipAngleFront(), v.slipAngleRear(),
                            v.gear(), v.escActive() ? 1 : 0, v.tractionLimited() ? 1 : 0);
            if (std::hypot(v.speed(), v.lateralSpeed()) > 4.0)
                betaPw[aids] = std::max(betaPw[aids], std::abs(std::atan2(v.lateralSpeed(), std::abs(v.speed()))));
        }
    }
    std::printf("DYN sports car, max body slip angle: hand-brake turn aids off %.0f deg / on %.0f deg, "
                "power oversteer aids off %.0f deg / on %.0f deg\n",
                betaHb[0] * 57.3, betaHb[1] * 57.3, betaPw[0] * 57.3, betaPw[1] * 57.3);
    if (betaHb[0] < 0.25 || betaPw[1] > betaPw[0])
    {
        std::printf("  FAIL: no drift without the aids, or the aids do not stabilize\n");
        ++fails;
    }
    return fails;
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "--accel")
        return testAcceleration(true);
    if (argc > 1 && std::string(argv[1]) == "--dyn")
        return testDynamics();

    // on the heap: DriveSim is large (LiDAR buffers) and would overflow the 1 MB main-thread stack
    auto simOwner = std::make_unique<DriveSim>();
    DriveSim& sim = *simOwner;
    sim.init();

    if (argc > 1 && std::string(argv[1]) == "--bench")
    {
        benchLidar(sim);
        return 0;
    }
    if (argc > 2 && std::string(argv[1]) == "--record")
        return recordDataset(sim, argc, argv);
    if (argc > 2 && std::string(argv[1]) == "--slamexport")   // mapping drive for FAST-LIO: <dir> [speed m/s]
        return exportSlam(sim, std::filesystem::path(argv[2]).u8string(), argc > 3 ? std::atof(argv[3]) : 15.0);
    if (argc > 1 && std::string(argv[1]) == "--net")   // drive the street network (viaduct, underpass, junctions)
        return testNet(sim, argc > 2 ? std::atof(argv[2]) : 15.0);
    if (argc > 1 && std::string(argv[1]) == "--crash")   // impacts / knocked objects / wreck, every vehicle type
    {
        int f = testKnockProps(sim);
        for (int type = 0; type < VehicleShapes::TypeCount; ++type)
        {
            sim.setVehicleType(type);
            std::printf("##### vehicle type %d #####\n", type);
            f += runManualCrash(sim);
        }
        sim.setVehicleType(0);
        return f;
    }
    if (argc > 1 && std::string(argv[1]) == "--jump")
        return testJump(sim);
    if (argc > 1 && std::string(argv[1]) == "--scrape")
        return testScrape(sim);
    if (argc > 1 && std::string(argv[1]) == "--road")
        return testRoad(sim.world.road());
    if (argc > 2 && std::string(argv[1]) == "--lap")   // one autopilot lap: cruise speed [m/s] [vehicle type]
    {
        if (argc > 3)
            sim.setVehicleType(std::atoi(argv[3]));
        return runAutopilot(sim, std::atof(argv[2]));
    }

    for (const auto& [pid, grid] : sim.world.pidGrids())
    {
        double range[2] = { 0, 0 };
        if (auto* s = grid->GetCellData()->GetScalars())
            s->GetRange(range);
        std::printf("PID %2d %-28s cells=%7lld material %g..%g\n", pid, World::pidName(pid),
                    static_cast<long long>(grid->GetNumberOfCells()), range[0], range[1]);
    }

    int fails = testQuaternion() != 0;
    fails += testRoad(sim.world.road()) != 0;
    fails += testAcceleration(false) != 0;
    fails += testDynamics() != 0;
    fails += testScrape(*simOwner) != 0;
    fails += testKnockProps(*simOwner) != 0;
    fails += testNet(sim, 15.0) != 0;
    for (double cruise : { 8.0, 12.0, 16.0, 22.0 })
        fails += runAutopilot(sim, cruise) != 0;
    fails += runManualCrash(sim) != 0;
    // the other vehicle types: one lap each + a crash
    for (int type = 1; type < 4; ++type)
    {
        sim.setVehicleType(type);
        std::printf("\n##### vehicle type %d #####", type);
        fails += runAutopilot(sim, 16.0) != 0;
        fails += runManualCrash(sim) != 0;
    }
    sim.setVehicleType(0);

    std::printf("\n%s (%d failing runs)\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails;
}
