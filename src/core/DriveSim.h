#pragma once

#include <vector>

#include "AutoPilot.h"
#include "CrashDeformer.h"
#include "LidarSensor.h"
#include "VehicleModel.h"
#include "World.h"

class vtkTransform;

// Everything that is not rendering: world, vehicle dynamics, traffic, LiDAR, autopilot, crash.
// Used by the Qt application and by the headless test (autopilot_sim).
class DriveSim
{
public:
    struct Pose { double x, y, heading, z = 0.0, pitch = 0.0; };

    struct CrashInfo
    {
        bool happened = false;
        double speed = 0.0;              // impact speed [m/s]
        double contactWorld[3] = { 0, 0, 0 };
        double impactLocal[3] = { 0, 0, 0 };
        double normalLocal[3] = { 1, 0, 0 };
        int npc = -1;                    // index of the traffic car that was hit, or -1
        ObstacleKind kind = ObstacleKind::Wall;
    };

    // Crash analysis for non-experts (all texts in Korean)
    struct CrashReport
    {
        double speedKmh = 0;        // impact speed
        double severity = 0;        // how hard the object is (0..1): cone 0.15 ... wall 1.0
        double effectiveKmh = 0;    // speed * severity -> used for deformation / grading
        double energyKJ = 0;        // energy dissipated by the collision
        double dropHeightM = 0;     // same impact as falling from this height
        double maxCrushM = 0;       // largest deformation
        double intrusionM = 0;      // how far the cabin (driver space) was pushed in
        double damagedPct = 0;      // % of body elements deformed more than 3 cm
        double relSpeedKmh = 0;     // closing speed along the contact normal
        double objectMassKg = 0;    // 0 = fixed in the ground
        double carKJ = 0;           // energy absorbed by the car's body in this impact
        double damagePct = 0;       // accumulated damage, 100 % = wrecked
        int impacts = 0;
        bool wrecked = false;
        double maxStrainPct = 0;    // largest effective strain of a body element [%]
        double peakG = 0;           // peak deceleration of the crash pulse [g]
        double avgG = 0;            // mean deceleration [g]
        double pulseMs = 0;         // time from contact to maximum crush [ms]
        int grade = 0;              // 1 minor, 2 moderate, 3 severe, 4 total loss
        const char* gradeKo = "";
        const char* gradeEn = "";
        const char* sideKo = "";
        const char* objectKo = "";
        const char* injuryKo = "";
        const char* repairKo = "";
    };
    static constexpr double kTruckMassKg = 3500.0;

    // traffic car: lane follower with simple gap keeping.
    // Cars the player has left far behind (and oncoming cars that have passed) are recycled ahead of
    // the player, so there is always traffic around -> more interesting LiDAR data.
    struct Npc
    {
        double s = 0;                         // station along the loop
        double cx = 0, cy = 0, heading = 0;   // center of the car (world)
        double speed = 0, cruise = 5;
        double factor = 1.0;                  // cruise = speed limit x factor
        int road = 0;                         // 0 main loop, k >= 1: street k - 1 of the network
        double z = 0.0, pitch = 0.0;          // on the viaduct / in the underpass
        // turning through a junction (Bezier from the lane end to the next street's lane start)
        bool inJunction = false;
        double jt = 0.0, jLen = 1.0, jp[4][2] = {}, jz[2] = {};
        int nextRoad = 0, nextDir = 1, nextLane = 0;
        double nextS = 0.0;
        double crashedFor = 0.0;              // > 0: was hit, stands still
        // dynamics: a car that was hit leaves its lane and moves as a free body (single-track model
        // with locked / ABS brakes): body velocity vx (forward), vy (left), yaw rate r
        bool free = false;
        double vx = 0.0, vy = 0.0, r = 0.0;
        // body roll / pitch on the suspension (from the accelerations), like the player car
        double ax = 0.0, ay = 0.0, lastSpeed = 0.0;
        double roll = 0.0, rollRate = 0.0, bodyPitch = 0.0, bodyPitchRate = 0.0;
        void velocity(double& wx, double& wy) const;
        int lane = 0;
        int dir = 1;                          // +1 player direction, -1 oncoming
        bool active = false;                  // starts when the player gets close
        bool alive = true;                    // false: parked out of the world, waiting for a free spot
        void toTransform(vtkTransform* t) const;
    };

    void updateVertical(double dt);
    void hardLanding(double vDown);
    double m_groundZ = 0.0, m_groundPitch = 0.0, m_prevGroundZ = 0.0;
    bool m_groundValid = false;
    double m_vz = 0.0, m_airZ = 0.0, m_airPitch = 0.0;
    void setupWheels();
    void resetWheels();
    void damageWheels(const double c[3], const double n[3], double energyJ);
    void updateWheels(double dt);
    std::vector<double> m_wheelRest;
    std::vector<int> m_wheelOf;
    bool m_wheelsMoving = false;

    static constexpr double kNpcHalfL = 2.0;
    static constexpr double kNpcHalfW = 0.62;

    void init();
    void reset();
    // player vehicle model (VehicleShapes::Type): mesh, dynamics, sensor positions; resets the run
    void setVehicleType(int type);
    int vehicleType() const { return world.vehicleType(); }

    // traffic light in front of the player (city): junction index (-1 = none within 150 m),
    // distance from the front bumper to its stop line, current aspect and seconds until it changes
    struct SignalAhead
    {
        int junction = -1;
        double distance = 1e9;
        Light light = Light::Green;
        double remaining = 0.0;
        bool mustStop = false;
    };
    SignalAhead signalAhead;
    int redLightViolations = 0;
    double lastViolationTime = -100.0;
    void step(double dt, const VehicleModel::Command& manual);

    // ---- wheels and axles of the player car: bent (camber / toe), pushed in, or torn off
    struct WheelState
    {
        double hub[3] = { 0, 0, 0 };   // rest hub center (local)
        double damage = 0.0;           // 0 intact, 1 axle / suspension bent, > 1.3 torn off
        double push[2] = { 0, 0 };     // hub pushed in [m] (local x, y)
        double camber = 0.0, toe = 0.0;   // [rad]
        bool detached = false;
        double fall = 0.0;             // 0..1 animation of a wheel coming off
        double cur[4] = { 0, 0, 0, 0 };   // shown: push x, push y, camber, toe
        bool front() const { return hub[0] < 2.0; }
        bool left() const { return hub[1] < 0.5; }
    };
    std::vector<WheelState> wheels;
    // vertical motion: the car follows the road while gravity can keep it there; over a sharp crest at
    // speed or off an edge it flies (ballistic) and lands
    bool airborne = false;
    double airTime = 0.0;             // of the current / last flight [s]
    double lastLandingSpeed = 0.0;    // vertical speed at touch-down [m/s]
    double lastAirHeight = 0.0;       // highest point above the ground of the last flight [m]
    unsigned landingSerial = 0;
    bool userDriverAids = true;        // X key: ABS / TCS / ESC off for drifting (autopilot keeps them on)

    // deformation and knocked objects move toward their state in real time. step() calls it;
    // after the car is wrecked (step() does nothing any more) the UI keeps calling it.
    void animate(double dt);

    // which road surface the player is on (sets the car's height and pitch)
    enum class Surface { Main, Net, Connector, Offroad };
    Surface surface = Surface::Main;
    int netRoad = -1, netNode = -1;     // street / junction of the network under the car
    double netS = 0.0;
    int junctionTurns = 0;              // traffic cars that turned at a junction (statistics)
    // the autopilot follows the main loop only
    bool canAutopilot() const { return surface == Surface::Main; }
    const char* surfaceNameKo() const;

    // ---- damage of the player vehicle accumulated over all impacts ----
    // Every impact exchanges momentum with the object (its mass: a cone flies away, a wall stops the car,
    // a traffic car is pushed) and the car's body absorbs part of the dissipated energy.
    // damagePct = absorbed energy / energy of a 48 km/h frontal wall crash. At 100 % (or when the driver
    // space is pushed in more than kIntrusionWreck) the car is wrecked and the run ends (crash.happened).
    static constexpr double kWreckPct = 100.0;
    static constexpr double kIntrusionWreck = 0.35;
    double damagePct = 0.0;
    int impacts = 0;
    unsigned impactSerial = 0;          // increments with every impact (UI effects)
    struct ImpactInfo
    {
        ObstacleKind kind = ObstacleKind::Wall;
        int npc = -1;
        bool knocked = false;           // a loose object was knocked away
        double speedKmh = 0, relSpeedKmh = 0, objectMassKg = 0;
        double energyKJ = 0, carKJ = 0, crushAdded = 0;
        double peakG = 0, avgG = 0, pulseMs = 0;
        double contactWorld[3] = { 0, 0, 0 };
        double normalLocal[3] = { 1, 0, 0 };
        double time = 0, damageBefore = 0, damageAfter = 0;
    };
    ImpactInfo lastImpact;
    std::vector<ImpactInfo> impactLog;   // every impact of this run (crash analysis panel)
    double wreckEnergy() const;
    Footprint footprint() const;

    bool autopilotEnabled = false;
    bool trafficEnabled = true;   // false: no traffic cars (mapping runs for SLAM)

    // where reset() puts the player (station along the loop, ego-direction lane 0..2)
    double startS = 17.0;
    int startLane = 1;
    // start on a street of the network instead (street index, station); -1 = main loop
    int startNetRoad = -1;
    double startNetS = 0.0;

    // player position along the road (station s, lateral offset)
    void egoRoadPosition(double& s, double& lat) const;

    World world;
    VehicleModel vehicle;
    LidarSensor lidar;
    AutoPilot pilot;
    CrashDeformer deformer;
    std::vector<Npc> npcs;

    CrashInfo crash;
    CrashReport report;
    double time = 0.0;
    double distance = 0.0;
    std::vector<Pose> path;      // trajectory of the current run (for the ghost replay)
    bool scannedThisStep = false;
    VehicleModel::Command lastCommand;

private:
    void updateTraffic(double dt);
    void updateNetNpc(Npc& n, size_t i, double dt);
    void updateSurface();
    void updateSignalAhead();
    int m_prevSignalJ = -1;
    double m_prevSignalD = 1e9;
    bool respawnAhead(size_t i, double egoS);
    void placeNpc(Npc& n, double s) const;
    std::vector<LidarSensor::DynamicBox> dynamicBoxes() const;
    void resolveImpact(const double contactWorld[3], ObstacleKind kind, int npc, int prop);
    // long barrier (guard rail, median, wall along a road): its surface normal at the contact, pointing
    // toward the car. false = not a road barrier (use the car's face normal)
    bool barrierNormal(const double contactWorld[3], ObstacleKind kind, double nb[2]) const;
    void scrape(const double contactWorld[3], ObstacleKind kind, const double nW[3], double tx, double ty, double tl);
    double m_dt = 0.01;
    double m_scrapeJ = 0.0, m_scrapeClock = 0.0;
  public:
    double lastScrapeTime = -100.0;   // the car was grinding along a barrier (UI: sparks / sound)
    double scrapeSpeed = 0.0;
  private:
    void buildReport();
};
