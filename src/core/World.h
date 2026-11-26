#pragma once

#include <array>
#include <map>
#include <vector>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "LidarSensor.h"
#include "RoadModel.h"

class VehicleModel;

// what an obstacle is (for the crash report)
enum class ObstacleKind { Barrel, Cone, Crate, Wall, Building, Tree, TrafficLight, Sign,
                          StreetLight, ParkedCar, Car, Guardrail, TunnelWall, Pier, Ground };

// traffic light aspect (Korean 4-lamp signal: red, yellow, left arrow, green)
enum class Light { Green, Yellow, Red, RedArrow };   // RedArrow: red for straight + protected left arrow

// one lamp of a signal head (rendered by the app with a color that changes over time)
struct SignalLamp
{
    int junction;    // index into World::intersections()
    int dir;         // +1 player direction, -1 oncoming, 0 cross street
    int slot;        // 0 red, 1 yellow, 2 left arrow, 3 green
    std::vector<std::array<double, 3>> pts;   // polygon(s) in world coords
    std::vector<int> polySizes;               // the polygon split into cells (arrow = triangle + bar)
};

struct CircleObstacle { double x, y, r; ObstacleKind kind; int prop = -1; };
// oriented box: center, half extents along its local x / y, yaw [rad]
struct BoxObstacle    { double cx, cy, hx, hy, yaw; ObstacleKind kind; int prop = -1; };

// how a car experiences an object it hits
struct ObstacleProps
{
    double massKg;        // <= 0: fixed in the ground (infinite mass)
    double carShare;      // fraction of the crash energy the car's body absorbs (the rest: the object)
    double restitution;   // bounce (0 = plastic)
    double friction;      // tangential / normal impulse (scraping along a guard rail)
    bool knockable;       // light loose object: knocked away when hit
};

// vehicle footprint in local coords (bumpers, tires, mirrors), shrunk where the body is crushed
struct Footprint { double x0 = -0.08, x1 = 4.04, y0 = -0.13, y1 = 1.13; };

// loose object of the obstacle course (cone, barrel, crate): stands until hit, then flies, tumbles
// and comes to rest. Its points in the PidObstacles grid are moved directly.
struct Prop
{
    ObstacleKind kind;
    double x0, y0, yaw0;          // standing place
    double halfL, halfW, height;
    int material;
    bool knocked = false, resting = false;
    double pos[3] = { 0, 0, 0 };  // center (world)
    double vel[3] = { 0, 0, 0 };
    double spin = 0, spinRate = 0;            // yaw added since standing
    double tilt = 0, tiltRate = 0, tiltDir = 0;   // falling over about a horizontal axis (direction tiltDir)
    std::vector<long long> ids;   // point ids in the PidObstacles grid
    std::vector<double> rest;     // 3 per point: offset from the standing center
};

// Traffic car definition (driven by DriveSim)
struct NpcSpawn
{
    double s;       // start station along the loop
    int lane;
    double cruise;  // [m/s]
    int paint;      // Palette index
    int dir;        // +1: same direction as the player, -1: oncoming
    int road = 0;   // 0: main loop, k >= 1: street k - 1 of the network
};

// Scene definition: PID meshes + obstacle geometry used for collision and LiDAR, terrain, lake.
// The road is a closed loop (see RoadModel): city -> forest obstacle course -> highway -> suburb.
class World
{
public:
    enum Pid
    {
        PidCab       = 1,   // vehicle (local frame)
        PidCargo     = 2,
        PidTires     = 3,
        PidCamera    = 4,
        PidPoles     = 5,   // world: traffic lights + road signs + gantry
        PidRoad      = 6,
        PidObstacles = 7,
        PidWall      = 8,   // guard rails, median barrier
        PidMarkings  = 9,
        PidBuildings = 10,
        PidTrees     = 11,
        PidLights    = 12,
        PidGlass     = 13,  // glass of the buildings (reflective PBR material)
        PidTunnel    = 14,  // highway tunnel: walls, vault, portals
        PidBridge    = 15   // second ring road: viaduct over the highway, underpass under the suburb road
    };

    static bool isVehiclePid(int pid) { return pid >= PidCab && pid <= PidCamera; }
    static const char* pidName(int pid);       // English
    static const char* pidNameKo(int pid);     // Korean (UI)
    const char* sectionNameKo(double s) const;

    void build();

    // player vehicle model (VehicleShapes::Type). Rebuilds PIDs 1-4 IN PLACE (same grid objects:
    // actors, shadows and the crash deformer keep their pointers; call CrashDeformer setup again).
    void setVehicleType(int type);
    // wheel hub centers of the player vehicle (local frame; y < 0.5 = left side) and the tire radius
    const std::vector<std::array<double, 3>>& wheelHubs() const { return m_wheelHubs; }
    double wheelRadius() const { return m_wheelR; }
    int vehicleType() const { return m_vehicleType; }

    // ---- traffic lights (city junctions) ----
    // aspect for a direction at time t: dir +1 player direction, -1 oncoming, 0 cross street
    Light signalAt(int junction, int dir, double t) const;
    double signalCycle(int junction) const;
    // seconds until the aspect of (junction, dir) changes
    double signalRemaining(int junction, int dir, double t) const;
    // station of the stop line of a junction for a direction (+1 / -1)
    double stopLineS(int junction, int dir) const;
    // nearest junction ahead whose stop line is between 'minAhead' and 'maxAhead' meters ahead of station s
    // when driving in direction dir; returns -1 if none. dist = distance to the stop line.
    int nextStopLine(double s, int dir, double maxAhead, double& dist, double minAhead = -0.5) const;
    const std::vector<SignalLamp>& signalLamps() const { return m_lamps; }

    // ---- tunnel (highway) ----
    bool inTunnel(double s) const;
    double tunnelFrom() const { return m_tunnelFrom; }
    double tunnelTo() const { return m_tunnelTo; }
    // height of the tunnel ceiling above the road at lateral offset lat
    static double tunnelCeiling(double lat, double halfWidth);
    // emissive light strips under the tunnel ceiling (drawn unlit by the app)
    vtkUnstructuredGrid* tunnelLights() const { return m_tunnelLights; }

    const RoadModel& road() const { return m_road; }

    // ---- street network besides the main loop ----
    // Junctions (nodes) joined by curved streets (open RoadModel splines with their own lane count and
    // elevation profile). It meets downtown at two connector streets (continuations of the cross streets
    // at x = -35 and x = -165) and crosses the main loop without junctions: a viaduct over the highway
    // and an underpass under the suburb road. Outside the loop a mountain road and a dead end (lookout).
    struct NetNode
    {
        double x = 0, y = 0, z = 0, radius = 10;
        std::vector<std::pair<int, int>> ends;   // (street, 0 = its start / 1 = its end)
        const char* nameKo = "";
    };
    struct NetRoad
    {
        RoadModel road;
        int nodeA = -1, nodeB = -1;
        std::vector<char> st;        // structure per elevation sample
        std::vector<double> nat;     // natural ground along the centerline
        double limit = 50.0 / 3.6;   // speed limit [m/s]
        const char* nameKo = "";
    };
    enum NetStructure { NetGraded = 0, NetViaduct, NetTrench, NetCovered };
    const std::vector<NetRoad>& net() const { return m_net; }
    const std::vector<NetNode>& nodes() const { return m_nodes; }
    int netStructure(int road, double s) const;
    // height of the underpass cover slab top (relative to the street level) at station s
    double netCoverTop(int road, double s) const;
    // guard rail / parapet / retaining wall at station s (viaduct, underpass, high embankments)
    bool netRailAt(int road, double s, int side) const;
    double netWallLat(int road) const { return m_net[road].road.halfWidth + 1.5; }
    int viaductRoad() const { return m_viaductRoad; }
    double viaductS() const { return m_viaductS; }
    int underpassRoad() const { return m_underRoad; }
    double underpassS() const { return m_underS; }
    // connector streets from downtown (main-loop frame: cross street stations 55 / 185, lateral range)
    static constexpr double kConnLat0 = 84.0, kConnLat1 = 141.5;
    static bool inConnector(double s, double lat)
    {
        return ((s > 47.0 && s < 63.0) || (s > 177.0 && s < 193.0)) && lat > kConnLat0 - 7.0 && lat < kConnLat1 + 1.0;
    }
    // where on the network is (x, y), choosing the surface closest to the height zPrev
    struct NetPos { int road = -1; int node = -1; double s = 0, lat = 0, z = 0; };
    bool netLocate(double x, double y, double zPrev, NetPos& out) const;
    // the ground plane has no ground here (open trench of the underpass): the LiDAR must look past it
    bool groundHole(double x, double y) const;
    // terrain without the streets' cuts and fills
    double naturalHeight(double x, double y) const;
    // is (x, y) within 'margin' meters of a street edge / a junction?
    bool nearNet(double x, double y, double margin) const;
    // fast raster lookup (1 m cells) for the LiDAR worker threads:
    // 0 nothing, 1 open trench, 2 + street index: street on the ground, 255 junction
    int netMask(double x, double y) const;
    const std::map<int, vtkSmartPointer<vtkUnstructuredGrid>>& pidGrids() const { return m_pidGrids; }
    vtkUnstructuredGrid* grid(int pid) const;

    const std::vector<NpcSpawn>& npcSpawns() const { return m_npcSpawns; }
    vtkUnstructuredGrid* npcGrid(size_t i) const { return m_npcGrids[i]; }

    // all LiDAR-visible static geometry (world coords)
    vtkPolyData* obstacleSurface() const { return m_obstacleSurface; }
    // hills around the road (point colors "Colors") and the lake surface
    vtkPolyData* terrain() const { return m_terrain; }
    vtkPolyData* water() const { return m_water; }
    double terrainHeight(double x, double y) const;
    const LidarSensor::HeightField& heightField() const { return m_heightField; }

    // static obstacles: true if the vehicle footprint overlaps one; contact point in world coords.
    // prop: index of the loose object that was hit (or -1)
    // surface: 0 main loop / ground level, 1 street network (on street netRoad, -1 = in a junction),
    // 2 connector street, 3 off road
    bool checkCollision(const VehicleModel& vehicle, double contactWorld[3], ObstacleKind* kind = nullptr,
                        const Footprint* fp = nullptr, int* prop = nullptr, int surface = 0, int netRoad = -1) const;
    static const char* kindNameKo(ObstacleKind k);
    static ObstacleProps obstacleProps(ObstacleKind k);
    // speed limit [m/s] for traffic at station s (city 50 km/h ... highway 100 km/h)
    double speedLimit(double s) const;

    // ---- loose objects of the obstacle course ----
    const std::vector<Prop>& props() const { return m_props; }
    // knock prop i away with this velocity [m/s] (world)
    void knockProp(int i, const double vel[3]);
    // fly / tumble the knocked props; returns true if the PidObstacles grid changed
    bool updateProps(double dt);
    void resetProps();
    // take the loose objects out of the world (mapping drives for SLAM)
    void hideProps();

    // Palette material of the ground at (x, y): asphalt, lane paint, crosswalk, sidewalk, grass, water.
    // Used by the LiDAR intensity model (lane markings light up in the point cloud).
    int groundMaterial(double x, double y) const;
    const std::vector<double>& intersections() const { return m_intersections; }   // stations

    // footprint vs. oriented box (center, heading, half extents)
    static bool footprintHitsBox(const VehicleModel& v, double cx, double cy, double heading,
                                 double halfL, double halfW, double contactWorld[3], const Footprint* fp = nullptr);

    // start places for the UI: 0 city, 1 obstacle course, 2 highway, 3 suburb
    double placeS(int place) const;
    double startS() const { return placeS(0); }
    int startLane() const { return 1; }

    // where traffic may drive / respawn (dir: +1 player direction, -1 oncoming)
    bool trafficAllowed(double s, int dir) const;
    bool inObstacleCourse(double s) const;
    double courseFrom() const { return m_courseFrom; }
    double courseTo() const { return m_courseTo; }

    // static obstacles (for ground-truth labels in recorded datasets)
    const std::vector<CircleObstacle>& circles() const { return m_circles; }
    const std::vector<BoxObstacle>& boxes() const { return m_boxes; }

    bool guardrailAt(double s, int side) const;   // side +1 right, -1 left

private:
    std::vector<std::array<double, 3>> m_wheelHubs;
    double m_wheelR = 0.3;
    bool medianBarrierAt(double s) const;
    bool nearIntersection(double s, double margin) const;
    void buildTerrain();
    vtkSmartPointer<vtkPolyData> buildTunnel();

    RoadModel m_road;
    std::vector<NetRoad> m_net;
    std::vector<NetNode> m_nodes;
    std::vector<unsigned char> m_netMask;
    double m_maskX0 = 0, m_maskY0 = 0;
    int m_maskNx = 0, m_maskNy = 0;
    int m_viaductRoad = -1, m_underRoad = -1;
    double m_viaductS = 0.0, m_underS = 0.0;
    bool m_netReady = false;
    void buildNetwork();
    struct NetParts
    {
        std::vector<vtkSmartPointer<vtkUnstructuredGrid>> roads, marks, walls, bridge, lidar, lights, props;
        std::vector<std::array<double, 4>> poles;   // street light poles: x, y, z0, height
    };
    NetParts buildNetGeometry();
    std::map<int, vtkSmartPointer<vtkUnstructuredGrid>> m_pidGrids;
    std::vector<NpcSpawn> m_npcSpawns;
    std::vector<vtkSmartPointer<vtkUnstructuredGrid>> m_npcGrids;
    vtkSmartPointer<vtkPolyData> m_obstacleSurface;
    vtkSmartPointer<vtkPolyData> m_terrain;
    vtkSmartPointer<vtkPolyData> m_water;
    LidarSensor::HeightField m_heightField;        // hills for the LiDAR
    std::vector<CircleObstacle> m_circles;
    std::vector<Prop> m_props;
    std::vector<BoxObstacle> m_boxes;
    std::vector<double> m_intersections;
    double m_courseFrom = 0.0, m_courseTo = 0.0;
    double m_tunnelFrom = 0.0, m_tunnelTo = 0.0;
    vtkSmartPointer<vtkPolyData> m_tunnelHill;          // earth over the tunnel (merged into the terrain)
    vtkSmartPointer<vtkUnstructuredGrid> m_tunnelLights;
    std::vector<SignalLamp> m_lamps;
    int m_vehicleType = 0;
    // paved areas off the main road (back streets, gas station) in road coordinates, for groundMaterial()
    struct Paved { double s0, s1, lat0, lat1; int material; };
    std::vector<Paved> m_paved;
};
