#include "World.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkDoubleArray.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyDataNormals.h>
#include <vtkThreshold.h>
#include <vtkTriangleFilter.h>
#include <vtkUnsignedCharArray.h>

#include "Palette.h"
#include "SceneBuilder.h"
#include "VehicleModel.h"
#include "VehicleShapes.h"

using SceneBuilder::Grid;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

enum class ObstacleType { Barrel, Box, Cone };

struct ObstacleDef
{
    double ds;      // station offset from the first gate
    int lane;       // ego-direction lane 0..2 (0 = next to the median)
    double dl;      // offset from the lane center
    ObstacleType type;
};

// Obstacle course (forest). Every gate leaves one lane open, always next to the lane that was
// open at the previous gate -> one lane change per gate.
const ObstacleDef kCourse[] = {
    {   0.0, 1,  0.0, ObstacleType::Box    }, {   0.0, 2,  0.0, ObstacleType::Barrel },
    {   1.0, 2, -0.5, ObstacleType::Cone   },                                            // open: 0
    {  45.0, 0,  0.0, ObstacleType::Box    }, {  45.0, 2,  0.0, ObstacleType::Cone   }, // open: 1
    {  90.0, 0,  0.0, ObstacleType::Barrel }, {  90.9, 0, -0.45, ObstacleType::Cone  },
    {  90.0, 1,  0.0, ObstacleType::Box    },                                            // open: 2
    { 135.0, 2,  0.0, ObstacleType::Box    }, { 135.0, 0,  0.0, ObstacleType::Barrel }, // open: 1
    { 180.0, 1,  0.0, ObstacleType::Box    }, { 181.6, 1,  0.30, ObstacleType::Cone  }, // open: 0, 2
};
constexpr double kGate0 = 45.0;   // first gate, meters into the forest zone

constexpr double kBarrelRadius = 0.35;
constexpr double kBarrelHeight = 1.0;
constexpr double kConeRadius   = 0.30;
constexpr double kConeHeight   = 0.8;
constexpr double kBoxSize      = 1.2;

// city grid (stations along the straight)
const double kIntersections[] = { 55.0, 120.0, 185.0, 250.0, 315.0, 380.0 };
constexpr double kCrossHalf   = 6.0;    // half width of a cross street (2 lanes per direction)
constexpr double kCrossLength = 84.0;   // cross streets run to the far side of the back streets
constexpr double kBackLat     = 78.0;   // back streets parallel to the main road (center lateral offset)
constexpr double kWalkIn  = 9.6;        // sidewalk lateral range
constexpr double kWalkOut = 13.0;
constexpr double kCrosswalk = 8.5;      // zebra crossing center, distance from the junction center
constexpr double kStopLine  = 10.6;
constexpr double kMarkGap   = 10.9;     // no lane markings within this distance of a junction
constexpr double kSuburbWalkOut = 11.6;

// lake inside the loop
constexpr double kLakeX = -200.0, kLakeY = 300.0, kLakeRx = 105.0, kLakeRy = 95.0;   // room for the ring road east of it
constexpr double kWaterE = 1.15;        // water surface reaches this normalized ellipse radius

// deterministic pseudo random numbers (same map every run)
struct Rng
{
    uint32_t s = 12345u;
    double next() { s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216.0; }
    double range(double a, double b) { return a + (b - a) * next(); }
};

double smoothstep(double a, double b, double x)
{
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double hash2(int x, int y)
{
    uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (h ^ (h >> 16)) / 4294967295.0;
}

double valueNoise(double x, double y)
{
    const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
    double fx = x - ix, fy = y - iy;
    fx = fx * fx * (3 - 2 * fx);
    fy = fy * fy * (3 - 2 * fy);
    const double a = hash2(ix, iy), b = hash2(ix + 1, iy), c = hash2(ix, iy + 1), d = hash2(ix + 1, iy + 1);
    const double ab = a + (b - a) * fx, cd = c + (d - c) * fx;
    return ab + (cd - ab) * fy;
}

double fbm(double x, double y)
{
    double sum = 0.0, amp = 0.5, norm = 0.0;
    for (int o = 0; o < 5; ++o)
    {
        sum += amp * valueNoise(x, y);
        norm += amp;
        x = x * 2.03 + 17.1;
        y = y * 2.03 - 9.7;
        amp *= 0.5;
    }
    return sum / norm;
}

double lakeE(double x, double y)
{
    const double dx = (x - kLakeX) / kLakeRx, dy = (y - kLakeY) / kLakeRy;
    return dx * dx + dy * dy;
}

// split a grid into the cells with material 'mat' and the rest
void splitMaterial(vtkUnstructuredGrid* in, int mat, Grid& with, Grid& without)
{
    for (int pass = 0; pass < 2; ++pass)
    {
        vtkNew<vtkThreshold> th;
        th->SetInputData(in);
        th->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, "ElementScalar");
        th->SetThresholdFunction(vtkThreshold::THRESHOLD_BETWEEN);
        th->SetLowerThreshold(mat - 0.5);
        th->SetUpperThreshold(mat + 0.5);
        th->SetInvert(pass == 1);
        th->Update();
        Grid g = Grid::New();
        g->ShallowCopy(th->GetOutput());
        (pass == 0 ? with : without) = g;
    }
}

// fast batched primitives (one polydata for thousands of trees)
struct PolyBatch
{
    vtkNew<vtkPoints> pts;
    vtkNew<vtkCellArray> polys;

    // cone: base center (x, y, z), radius, height
    void cone(double x, double y, double z, double r, double h, int res = 10)
    {
        const vtkIdType apex = pts->InsertNextPoint(x, y, z + h);
        const vtkIdType base = pts->GetNumberOfPoints();
        for (int i = 0; i < res; ++i)
        {
            const double a = 2.0 * kPi * i / res;
            pts->InsertNextPoint(x + r * std::cos(a), y + r * std::sin(a), z);
        }
        for (int i = 0; i < res; ++i)
        {
            const vtkIdType tri[3] = { base + i, base + (i + 1) % res, apex };
            polys->InsertNextCell(3, tri);
        }
        std::vector<vtkIdType> cap(res);
        for (int i = 0; i < res; ++i)
            cap[i] = base + res - 1 - i;
        polys->InsertNextCell(res, cap.data());
    }

    // cylinder from z0 to z0 + h
    void cylinder(double x, double y, double z0, double r, double h, int res = 8)
    {
        const vtkIdType base = pts->GetNumberOfPoints();
        for (int i = 0; i < res; ++i)
        {
            const double a = 2.0 * kPi * i / res;
            pts->InsertNextPoint(x + r * std::cos(a), y + r * std::sin(a), z0);
            pts->InsertNextPoint(x + r * std::cos(a), y + r * std::sin(a), z0 + h);
        }
        for (int i = 0; i < res; ++i)
        {
            const int j = (i + 1) % res;
            const vtkIdType q[4] = { base + 2 * i, base + 2 * j, base + 2 * j + 1, base + 2 * i + 1 };
            polys->InsertNextCell(4, q);
        }
        std::vector<vtkIdType> top(res);
        for (int i = 0; i < res; ++i)
            top[i] = base + 2 * i + 1;
        polys->InsertNextCell(res, top.data());
    }

    Grid grid(int material)
    {
        vtkNew<vtkPolyData> pd;
        pd->SetPoints(pts);
        pd->SetPolys(polys);
        return SceneBuilder::fromPolyData(pd, material);
    }
};

// vehicle footprint perimeter in local coords (bumpers, tires and mirrors included; see Footprint)
constexpr double kLzMid = 0.6;

template <class F>
bool forPerimeter(const Footprint& fp, int n, F&& f)
{
    const double ex = fp.x1 - fp.x0, ey = fp.y1 - fp.y0, perim = 2.0 * (ex + ey);
    for (int i = 0; i < n; ++i)
    {
        const double t = perim * i / n;
        double l[3] = { 0.0, 0.0, kLzMid };
        if (t < ex)               { l[0] = fp.x0 + t;             l[1] = fp.y0; }
        else if (t < ex + ey)     { l[0] = fp.x1;                 l[1] = fp.y0 + (t - ex); }
        else if (t < 2 * ex + ey) { l[0] = fp.x1 - (t - ex - ey); l[1] = fp.y1; }
        else                      { l[0] = fp.x0;                 l[1] = fp.y1 - (t - 2 * ex - ey); }
        if (f(l))
            return true;
    }
    return false;
}
}

const char* World::pidName(int pid)
{
    switch (pid)
    {
    case PidCab:       return "Cab";
    case PidCargo:     return "Cargo";
    case PidTires:     return "Wheels";
    case PidCamera:    return "Sensors";
    case PidPoles:     return "Traffic lights, signs, gantry";
    case PidRoad:      return "Roads";
    case PidObstacles: return "Obstacles & parked cars";
    case PidWall:      return "Guard rails & median barrier";
    case PidMarkings:  return "Road markings";
    case PidBuildings: return "Buildings & houses";
    case PidTrees:     return "Trees";
    case PidLights:    return "Street lights & sidewalks";
    case PidGlass:     return "Glass";
    case PidTunnel:    return "Tunnel";
    case PidBridge:    return "Viaduct / underpass";
    }
    return "";
}

const char* World::pidNameKo(int pid)
{
    switch (pid)
    {
    case PidCab:       return u8"차체 앞·운전석 (변형됨)";
    case PidCargo:     return u8"차체 뒤·적재함 (변형됨)";
    case PidTires:     return u8"바퀴";
    case PidCamera:    return u8"LiDAR + 카메라";
    case PidPoles:     return u8"신호등·표지판";
    case PidRoad:      return u8"도로 (교차로 포함)";
    case PidObstacles: return u8"장애물·주차된 차";
    case PidWall:      return u8"가드레일·중앙분리대";
    case PidMarkings:  return u8"차선·횡단보도";
    case PidBuildings: return u8"건물·주택";
    case PidTrees:     return u8"나무";
    case PidLights:    return u8"가로등·인도";
    case PidGlass:     return u8"건물 유리";
    case PidTunnel:    return u8"터널";
    case PidBridge:    return u8"고가도로 · 지하차도 · 교차로";
    }
    return "";
}

const char* World::kindNameKo(ObstacleKind k)
{
    switch (k)
    {
    case ObstacleKind::Barrel:       return u8"드럼통";
    case ObstacleKind::Cone:         return u8"라바콘";
    case ObstacleKind::Crate:        return u8"나무 상자";
    case ObstacleKind::Wall:         return u8"콘크리트 중앙분리대";
    case ObstacleKind::Building:     return u8"건물";
    case ObstacleKind::Tree:         return u8"나무";
    case ObstacleKind::TrafficLight: return u8"신호등";
    case ObstacleKind::Sign:         return u8"표지판";
    case ObstacleKind::StreetLight:  return u8"가로등";
    case ObstacleKind::ParkedCar:    return u8"주차된 차";
    case ObstacleKind::Car:          return u8"주행 중인 차";
    case ObstacleKind::Guardrail:    return u8"가드레일";
    case ObstacleKind::TunnelWall:   return u8"터널 벽";
    case ObstacleKind::Pier:         return u8"교각";
    case ObstacleKind::Ground:       return u8"땅 (착지)";
    }
    return "";
}

ObstacleProps World::obstacleProps(ObstacleKind k)
{
    //                         mass kg  car share  bounce  friction  knockable
    switch (k)
    {
    case ObstacleKind::Cone:         return { 4.0,    0.25, 0.50, 0.2, true };    // plastic: bounces off
    case ObstacleKind::Barrel:       return { 50.0,   0.30, 0.45, 0.2, true };    // half-filled plastic drum
    case ObstacleKind::Crate:        return { 60.0,   0.35, 0.45, 0.3, true };    // empty wooden box 1.2 m
    case ObstacleKind::Sign:         return { 0.0,    0.4,  0.10, 0.3, false };
    case ObstacleKind::StreetLight:  return { 0.0,    0.9,  0.10, 0.3, false };
    case ObstacleKind::TrafficLight: return { 0.0,    0.9,  0.10, 0.3, false };
    case ObstacleKind::Tree:         return { 0.0,    1.0,  0.10, 0.3, false };
    case ObstacleKind::Building:     return { 0.0,    1.0,  0.10, 0.3, false };
    case ObstacleKind::Wall:         return { 0.0,    1.0,  0.15, 0.2, false };
    case ObstacleKind::TunnelWall:   return { 0.0,    1.0,  0.15, 0.2, false };
    case ObstacleKind::Pier:         return { 0.0,    1.0,  0.10, 0.3, false };
    case ObstacleKind::Ground:       return { 0.0,    1.0,  0.10, 0.6, false };   // hard landing
    case ObstacleKind::Guardrail:    return { 0.0,    0.6,  0.30, 0.15, false };   // deflects the car
    case ObstacleKind::ParkedCar:    return { 0.0,    0.5,  0.15, 0.3, false };    // braked: does not move
    case ObstacleKind::Car:          return { 1450.0, 0.5,  0.15, 0.3, false };    // traffic car
    }
    return { 0.0, 1.0, 0.1, 0.3, false };
}

double World::speedLimit(double s) const
{
    switch (m_road.zoneAt(s))
    {
    case RoadModel::Zone::City:    return 50.0 / 3.6;
    case RoadModel::Zone::Suburb:  return 50.0 / 3.6;
    case RoadModel::Zone::Corner:  return 60.0 / 3.6;
    case RoadModel::Zone::Forest:  return 60.0 / 3.6;
    case RoadModel::Zone::Highway: return 100.0 / 3.6;
    default: break;
    }
    return 60.0 / 3.6;
}

void World::knockProp(int i, const double vel[3])
{
    if (i < 0 || i >= static_cast<int>(m_props.size()) || m_props[i].knocked)
        return;
    Prop& p = m_props[i];
    p.knocked = true;
    p.resting = false;
    for (int k = 0; k < 3; ++k)
        p.vel[k] = vel[k];
    const double v = std::hypot(vel[0], vel[1]);
    // falls over in the direction it is pushed, spins a little
    p.tiltDir = std::atan2(vel[1], vel[0]);
    p.tiltRate = std::min(14.0, 2.0 + 1.2 * v);
    p.spinRate = (std::fmod(p.x0 * 13.7 + p.y0 * 7.1, 1.0) - 0.5) * 2.0 * std::min(10.0, 0.8 * v);
}

void World::resetProps()
{
    const double gz = m_road.groundZ;
    for (auto& p : m_props)
    {
        p.knocked = p.resting = false;
        p.pos[0] = p.x0;
        p.pos[1] = p.y0;
        p.pos[2] = gz + 0.5 * p.height;
        p.vel[0] = p.vel[1] = p.vel[2] = 0.0;
        p.spin = p.spinRate = p.tilt = p.tiltRate = 0.0;
    }
    if (auto* g = grid(PidObstacles))
    {
        vtkPoints* pts = g->GetPoints();
        for (const auto& p : m_props)
            for (size_t k = 0; k < p.ids.size(); ++k)
                pts->SetPoint(p.ids[k], p.x0 + p.rest[3 * k], p.y0 + p.rest[3 * k + 1],
                              gz + 0.5 * p.height + p.rest[3 * k + 2]);
        pts->Modified();
        g->Modified();
    }
}

void World::hideProps()
{
    auto* g = grid(PidObstacles);
    vtkPoints* pts = g ? g->GetPoints() : nullptr;
    for (size_t i = 0; i < m_props.size(); ++i)
    {
        Prop& p = m_props[i];
        p.knocked = p.resting = true;   // no collision
        p.pos[0] = 1.0e4 + 5.0 * i;     // far away (LiDAR boxes too)
        p.pos[1] = 1.0e4;
        if (pts)
            for (size_t k = 0; k < p.ids.size(); ++k)
                pts->SetPoint(p.ids[k], p.pos[0], p.pos[1], -100.0);
    }
    if (pts)
    {
        pts->Modified();
        g->Modified();
    }
}

bool World::updateProps(double dt)
{
    const double gz = m_road.groundZ;
    bool moved = false;
    vtkUnstructuredGrid* g = grid(PidObstacles);
    if (!g)
        return false;
    vtkPoints* pts = g->GetPoints();
    for (auto& p : m_props)
    {
        if (!p.knocked || p.resting)
            continue;
        moved = true;
        // flight
        p.vel[2] -= 9.81 * dt;
        for (int k = 0; k < 3; ++k)
            p.pos[k] += p.vel[k] * dt;
        p.tilt += p.tiltRate * dt;
        p.spin += p.spinRate * dt;
        // lowest allowed center height for this tilt (lying on its side or standing)
        const double ct = std::abs(std::cos(p.tilt)), st = std::abs(std::sin(p.tilt));
        const double hc = 0.5 * p.height * ct + p.halfW * st;
        if (p.pos[2] < gz + hc)
        {
            // ground contact: bounce with losses, rolling friction
            p.pos[2] = gz + hc;
            if (p.vel[2] < 0.0)
                p.vel[2] = -0.25 * p.vel[2];
            p.vel[0] *= std::exp(-3.5 * dt);
            p.vel[1] *= std::exp(-3.5 * dt);
            p.tiltRate *= std::exp(-3.0 * dt);
            p.spinRate *= std::exp(-3.0 * dt);
            if (std::hypot(p.vel[0], p.vel[1]) < 0.25 && std::abs(p.vel[2]) < 0.4 && std::abs(p.tiltRate) < 1.5)
            {
                // come to rest: cones / barrels lie on their side, a crate on one of its faces
                const double q = 0.5 * kPi;
                p.tilt = std::round(p.tilt / q) * q;
                if (p.kind != ObstacleKind::Crate && std::abs(std::fmod(p.tilt, kPi)) < 1e-6)
                    p.tilt += q;   // a knocked cone does not land upright
                const double ct2 = std::abs(std::cos(p.tilt)), st2 = std::abs(std::sin(p.tilt));
                p.pos[2] = gz + 0.5 * p.height * ct2 + p.halfW * st2;
                p.resting = true;
            }
        }
        // rigid transform: tilt about the horizontal axis perpendicular to tiltDir, then spin about z
        const double ax = -std::sin(p.tiltDir), ay = std::cos(p.tiltDir);   // tilt axis (horizontal)
        const double ca = std::cos(p.tilt), sa = std::sin(p.tilt);
        const double cs = std::cos(p.spin), ss = std::sin(p.spin);
        for (size_t k = 0; k < p.ids.size(); ++k)
        {
            const double r0[3] = { p.rest[3 * k], p.rest[3 * k + 1], p.rest[3 * k + 2] };
            // Rodrigues rotation about (ax, ay, 0), angle tilt (falls toward tiltDir)
            const double dot = ax * r0[0] + ay * r0[1];
            const double cr[3] = { ay * r0[2], -ax * r0[2], ax * r0[1] - ay * r0[0] };
            double r1[3];
            for (int j = 0; j < 3; ++j)
            {
                const double a = j == 0 ? ax : (j == 1 ? ay : 0.0);
                r1[j] = r0[j] * ca + cr[j] * sa + a * dot * (1.0 - ca);
            }
            const double x = r1[0] * cs - r1[1] * ss, y = r1[0] * ss + r1[1] * cs;
            pts->SetPoint(p.ids[k], p.pos[0] + x, p.pos[1] + y, p.pos[2] + r1[2]);
        }
    }
    if (moved)
    {
        pts->Modified();
        g->Modified();
    }
    return moved;
}

const char* World::sectionNameKo(double s) const
{
    s = m_road.wrap(s);
    switch (m_road.zoneAt(s))
    {
    case RoadModel::Zone::City:
        return s < 40.0 ? u8"출발 (도심)" : u8"도심 (교차로·교통)";
    case RoadModel::Zone::Corner:  return u8"코너 (감속 구간)";
    case RoadModel::Zone::Forest:
        return inObstacleCourse(s) ? u8"장애물 코스 (숲길)" : u8"숲길";
    case RoadModel::Zone::Highway:
        return inTunnel(s) ? u8"고속도로 터널" : u8"고속도로 (가드레일·중앙분리대)";
    case RoadModel::Zone::Suburb:  return u8"교외 주택가";
    }
    return "";
}

double World::placeS(int place) const
{
    switch (place)
    {
    case 1: return m_road.zoneFrom(RoadModel::Zone::Forest) + 5.0;
    case 2: return m_road.zoneFrom(RoadModel::Zone::Highway) + 20.0;
    case 3: return m_road.zoneFrom(RoadModel::Zone::Suburb) + 10.0;
    }
    return 17.0;
}

bool World::inObstacleCourse(double s) const
{
    s = m_road.wrap(s);
    return s > m_courseFrom && s < m_courseTo;
}

bool World::trafficAllowed(double s, int dir) const
{
    // the obstacle course blocks the ego-direction lanes
    s = m_road.wrap(s);
    return dir < 0 || s < m_courseFrom - 10.0 || s > m_courseTo + 5.0;
}

bool World::nearIntersection(double s, double margin) const
{
    for (double si : m_intersections)
        if (std::abs(m_road.diff(s, si)) < margin)
            return true;
    return false;
}

bool World::guardrailAt(double s, int side) const
{
    const auto z = m_road.zoneAt(s);
    if (z == RoadModel::Zone::Highway)
        return true;
    return z == RoadModel::Zone::Corner && side < 0;   // outer side of the (right-hand) corners
}

bool World::medianBarrierAt(double s) const { return m_road.zoneAt(s) == RoadModel::Zone::Highway; }

// ---------------------------------------------------------------- traffic lights
// Every junction has its own cycle length and offset, so the lights along the street change at
// different moments. One cycle (main road = the loop, cross = side street):
//   main green -> main yellow (3 s) -> main red + protected LEFT ARROW (6 s) -> all red (1 s)
//   -> cross green -> cross yellow (3 s) -> all red (1 s)
namespace
{
const double kCycle[]  = { 34.0, 42.0, 38.0, 46.0, 36.0, 40.0 };
const double kOffset[] = { 4.0, 17.0, 29.5, 8.5, 22.0, 12.5 };
constexpr double kYellowT = 3.0, kArrowT = 6.0, kAllRedT = 1.0;

struct Timeline
{
    double C, G, Y, A, crossStart, crossYellow, crossEnd;
};
Timeline timeline(int j)
{
    Timeline tl;
    tl.C = kCycle[j % 6];
    tl.G = 0.45 * tl.C;            // main green ends
    tl.Y = tl.G + kYellowT;        // main yellow ends
    tl.A = tl.Y + kArrowT;         // left arrow ends
    tl.crossStart = tl.A + kAllRedT;
    tl.crossEnd = tl.C - kAllRedT;
    tl.crossYellow = tl.crossEnd - kYellowT;
    return tl;
}
double phaseOf(int j, double t, double C)
{
    double tau = std::fmod(t + kOffset[j % 6], C);
    return tau < 0.0 ? tau + C : tau;
}
}

double World::signalCycle(int junction) const { return kCycle[junction % 6]; }

Light World::signalAt(int junction, int dir, double t) const
{
    const Timeline tl = timeline(junction);
    const double tau = phaseOf(junction, t, tl.C);
    if (dir != 0)
    {
        if (tau < tl.G) return Light::Green;
        if (tau < tl.Y) return Light::Yellow;
        if (tau < tl.A) return Light::RedArrow;
        return Light::Red;
    }
    if (tau >= tl.crossStart && tau < tl.crossYellow) return Light::Green;
    if (tau >= tl.crossYellow && tau < tl.crossEnd) return Light::Yellow;
    return Light::Red;
}

double World::signalRemaining(int junction, int dir, double t) const
{
    const Timeline tl = timeline(junction);
    const double tau = phaseOf(junction, t, tl.C);
    const std::vector<double> marks = dir != 0
        ? std::vector<double>{ tl.G, tl.Y, tl.A, tl.C }
        : std::vector<double>{ tl.crossStart, tl.crossYellow, tl.crossEnd, tl.C + tl.crossStart };
    for (double m : marks)
        if (m > tau + 1e-9)
            return m - tau;
    return tl.C - tau;
}

double World::stopLineS(int junction, int dir) const
{
    return m_road.wrap(m_intersections[junction] - dir * kStopLine);
}

int World::nextStopLine(double s, int dir, double maxAhead, double& dist, double minAhead) const
{
    int best = -1;
    dist = 1e9;
    for (size_t j = 0; j < m_intersections.size(); ++j)
    {
        const double d = dir * m_road.diff(stopLineS(static_cast<int>(j), dir), s);
        if (d >= minAhead && d <= maxAhead && d < dist)
        {
            dist = d;
            best = static_cast<int>(j);
        }
    }
    return best;
}

bool World::inTunnel(double s) const
{
    s = m_road.wrap(s);
    return s > m_tunnelFrom && s < m_tunnelTo;
}

double World::tunnelCeiling(double lat, double halfWidth)
{
    // two bores (one per direction) separated by the center wall; vertical walls up to 4.8 m and a
    // vault rising to 6.7 m
    const double a = std::abs(lat);
    const double wallIn = halfWidth + 1.0;
    if (a <= 0.25 || a >= wallIn)
        return 4.8;
    const double t = (a - 0.25) / (wallIn - 0.25);
    return 4.8 + 1.9 * std::sin(kPi * t);
}

void World::setVehicleType(int type)
{
    m_vehicleType = type;
    VehicleShapes::Parts parts = VehicleShapes::buildVehicle(type);
    m_wheelHubs.clear();
    for (double x : parts.axles)
    {
        m_wheelHubs.push_back({ x, -0.02, parts.wheelZ });
        m_wheelHubs.push_back({ x, 1.02, parts.wheelZ });
    }
    m_wheelR = parts.wheelR;
    const std::pair<int, Grid> pieces[] = { { PidCab, parts.cab }, { PidCargo, parts.cargo },
                                            { PidTires, parts.wheels }, { PidCamera, parts.sensors } };
    for (const auto& [pid, g] : pieces)
    {
        auto it = m_pidGrids.find(pid);
        if (it == m_pidGrids.end() || !it->second)
        {
            m_pidGrids[pid] = g;
        }
        else
        {
            it->second->DeepCopy(g);   // same object: every actor / mapper keeps working
            it->second->Modified();
        }
    }
}

vtkUnstructuredGrid* World::grid(int pid) const
{
    auto it = m_pidGrids.find(pid);
    return it == m_pidGrids.end() ? nullptr : it->second.GetPointer();
}

double World::terrainHeight(double x, double y) const
{
    const double h = naturalHeight(x, y);
    if (!m_netReady)
        return h;
    // the strongest influence wins: junction pads, streets (cut / fill), underpass floor
    double best = 0.0, out = h;
    for (const auto& nd : m_nodes)
    {
        const double d = std::hypot(x - nd.x, y - nd.y);
        if (d > nd.radius + 90.0)
            continue;
        // slope at most ~1:3 between the junction and the natural ground
        const double span = 20.0 + 3.0 * std::abs(nd.z - h);
        if (d > nd.radius + span)
            continue;
        const double w = smoothstep(nd.radius + span, nd.radius + 2.0, d);
        if (w > best)
        {
            best = w;
            out = h + (nd.z - 0.35 - h) * w;
        }
    }
    for (size_t r = 0; r < m_net.size(); ++r)
    {
        const NetRoad& nr = m_net[r];
        double s, lat;
        if (!nr.road.project(x, y, s, lat))
            continue;
        const double a = std::abs(lat);
        const double hw = nr.road.halfWidth;
        if (a > hw + 90.0)
            continue;
        const int st = netStructure(static_cast<int>(r), s);
        const double zr = nr.road.elevation(s);
        if (st == NetViaduct)
            continue;                                         // open under the deck
        if (st == NetTrench || st == NetCovered)
        {
            if (a < hw + 1.5)
                return zr;                                    // floor between the retaining walls
            continue;
        }
        // the coarse terrain grid must not poke through the road in sag curves: 0.35 m lower under the lanes
        // embankment / cut slope at most ~1:3 (the blend widens with the height difference)
        const double span = 22.0 + 3.0 * std::abs(zr - h);
        if (a > hw + 3.0 + span)
            continue;
        const double w = smoothstep(hw + 3.0 + span, hw + 3.0, a);
        if (w > best)
        {
            best = w;
            const double sink = 0.35 * smoothstep(hw + 3.0, hw, a);
            out = h + (zr - sink - h) * w;
        }
    }
    return out;
}

double World::netCoverTop(int road, double s) const
{
    // the cover slab lies flush with the ground beside the tunnel (never below the street level 0)
    const NetRoad& nr = m_net[road];
    const double ext = netWallLat(road) + 3.0;
    double top = 0.0;
    for (double side : { -1.0, 1.0 })
    {
        double x, y;
        nr.road.point(s, side * ext, x, y);
        top = std::max(top, terrainHeight(x, y));
    }
    return std::min(top, 3.0);
}

double World::naturalHeight(double x, double y) const
{
    double s = 0.0, lat = 0.0;
    double d = 400.0;
    if (m_road.project(x, y, s, lat))
    {
        d = std::abs(lat);
        if (m_road.zoneAt(s) == RoadModel::Zone::City)
        {
            if (d < 150.0)
                return 0.0;   // flat downtown (buildings, cross streets, back streets, connector)
            d -= 125.0;       // hills start gently behind the last row of buildings
        }
    }
    const double n = fbm(x * 0.004 + 11.0, y * 0.004 - 7.0);
    if (m_road.insideLoop(x, y))
    {
        // park with gentle hills and a lake
        const double h = smoothstep(35.0, 110.0, d) * (1.0 + 9.0 * n);
        const double t = smoothstep(0.75, 1.6, lakeE(x, y));
        return -2.2 * (1.0 - t) + h * t;
    }
    // mountains around the loop
    const double m = fbm(x * 0.0022 - 3.0, y * 0.0022 + 5.0);
    return smoothstep(40.0, 230.0, d) * (8.0 + 80.0 * m * m) + smoothstep(25.0, 60.0, d) * 2.0 * n;
}

void World::build()
{
    m_pidGrids.clear();
    m_circles.clear();
    m_boxes.clear();
    m_npcSpawns.clear();
    m_npcGrids.clear();
    m_paved.clear();
    m_intersections.assign(std::begin(kIntersections), std::end(kIntersections));

    const RoadModel& r = m_road;
    const double gz = r.groundZ;
    const double hw = r.halfWidth;
    Rng rng;

    const double forestFrom = r.zoneFrom(RoadModel::Zone::Forest), forestTo = r.zoneTo(RoadModel::Zone::Forest);
    const double hwyFrom = r.zoneFrom(RoadModel::Zone::Highway), hwyTo = r.zoneTo(RoadModel::Zone::Highway);
    const double subFrom = r.zoneFrom(RoadModel::Zone::Suburb), subTo = r.zoneTo(RoadModel::Zone::Suburb);
    const double cityTo = r.zoneTo(RoadModel::Zone::City);
    m_courseFrom = forestFrom + kGate0 - 25.0;
    m_courseTo = forestFrom + kGate0 + 195.0;
    // highway tunnel through a hill: 170 m, inside the long S-bend (west of the ring road viaduct)
    m_tunnelFrom = hwyFrom + 90.0;
    m_tunnelTo = hwyFrom + 260.0;
    auto nearTunnel = [&](double s, double margin) {
        return m_road.diff(s, m_tunnelFrom) > -margin && m_road.diff(s, m_tunnelTo) < margin;
    };
    // under the hill over the tunnel (no trees growing through it)
    auto underHill = [&](double x, double y) {
        double ps, pl;
        return r.project(x, y, ps, pl) && nearTunnel(ps, 40.0) && pl > -(hw + 66.0) && pl < hw + 42.0;
    };

    auto P = [&](double s, double lat, double& x, double& y) { r.point(s, lat, x, y); };
    auto yawDeg = [&](double s) { return r.heading(s) * kRad2Deg; };
    auto constMat = [](int m) { return [m](int, int, int, double, double, double) { return m; }; };

    // street network: elevation profiles first (terrainHeight() cuts / fills along them), then the meshes
    buildNetwork();
    NetParts ring = buildNetGeometry();

    // ---------------------------------------------------------------
    // Vehicle (local frame): deformable hex meshes shaped like a truck
    // ---------------------------------------------------------------
    setVehicleType(m_vehicleType);

    // ---------------------------------------------------------------
    // Roads: divided 6-lane loop + 4-lane cross streets in the city
    // ---------------------------------------------------------------
    std::vector<Grid> roads = { SceneBuilder::roadStrip(r, 2.0, 12, Palette::Asphalt) };
    std::vector<Grid> marks = { SceneBuilder::laneMarkings(r, Palette::LaneWhite, Palette::LaneYellow,
                                                           m_intersections, kMarkGap) };
    for (double si : kIntersections)
    {
        // cross street slightly below the main road (no z-fighting where they overlap)
        roads.push_back(SceneBuilder::ribbon(r, si - kCrossHalf, si + kCrossHalf, -kCrossLength, kCrossLength,
                                             gz - 0.004, 2.0, Palette::Asphalt, 20));
        // cross street markings outside the junction: yellow double center line, dashed dividers
        for (double side : { -1.0, 1.0 })
        {
            const double l0 = side * kWalkIn, l1 = side * (kBackLat - kCrossHalf);
            for (double ds : { -0.13, 0.13 })
                marks.push_back(SceneBuilder::ribbon(r, si + ds - 0.07, si + ds + 0.07, std::min(l0, l1),
                                                     std::max(l0, l1), gz + 0.004, 1.0, Palette::LaneYellow, 8));
            for (double d = kWalkIn; d + 3.0 < kBackLat - kCrossHalf; d += 7.0)
                for (double ds : { -3.0, 3.0 })
                {
                    const double a = side * d, b = side * (d + 3.0);
                    marks.push_back(SceneBuilder::ribbon(r, si + ds - 0.07, si + ds + 0.07, std::min(a, b),
                                                         std::max(a, b), gz + 0.004, 1.0, Palette::LaneWhite));
                }
        }
        // zebra crossings on both sides of the junction
        for (double side : { 1.0, -1.0 })
        {
            const double sc = si + side * kCrosswalk;
            for (int k = 0; k < 20; ++k)
            {
                const double lat = -8.55 + 0.9 * k;
                marks.push_back(SceneBuilder::ribbon(r, sc - 1.5, sc + 1.5, lat - 0.25, lat + 0.25, gz + 0.006,
                                                     3.0, Palette::LaneWhite));
            }
        }
        // stop lines: player direction before the junction (right half), oncoming after it (left half)
        marks.push_back(SceneBuilder::ribbon(r, si - kStopLine - 0.25, si - kStopLine + 0.25, 0.3, hw, gz + 0.006,
                                             1.0, Palette::LaneWhite));
        marks.push_back(SceneBuilder::ribbon(r, si + kStopLine - 0.25, si + kStopLine + 0.25, -hw, -0.3, gz + 0.006,
                                             1.0, Palette::LaneWhite));
    }
    // back streets parallel to the main road on both sides of downtown: every cross street ends in
    // one (T-junction), so a left or right turn in the city leads around the block, not into a dead end
    {
        const double b0 = kIntersections[0] - kCrossHalf;
        const double b1 = kIntersections[std::size(kIntersections) - 1] + kCrossHalf;
        for (double side : { -1.0, 1.0 })
        {
            const double lc = side * kBackLat;
            roads.push_back(SceneBuilder::ribbon(r, b0, b1, lc - kCrossHalf, lc + kCrossHalf, gz - 0.006, 2.0,
                                                 Palette::Asphalt, 6));
            m_paved.push_back({ b0, b1, lc - kCrossHalf, lc + kCrossHalf, int(Palette::Asphalt) });
            for (size_t k = 0; k + 1 < std::size(kIntersections); ++k)
            {
                const double a = kIntersections[k] + kCrossHalf + 1.0, b = kIntersections[k + 1] - kCrossHalf - 1.0;
                for (double dl : { -0.13, 0.13 })
                    marks.push_back(SceneBuilder::ribbon(r, a, b, lc + dl - 0.07, lc + dl + 0.07, gz + 0.004, 2.0,
                                                         Palette::LaneYellow));
                for (double s = a; s + 3.0 < b; s += 7.0)
                    for (double dl : { -3.0, 3.0 })
                        marks.push_back(SceneBuilder::ribbon(r, s, s + 3.0, lc + dl - 0.07, lc + dl + 0.07, gz + 0.004,
                                                             1.0, Palette::LaneWhite));
            }
            // stop lines on the cross streets before they meet the back street (right-hand lanes)
            for (double si : kIntersections)
            {
                const double l = side * (kBackLat - kCrossHalf - 1.0);
                const double sa = side > 0 ? si - kCrossHalf + 0.3 : si + 0.3;
                marks.push_back(SceneBuilder::ribbon(r, sa, sa + kCrossHalf - 0.6, std::min(l - 0.25, l + 0.25),
                                                     std::max(l - 0.25, l + 0.25), gz + 0.006, 1.0,
                                                     Palette::LaneWhite));
            }
        }
    }
    roads.insert(roads.end(), ring.roads.begin(), ring.roads.end());
    marks.insert(marks.end(), ring.marks.begin(), ring.marks.end());
    m_pidGrids[PidRoad] = SceneBuilder::merge(roads);
    m_pidGrids[PidMarkings] = SceneBuilder::merge(marks);

    // ---------------------------------------------------------------
    // Obstacle course (forest)
    // ---------------------------------------------------------------
    std::vector<Grid> obstacleGrids;
    PolyBatch barrelBatch, coneBatch;
    m_props.clear();
    for (const auto& o : kCourse)
    {
        const double s = forestFrom + kGate0 + o.ds;
        double x, y;
        P(s, r.laneOffset(o.lane) + o.dl, x, y);
        Prop pr;
        pr.x0 = x;
        pr.y0 = y;
        pr.yaw0 = r.heading(s);
        const int pi = static_cast<int>(m_props.size());
        switch (o.type)
        {
        case ObstacleType::Barrel:
            barrelBatch.cylinder(x, y, gz, kBarrelRadius, kBarrelHeight, 16);
            m_circles.push_back({ x, y, kBarrelRadius, ObstacleKind::Barrel, pi });
            pr.kind = ObstacleKind::Barrel;
            pr.halfL = pr.halfW = kBarrelRadius;
            pr.height = kBarrelHeight;
            pr.material = Palette::Barrel;
            break;
        case ObstacleType::Cone:
            coneBatch.cone(x, y, gz, kConeRadius, kConeHeight, 16);
            m_circles.push_back({ x, y, kConeRadius, ObstacleKind::Cone, pi });
            pr.kind = ObstacleKind::Cone;
            pr.halfL = pr.halfW = kConeRadius;
            pr.height = kConeHeight;
            pr.material = Palette::Barrel;
            break;
        case ObstacleType::Box:
            obstacleGrids.push_back(SceneBuilder::orientedBlock(x, y, yawDeg(s), kBoxSize, kBoxSize, gz,
                                                                gz + kBoxSize, 3, 3, 3, constMat(Palette::Crate)));
            m_boxes.push_back({ x, y, 0.5 * kBoxSize, 0.5 * kBoxSize, r.heading(s), ObstacleKind::Crate, pi });
            pr.kind = ObstacleKind::Crate;
            pr.halfL = pr.halfW = 0.5 * kBoxSize;
            pr.height = kBoxSize;
            pr.material = Palette::Crate;
            break;
        }
        pr.pos[0] = x;
        pr.pos[1] = y;
        pr.pos[2] = gz + 0.5 * pr.height;
        m_props.push_back(pr);
    }
    obstacleGrids.push_back(barrelBatch.grid(Palette::Barrel));
    obstacleGrids.push_back(coneBatch.grid(Palette::Barrel));

    // LiDAR proxy geometry: like a physics collider, detailed render meshes (sedans, window-gridded
    // buildings) are replaced by simple boxes for ray casting. The loose course objects are NOT in it:
    // they can be knocked away, so the LiDAR sees them as moving boxes (DriveSim::dynamicBoxes).
    std::vector<Grid> lidarProxy;

    // parked cars along the cross streets (city)
    {
        const int paints[] = { Palette::NpcWhite, Palette::NpcBlue, Palette::Rim, Palette::NpcGreen, Palette::Body };
        int pi = 0;
        for (double si : kIntersections)
            for (double lat : { -64.0, -55.0, -46.0, -37.0, -24.0, 22.0, 31.0, 44.0, 53.0, 62.0 })
            {
                if (rng.next() < 0.35)
                    continue;  // leave some gaps
                const double s = si + (lat > 0 ? -4.6 : 4.6);   // right-hand curb of the cross street
                double px, py;
                P(s, lat, px, py);
                const int paint = paints[pi++ % 5];
                // long axis along the cross street
                const double yaw = r.heading(s) + (lat > 0 ? -0.5 * kPi : 0.5 * kPi);
                lidarProxy.push_back(SceneBuilder::orientedBlock(px, py, yaw * kRad2Deg, 4.0, 1.26, gz, gz + 1.25,
                                                                 1, 1, 1, constMat(paint)));
                // sedan mesh: local center (2, 0.5), forward = -x -> rotation angle = yaw - pi
                Grid car = VehicleShapes::buildSedan(paint);
                const double a = yaw - kPi;
                const double ox = px - (std::cos(a) * 2.0 - std::sin(a) * 0.5);
                const double oy = py - (std::sin(a) * 2.0 + std::cos(a) * 0.5);
                obstacleGrids.push_back(SceneBuilder::placed(car, ox, oy, a * kRad2Deg));
                m_boxes.push_back({ px, py, 2.05, 0.65, yaw, ObstacleKind::ParkedCar });
            }
    }
    m_pidGrids[PidObstacles] = SceneBuilder::merge(obstacleGrids);
    // points of every loose course object in the merged grid (they stand apart from everything else)
    {
        vtkPoints* pts = m_pidGrids[PidObstacles]->GetPoints();
        for (auto& pr : m_props)
        {
            pr.ids.clear();
            pr.rest.clear();
            const double reach = std::max(pr.halfL, pr.halfW) * 1.5 + 0.05;
            for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i)
            {
                double q[3];
                pts->GetPoint(i, q);
                if (std::abs(q[0] - pr.x0) > reach || std::abs(q[1] - pr.y0) > reach || q[2] > gz + pr.height + 0.2)
                    continue;
                pr.ids.push_back(i);
                pr.rest.push_back(q[0] - pr.x0);
                pr.rest.push_back(q[1] - pr.y0);
                pr.rest.push_back(q[2] - (gz + 0.5 * pr.height));
            }
        }
    }

    // ---------------------------------------------------------------
    // Guard rails (highway both sides, outer side of the corners) + median barrier (highway)
    // ---------------------------------------------------------------
    {
        std::vector<Grid> walls;
        for (const auto& pc : r.pieces())
        {
            for (int side : { -1, 1 })
            {
                if (!guardrailAt(pc.s0 + 0.5 * pc.len, side))
                    continue;
                const double lat = side * (hw + 1.0);
                // rail pieces outside the tunnel (the tunnel walls take over inside)
                std::vector<std::pair<double, double>> spans = { { pc.s0, pc.s0 + pc.len } };
                if (pc.zone == RoadModel::Zone::Highway)
                {
                    spans.clear();
                    const double a = pc.s0, b = pc.s0 + pc.len;
                    if (a < m_tunnelFrom) spans.push_back({ a, std::min(b, m_tunnelFrom) });
                    if (b > m_tunnelTo) spans.push_back({ std::max(a, m_tunnelTo), b });
                }
                for (const auto& [a, b] : spans)
                    if (b - a > 0.5)
                        walls.push_back(SceneBuilder::wallAlong(r, a, b, lat, 0.08, gz + 0.45, gz + 0.8, 3.0,
                                                                Palette::Metal));
                for (double s = pc.s0 + 1.0; s < pc.s0 + pc.len; s += 4.0)
                {
                    if (inTunnel(s))
                        continue;
                    double x, y;
                    P(s, lat + side * 0.08, x, y);
                    walls.push_back(SceneBuilder::orientedBlock(x, y, yawDeg(s), 0.12, 0.12, gz, gz + 0.78, 1, 1, 1,
                                                                constMat(Palette::Metal)));
                }
            }
        }
        walls.push_back(SceneBuilder::wallAlong(r, hwyFrom, hwyTo, 0.0, 0.5, gz, gz + 0.85, 4.0, Palette::Jersey));
        walls.insert(walls.end(), ring.walls.begin(), ring.walls.end());
        m_pidGrids[PidWall] = SceneBuilder::merge(walls);
    }

    // ---------------------------------------------------------------
    // Countryside: apartment complexes, a second row of houses, a gas station, farm fields with
    // vinyl greenhouses, wind turbines and power lines on the hills, billboards, boats on the lake.
    // Placed before the trees: every structure reserves its ground, trees keep out of it.
    // ---------------------------------------------------------------
    std::vector<Grid> extraBuildings, extraProps, extraVeg;
    PolyBatch extraWhite, extraPoles;
    for (const auto& pl : ring.poles)
        extraPoles.cylinder(pl[0], pl[1], pl[2], 0.09, pl[3], 10);
    for (const auto& g : ring.props)
    {
        extraProps.push_back(g);
        lidarProxy.push_back(g);
    }
    struct Reserved { double cx, cy, c, s, hx, hy; };
    std::vector<Reserved> reserved;
    auto reserve = [&](double cx, double cy, double yawRad, double hx, double hy) {
        reserved.push_back({ cx, cy, std::cos(yawRad), std::sin(yawRad), hx, hy });
    };
    auto isReserved = [&](double x, double y) {
        for (const auto& q : reserved)
        {
            const double dx = x - q.cx, dy = y - q.cy;
            if (std::abs(q.c * dx + q.s * dy) < q.hx && std::abs(-q.s * dx + q.c * dy) < q.hy)
                return true;
        }
        return false;
    };
    // corner / center sample points of a rectangle (a: along yaw, b: across)
    auto rectPoint = [](double cx, double cy, double yawRad, double a, double b, double& x, double& y) {
        x = cx + a * std::cos(yawRad) - b * std::sin(yawRad);
        y = cy + a * std::sin(yawRad) + b * std::cos(yawRad);
    };
    auto groundUnder = [&](double cx, double cy, double yawRad, double L, double W, double& lo, double& hi) {
        lo = hi = terrainHeight(cx, cy);
        for (int a : { -1, 1 })
            for (int b : { -1, 1 })
            {
                double x, y;
                rectPoint(cx, cy, yawRad, 0.5 * a * L, 0.5 * b * W, x, y);
                const double h = terrainHeight(x, y);
                lo = std::min(lo, h);
                hi = std::max(hi, h);
            }
    };
    // far enough from every road (and downtown), not in the lake, not on the tunnel hill, not taken
    auto freeArea = [&](double cx, double cy, double yawRad, double L, double W, double minLat) {
        for (int a : { -1, 0, 1 })
            for (int b : { -1, 0, 1 })
            {
                double x, y, ps, pl;
                rectPoint(cx, cy, yawRad, 0.5 * a * L, 0.5 * b * W, x, y);
                if (r.project(x, y, ps, pl) &&
                    (std::abs(pl) < minLat || (r.zoneAt(ps) == RoadModel::Zone::City && std::abs(pl) < 135.0)))
                    return false;
                if (lakeE(x, y) < 1.7 || underHill(x, y) || isReserved(x, y) || nearNet(x, y, 14.0))
                    return false;
            }
        return true;
    };
    // thin beam between two points, optionally sagging (pylon legs, wires, masts)
    auto beam = [&](std::array<double, 3> A, std::array<double, 3> B, double t, int mat, int nseg, double sag) {
        double d[3] = { B[0] - A[0], B[1] - A[1], B[2] - A[2] };
        const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        for (double& k : d) k /= len;
        double e1[3] = { -d[1], d[0], 0.0 };
        double l1 = std::hypot(e1[0], e1[1]);
        if (l1 < 1e-6) { e1[0] = 1.0; e1[1] = 0.0; l1 = 1.0; }
        e1[0] /= l1;
        e1[1] /= l1;
        const double e2[3] = { d[1] * e1[2] - d[2] * e1[1], d[2] * e1[0] - d[0] * e1[2], d[0] * e1[1] - d[1] * e1[0] };
        const std::array<double, 3> D = { d[0], d[1], d[2] }, E1 = { e1[0], e1[1], e1[2] }, E2 = { e2[0], e2[1], e2[2] };
        return SceneBuilder::morphedHex(nseg, 1, 1,
            [=](double u, double v, double w, double p[3]) {
                for (int k = 0; k < 3; ++k)
                    p[k] = A[k] + u * len * D[k] + (v - 0.5) * t * E1[k] + (w - 0.5) * t * E2[k];
                p[2] -= sag * 4.0 * u * (1.0 - u);
            },
            constMat(mat));
    };
    // house with a gable roof (suburb)
    const int roofColors[] = { Palette::Roof, Palette::RoofGray, Palette::Roof };
    auto house = [&](double cx, double cy, double yawDeg_, double L, double W, double wallH, double gable, int wm,
                     int roof, std::vector<Grid>& out) {
        const double c = std::cos(yawDeg_ / kRad2Deg), sn = std::sin(yawDeg_ / kRad2Deg);
        out.push_back(SceneBuilder::morphedHex(4, 2, 2,
            [=](double u, double v, double w, double o[3]) {
                const double lx = (u - 0.5) * L, ly = (v - 0.5) * W;
                o[0] = cx + c * lx - sn * ly;
                o[1] = cy + sn * lx + c * ly;
                o[2] = gz + w * (wallH + gable * (1.0 - std::abs(2.0 * v - 1.0)));
            },
            [wm](int i, int, int k, double, double, double) {
                return (k == 0 && (i == 1 || i == 2)) ? int(Palette::Window) : wm;
            }));
        out.push_back(SceneBuilder::gableRoof(cx, cy, yawDeg_, L, W, gz + wallH, gable, roof));
        m_boxes.push_back({ cx, cy, 0.5 * L, 0.5 * W, yawDeg_ / kRad2Deg, ObstacleKind::Building });
        lidarProxy.push_back(SceneBuilder::orientedBlock(cx, cy, yawDeg_, L, W, gz, gz + wallH + 0.5 * gable, 1, 1, 1,
                                                         constMat(wm)));
    };

    // ---- gas station on the suburb road (inside of the loop, near the end of the suburb)
    const double gasS0 = subFrom + 40.0, gasS1 = subFrom + 96.0;   // north end (the ring passes the south end)
    {
        const double sc = 0.5 * (gasS0 + gasS1);
        const double yaw = yawDeg(sc);
        // forecourt (concrete), from the sidewalk to the back of the lot
        extraProps.push_back(SceneBuilder::ribbon(r, gasS0, gasS1, kSuburbWalkOut, 38.0, gz + 0.02, 2.0,
                                                  Palette::Concrete, 4));
        m_paved.push_back({ gasS0, gasS1, kSuburbWalkOut, 38.0, int(Palette::Concrete) });
        {
            double x0, y0, x1, y1;
            P(gasS0, 25.0, x0, y0);
            P(gasS1, 25.0, x1, y1);
            reserve(0.5 * (x0 + x1), 0.5 * (y0 + y1), r.heading(sc), 0.5 * (gasS1 - gasS0) + 3.0, 16.0);
        }
        // canopy on four posts
        const double cs = sc - 6.0, cl = 23.0;
        for (double ds : { -9.0, 9.0 })
            for (double dl : { -4.0, 4.0 })
            {
                double x, y;
                P(cs + ds, cl + dl, x, y);
                extraPoles.cylinder(x, y, gz, 0.22, 5.2, 10);
                m_circles.push_back({ x, y, 0.25, ObstacleKind::StreetLight });
            }
        {
            double x, y;
            P(cs, cl, x, y);
            extraBuildings.push_back(SceneBuilder::orientedBlock(x, y, yaw, 24.0, 13.0, gz + 5.2, gz + 6.1, 6, 3, 2,
                [](int, int, int k, double, double, double) {
                    return k == 0 ? int(Palette::LaneWhite) : int(Palette::Body);
                }));
        }
        // pump islands
        for (double ds : { -6.0, 0.0, 6.0 })
        {
            double x, y;
            P(cs + ds, cl, x, y);
            extraBuildings.push_back(SceneBuilder::orientedBlock(x, y, yaw, 3.4, 1.1, gz, gz + 0.22, 1, 1, 1,
                                                                 constMat(Palette::Concrete)));
            extraBuildings.push_back(SceneBuilder::orientedBlock(x, y, yaw, 0.9, 0.6, gz + 0.22, gz + 1.9, 1, 1, 2,
                [](int, int, int k, double, double, double) {
                    return k == 1 ? int(Palette::Body) : int(Palette::NpcWhite);
                }));
            m_boxes.push_back({ x, y, 1.7, 0.55, r.heading(cs + ds), ObstacleKind::Crate });
            lidarProxy.push_back(SceneBuilder::orientedBlock(x, y, yaw, 1.0, 0.7, gz, gz + 1.9, 1, 1, 1,
                                                             constMat(Palette::NpcWhite)));
        }
        // shop
        {
            double x, y;
            P(sc + 16.0, 31.0, x, y);
            extraBuildings.push_back(SceneBuilder::orientedBlock(x, y, yaw, 13.0, 9.0, gz, gz + 4.3, 5, 3, 2,
                [](int i, int j, int k, double, double, double) {
                    if (k == 1 && (j == 2) && i > 0 && i < 4) return int(Palette::Window);
                    return k == 1 && j == 2 ? int(Palette::Body) : int(Palette::Stucco);
                }));
            m_boxes.push_back({ x, y, 6.5, 4.5, r.heading(sc + 16.0), ObstacleKind::Building });
            lidarProxy.push_back(SceneBuilder::orientedBlock(x, y, yaw, 13.0, 9.0, gz, gz + 4.3, 1, 1, 1,
                                                             constMat(Palette::Stucco)));
        }
        // price sign on a tall pole at the entrance
        {
            double x, y;
            P(gasS0 + 6.0, 13.2, x, y);
            extraPoles.cylinder(x, y, gz, 0.15, 7.2, 10);
            m_circles.push_back({ x, y, 0.2, ObstacleKind::Sign });
            extraBuildings.push_back(SceneBuilder::orientedBlock(x, y, yaw, 0.3, 2.4, gz + 4.2, gz + 7.0, 1, 1, 4,
                [](int, int, int k, double, double, double) {
                    return k == 3 ? int(Palette::Body) : (k == 0 ? int(Palette::Trim) : int(Palette::LaneWhite));
                }));
        }
    }

    // ---- second row of suburb houses
    for (double side : { -1.0, 1.0 })
        for (double s = subFrom + 18.0; s < subTo - 14.0; s += rng.range(20.0, 26.0))
        {
            if (side > 0 && s > gasS0 - 8.0 && s < gasS1 + 8.0)
                continue;
            const double L = rng.range(9.0, 12.0), W = rng.range(7.0, 9.0);
            const double latC = side * rng.range(39.0, 44.0);
            double cx, cy;
            P(s, latC, cx, cy);
            const double yawR = r.heading(s);
            if (!freeArea(cx, cy, yawR, L + 2.0, W + 2.0, 32.0))
                continue;
            const int wm = rng.next() < 0.33 ? int(Palette::Brick) : int(Palette::Stucco);
            house(cx, cy, yawR * kRad2Deg, L, W, rng.range(5.0, 6.2), rng.range(2.2, 3.0), wm,
                  roofColors[static_cast<int>(rng.next() * 3) % 3], extraBuildings);
            reserve(cx, cy, yawR, 0.5 * L + 3.0, 0.5 * W + 3.0);
        }

    // ---- apartment complexes (Korean "apateu danji") outside the suburb
    {
        const int walls[] = { Palette::NpcWhite, Palette::Stucco, Palette::Concrete };
        int ai = 0;
        for (double s = subFrom + 25.0; s < subTo - 20.0; s += 60.0)
            for (double lat : { -84.0, -124.0 })
            {
                const double L = 44.0, W = 13.0;
                double cx, cy;
                P(s, lat, cx, cy);
                const double yawR = r.heading(s);
                if (!freeArea(cx, cy, yawR, L + 8.0, W + 8.0, 30.0))
                    continue;
                double lo, hi;
                groundUnder(cx, cy, yawR, L, W, lo, hi);
                if (hi - lo > 10.0)
                    continue;   // too steep
                const int floors = 12 + static_cast<int>(rng.next() * 11);
                const double z0 = gz + lo - 0.5, z1 = gz + hi + 2.9 * floors;
                const int nx = 16, ny = 4, nz = floors;
                const int wall = walls[ai++ % 3];
                extraBuildings.push_back(SceneBuilder::orientedBlock(cx, cy, yawR * kRad2Deg, L, W, z0, z1, nx, ny, nz,
                    [=](int i, int j, int k, double, double, double) {
                        if (k == 0 || k == nz - 1) return wall;
                        if ((j == 0 || j == ny - 1) && i % 3 != 0) return int(Palette::Window);   // balconies
                        if ((i == 0 || i == nx - 1) && j == 1) return int(Palette::Window);       // stair windows
                        return wall;
                    }));
                // elevator housing on the roof + a colored band (building number side)
                extraBuildings.push_back(SceneBuilder::orientedBlock(cx, cy, yawR * kRad2Deg, 8.0, 5.0, z1, z1 + 3.2,
                                                                     1, 1, 1, constMat(wall)));
                extraBuildings.push_back(SceneBuilder::orientedBlock(cx, cy, yawR * kRad2Deg, L + 0.3, W + 0.3,
                                                                     z1 - 1.2, z1 - 0.4, 8, 2, 1,
                                                                     constMat(ai % 2 ? int(Palette::NpcBlue)
                                                                                     : int(Palette::NpcGreen))));
                m_boxes.push_back({ cx, cy, 0.5 * L, 0.5 * W, yawR, ObstacleKind::Building });
                lidarProxy.push_back(SceneBuilder::orientedBlock(cx, cy, yawR * kRad2Deg, L, W, z0, z1, 1, 1, 1,
                                                                 constMat(wall)));
                reserve(cx, cy, yawR, 0.5 * L + 6.0, 0.5 * W + 6.0);
            }
    }

    // ---- billboards on the outside of every corner
    for (const auto& pc : r.pieces())
    {
        if (pc.zone != RoadModel::Zone::Corner)
            continue;
        static int bb = 0;
        const double s = pc.s0 + 0.5 * pc.len;
        const double lat = -(hw + 17.0);
        double cx, cy;
        P(s, lat, cx, cy);
        for (double dl : { -3.6, 3.6 })
        {
            double x, y;
            P(s, lat + dl, x, y);
            extraPoles.cylinder(x, y, gz, 0.25, 6.0, 10);
            m_circles.push_back({ x, y, 0.3, ObstacleKind::Sign });
        }
        const int schemes[4][3] = { { Palette::Body, Palette::LaneWhite, Palette::NpcBlue },
                                    { Palette::NpcBlue, Palette::LaneYellow, Palette::NpcWhite },
                                    { Palette::NpcGreen, Palette::LaneWhite, Palette::Body },
                                    { Palette::LaneYellow, Palette::Trim, Palette::Body } };
        const int* sc = schemes[bb++ % 4];
        extraBuildings.push_back(SceneBuilder::orientedBlock(cx, cy, yawDeg(s), 0.4, 11.0, gz + 5.6, gz + 10.8, 1, 8, 4,
            [sc](int, int j, int k, double, double, double) {
                if (j == 0 || j == 7 || k == 0 || k == 3) return int(Palette::Trim);   // frame
                if (k == 2) return sc[0];
                return j < 4 ? sc[1] : sc[2];
            }));
        reserve(cx, cy, r.heading(s), 3.0, 8.0);
    }

    // ---- farm fields with vinyl greenhouses inside the loop
    {
        double b[4];
        r.bounds(b);
        int fields = 0;
        Rng frng;
        frng.s = 4242u;
        for (int tries = 0; tries < 600 && fields < 9; ++tries)
        {
            const double x = frng.range(b[0], b[1]), y = frng.range(b[2], b[3]);
            if (!r.insideLoop(x, y))
                continue;
            double ps, pl;
            const double yawR = r.project(x, y, ps, pl) ? r.heading(ps) : 0.0;
            const double L = frng.range(60.0, 85.0), W = frng.range(38.0, 55.0);
            if (!freeArea(x, y, yawR, L + 6.0, W + 6.0, 45.0))
                continue;
            double lo, hi;
            groundUnder(x, y, yawR, L, W, lo, hi);
            if (hi - lo > 1.6)
                continue;   // fields only on flat ground
            const int type = fields % 3;   // 0 vegetables, 1 rice paddy, 2 greenhouses
            ++fields;
            const double c = std::cos(yawR), sn = std::sin(yawR);
            // ground cover draped on the terrain, a little above it
            const int nx = type == 0 ? static_cast<int>(L / 2.6) : (type == 1 ? 6 : 6);
            extraVeg.push_back(SceneBuilder::morphedHex(nx, 6, 1,
                [=](double u, double v, double w, double o[3]) {
                    const double lx = (u - 0.5) * L, ly = (v - 0.5) * W;
                    o[0] = x + c * lx - sn * ly;
                    o[1] = y + sn * lx + c * ly;
                    o[2] = gz + terrainHeight(o[0], o[1]) + 0.10 + 0.06 * w;
                },
                [type](int i, int j, int, double, double, double) {
                    if (type == 0) return (i % 2) ? int(Palette::Leaf) : int(Palette::Trunk);   // crop rows
                    if (type == 1) return ((i + j) % 2) ? int(Palette::LeafLight) : int(Palette::Leaf);   // paddies
                    return int(Palette::Trunk);
                }));
            if (type == 2)
            {
                const int n = static_cast<int>(W / 8.5);
                for (int g = 0; g < n; ++g)
                {
                    const double ly0 = (g + 0.5) * W / n - 0.5 * W;
                    const double len = L - 8.0, rw = 3.1, rh = 3.0;
                    double gx, gy;
                    rectPoint(x, y, yawR, 0.0, ly0, gx, gy);
                    const double z0 = gz + terrainHeight(gx, gy) + 0.1;
                    // vinyl tunnel: half cylinder along the field
                    extraBuildings.push_back(SceneBuilder::morphedHex(10, 10, 1,
                        [=](double u, double v, double w, double o[3]) {
                            const double a = kPi * v;
                            const double rr = 1.0 + 0.02 * w;
                            const double lx = (u - 0.5) * len, ly = ly0 - std::cos(a) * rw * rr;
                            o[0] = x + c * lx - sn * ly;
                            o[1] = y + sn * lx + c * ly;
                            o[2] = z0 + std::sin(a) * rh * rr;
                        },
                        constMat(Palette::NpcWhite)));
                    m_boxes.push_back({ gx, gy, 0.5 * len, rw, yawR, ObstacleKind::Building });
                    lidarProxy.push_back(SceneBuilder::orientedBlock(gx, gy, yawR * kRad2Deg, len, 2.0 * rw, z0,
                                                                     z0 + rh, 1, 1, 1, constMat(Palette::NpcWhite)));
                }
            }
            reserve(x, y, yawR, 0.5 * L + 4.0, 0.5 * W + 4.0);
        }
    }

    // ---- wind turbines on the high ridges outside the loop
    {
        double b[4];
        r.bounds(b);
        const double ccx = 0.5 * (b[0] + b[1]), ccy = 0.5 * (b[2] + b[3]);
        struct Cand { double x, y, h; };
        std::vector<Cand> cands;
        Rng trng;
        trng.s = 9001u;
        for (int i = 0; i < 1500; ++i)
        {
            const double x = trng.range(b[0] - 520.0, b[1] + 520.0), y = trng.range(b[2] - 520.0, b[3] + 520.0);
            if (r.insideLoop(x, y) || underHill(x, y) || nearNet(x, y, 40.0))
                continue;
            double ps, pl;
            if (!r.project(x, y, ps, pl) || std::abs(pl) < 170.0 || std::abs(pl) > 480.0)
                continue;
            const double h = terrainHeight(x, y);
            if (h > 18.0)
                cands.push_back({ x, y, h });
        }
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& c) { return a.h > c.h; });
        std::vector<Cand> chosen;
        for (const auto& cd : cands)
        {
            bool ok = true;
            for (const auto& o : chosen)
                if (std::hypot(o.x - cd.x, o.y - cd.y) < 150.0) { ok = false; break; }
            if (!ok)
                continue;
            chosen.push_back(cd);
            if (chosen.size() >= 10)
                break;
        }
        for (const auto& t : chosen)
        {
            const double z0 = gz + t.h - 1.0, hubZ = z0 + 1.0 + 70.0;
            extraWhite.cylinder(t.x, t.y, z0 - 2.0, 1.7, 73.0, 16);
            // rotor faces the loop (the road): nacelle behind the hub
            double f[2] = { ccx - t.x, ccy - t.y };
            const double fl = std::hypot(f[0], f[1]);
            f[0] /= fl;
            f[1] /= fl;
            const double yawT = std::atan2(f[1], f[0]);
            extraBuildings.push_back(SceneBuilder::orientedBlock(t.x - 1.0 * f[0], t.y - 1.0 * f[1], yawT * kRad2Deg,
                                                                 10.0, 3.6, hubZ - 1.8, hubZ + 1.8, 2, 1, 1,
                                                                 constMat(Palette::NpcWhite)));
            const std::array<double, 3> hub = { t.x + 4.6 * f[0], t.y + 4.6 * f[1], hubZ };
            extraBuildings.push_back(beam({ hub[0] - 0.9 * f[0], hub[1] - 0.9 * f[1], hub[2] },
                                          { hub[0] + 1.2 * f[0], hub[1] + 1.2 * f[1], hub[2] }, 2.4,
                                          Palette::NpcWhite, 1, 0.0));
            const double side[3] = { -f[1], f[0], 0.0 };
            const double phi0 = trng.range(0.0, 2.0 * kPi);
            for (int k = 0; k < 3; ++k)
            {
                const double phi = phi0 + k * 2.0 * kPi / 3.0;
                const double d[3] = { std::cos(phi) * side[0], std::cos(phi) * side[1], std::sin(phi) };
                const double ch[3] = { -std::sin(phi) * side[0], -std::sin(phi) * side[1], std::cos(phi) };
                const double fx = f[0], fy = f[1];
                extraBuildings.push_back(SceneBuilder::morphedHex(8, 1, 1,
                    [=](double u, double v, double w, double o[3]) {
                        const double chord = u < 0.15 ? 1.6 + 1.8 * (u / 0.15) : 0.35 + 3.05 * (1.0 - u) / 0.85;
                        const double ff[3] = { fx, fy, 0.0 };
                        for (int q = 0; q < 3; ++q)
                            o[q] = hub[q] + d[q] * (1.2 + 38.0 * u) + ch[q] * (v - 0.5) * chord + ff[q] * (w - 0.5) * 0.35;
                    },
                    constMat(Palette::NpcWhite)));
            }
            m_circles.push_back({ t.x, t.y, 1.8, ObstacleKind::Building });
        }
    }

    // ---- power line along the outside of the highway and its corners
    {
        const double lat = -(hw + 50.0);
        std::vector<std::array<double, 3>> armL, armR, armL2, armR2;   // wire attachment points
        std::vector<bool> exists;
        for (double s = hwyFrom - 160.0; s < hwyTo + 160.0; s += 110.0)
        {
            double cx, cy;
            P(s, lat, cx, cy);
            double ps, pl;
            const bool ok = !underHill(cx, cy) && !nearTunnel(s, 60.0) && !nearNet(cx, cy, 20.0) && r.project(cx, cy, ps, pl) &&
                            std::abs(pl) > 40.0 && lakeE(cx, cy) > 1.7;
            exists.push_back(ok);
            const double h = r.heading(s);
            const double fw[2] = { std::cos(h), std::sin(h) }, lv[2] = { std::sin(h), -std::cos(h) };
            const double zb = gz + terrainHeight(cx, cy) - 0.5;
            auto at = [&](double a, double l, double z) {
                return std::array<double, 3>{ cx + a * fw[0] + l * lv[0], cy + a * fw[1] + l * lv[1], z };
            };
            armL.push_back(at(0.0, -7.2, zb + 24.0));
            armR.push_back(at(0.0, 7.2, zb + 24.0));
            armL2.push_back(at(0.0, -5.4, zb + 30.0));
            armR2.push_back(at(0.0, 5.4, zb + 30.0));
            if (!ok)
                continue;
            // lattice tower: 4 slanted legs, braces, two cross arms and a peak
            for (int a : { -1, 1 })
                for (int l : { -1, 1 })
                    extraProps.push_back(beam(at(a * 3.6, l * 3.6, zb), at(a * 0.9, l * 0.9, zb + 32.0), 0.35,
                                              Palette::Metal, 4, 0.0));
            for (double zf : { 0.3, 0.6 })
            {
                const double hw2 = 3.6 - 2.7 * zf;
                const double z = zb + 32.0 * zf;
                for (int a : { -1, 1 })
                    extraProps.push_back(beam(at(a * hw2, -hw2, z), at(a * hw2, hw2, z), 0.22, Palette::Metal, 1, 0.0));
                for (int l : { -1, 1 })
                    extraProps.push_back(beam(at(-hw2, l * hw2, z), at(hw2, l * hw2, z), 0.22, Palette::Metal, 1, 0.0));
            }
            extraProps.push_back(beam(at(0.0, -7.6, zb + 24.4), at(0.0, 7.6, zb + 24.4), 0.45, Palette::Metal, 2, 0.0));
            extraProps.push_back(beam(at(0.0, -5.8, zb + 30.4), at(0.0, 5.8, zb + 30.4), 0.4, Palette::Metal, 2, 0.0));
            extraProps.push_back(beam(at(0.0, 0.0, zb + 31.0), at(0.0, 0.0, zb + 35.0), 0.3, Palette::Metal, 1, 0.0));
            reserve(cx, cy, h, 6.0, 9.0);
        }
        // wires between neighboring towers (sagging)
        for (size_t i = 0; i + 1 < exists.size(); ++i)
        {
            if (!exists[i] || !exists[i + 1])
                continue;
            for (const auto* arm : { &armL, &armR, &armL2, &armR2 })
            {
                auto A = (*arm)[i], B = (*arm)[i + 1];
                A[2] -= 0.4;
                B[2] -= 0.4;
                extraProps.push_back(beam(A, B, 0.07, Palette::Trim, 12, 3.5));
            }
        }
    }

    // ---- lake: boats, a pier and a pavilion on the shore
    {
        Rng lrng;
        lrng.s = 777u;
        const double zw = gz - 0.3;
        for (int i = 0; i < 5; ++i)
        {
            const double a = lrng.range(0.0, 2.0 * kPi), rr = std::sqrt(lrng.range(0.05, 0.45));
            const double bx = kLakeX + rr * kLakeRx * std::cos(a), by = kLakeY + rr * kLakeRy * std::sin(a);
            const double yawB = lrng.range(0.0, 2.0 * kPi), c = std::cos(yawB), sn = std::sin(yawB);
            const double L = lrng.range(6.0, 8.5);
            // hull: blunt stern (u = 0), pointed bow (u = 1)
            extraBuildings.push_back(SceneBuilder::morphedHex(8, 4, 2,
                [=](double u, double v, double w, double o[3]) {
                    const double half = 1.3 * std::sqrt(std::sin(kPi * (0.15 + 0.85 * u))) * (0.55 + 0.45 * w);
                    const double lx = (u - 0.5) * L, ly = (v - 0.5) * 2.0 * half;
                    o[0] = bx + c * lx - sn * ly;
                    o[1] = by + sn * lx + c * ly;
                    o[2] = zw - 0.35 + 1.15 * w;
                },
                [](int, int, int k, double, double, double) { return k == 1 ? int(Palette::NpcWhite) : int(Palette::NpcBlue); }));
            // mast + sail (triangle from the mast top to the end of the boom)
            const double mx = bx + c * 0.12 * L, my = by + sn * 0.12 * L, deck = zw + 0.8;
            const double mastH = 1.15 * L;
            extraBuildings.push_back(beam({ mx, my, deck }, { mx, my, deck + mastH }, 0.14, Palette::Trim, 1, 0.0));
            extraBuildings.push_back(SceneBuilder::morphedHex(6, 1, 1,
                [=](double u, double v, double w, double o[3]) {
                    const double z = deck + 0.9 + u * (mastH - 1.0);
                    const double back = (1.0 - u) * 0.42 * L * v;   // toward the stern
                    o[0] = mx - c * back + (w - 0.5) * 0.04 * -sn;
                    o[1] = my - sn * back + (w - 0.5) * 0.04 * c;
                    o[2] = z;
                },
                constMat(Palette::LaneWhite)));
        }
        // pier: from the shore (first dry ground along a ray from the lake center) into the water
        {
            const double th = 0.75;
            const double dx = std::cos(th) * kLakeRx, dy = std::sin(th) * kLakeRy;
            double t = 0.6;
            while (t < 2.0 && terrainHeight(kLakeX + t * dx, kLakeY + t * dy) < 0.15)
                t += 0.01;
            const double sx = kLakeX + (t + 0.02) * dx, sy = kLakeY + (t + 0.02) * dy;
            const double ex = kLakeX + (t - 0.30) * dx, ey = kLakeY + (t - 0.30) * dy;
            const double len = std::hypot(ex - sx, ey - sy), yawP = std::atan2(ey - sy, ex - sx);
            const double zp = gz + std::max(0.35, terrainHeight(sx, sy) + 0.25);
            extraBuildings.push_back(SceneBuilder::orientedBlock(0.5 * (sx + ex), 0.5 * (sy + ey), yawP * kRad2Deg, len,
                                                                 2.6, zp - 0.2, zp, std::max(1, static_cast<int>(len / 1.5)), 2, 1,
                                                                 constMat(Palette::Crate)));
            for (double u = 0.05; u < 1.0; u += 4.0 / len)
                for (double side : { -1.2, 1.2 })
                {
                    const double px = sx + u * (ex - sx) - side * std::sin(yawP), py = sy + u * (ey - sy) + side * std::cos(yawP);
                    extraBuildings.push_back(SceneBuilder::orientedBlock(px, py, yawP * kRad2Deg, 0.25, 0.25, gz - 2.6,
                                                                         zp + 0.9, 1, 1, 1, constMat(Palette::Trunk)));
                }
            // pavilion ("jeongja") on the shore next to the pier
            const double ox = sx + 9.0 * std::cos(th + 1.3), oy = sy + 9.0 * std::sin(th + 1.3);
            const double zo = gz + terrainHeight(ox, oy);
            for (int a : { -1, 1 })
                for (int b2 : { -1, 1 })
                    extraBuildings.push_back(SceneBuilder::orientedBlock(ox + 2.2 * a, oy + 2.2 * b2, 0.0, 0.3, 0.3,
                                                                         zo - 0.5, zo + 3.1, 1, 1, 1,
                                                                         constMat(Palette::Trunk)));
            extraBuildings.push_back(SceneBuilder::orientedBlock(ox, oy, 0.0, 5.4, 5.4, zo - 0.5, zo + 0.6, 2, 2, 1,
                                                                 constMat(Palette::Crate)));
            extraBuildings.push_back(SceneBuilder::gableRoof(ox, oy, 0.0, 6.4, 6.4, zo + 3.1, 1.8, Palette::RoofGray));
            reserve(ox, oy, 0.0, 5.0, 5.0);
            reserve(0.5 * (sx + ex), 0.5 * (sy + ey), yawP, 0.5 * len + 3.0, 3.0);
        }
    }

    // ---- street lights along the back streets (downtown)
    for (double side : { -1.0, 1.0 })
        for (double s = kIntersections[0]; s < kIntersections[std::size(kIntersections) - 1]; s += 21.0)
        {
            bool nearJ = false;
            for (double si : kIntersections)
                if (std::abs(s - si) < kCrossHalf + 2.0)
                    nearJ = true;
            if (nearJ)
                continue;
            const double lp = side * (kBackLat + kCrossHalf + 0.9);
            double x, y, mx, my, xa, ya;
            P(s, lp, x, y);
            P(s, lp - side * 1.6, mx, my);
            P(s, lp - side * 3.2, xa, ya);
            extraPoles.cylinder(x, y, gz, 0.09, 6.0, 10);
            m_circles.push_back({ x, y, 0.12, ObstacleKind::StreetLight });
            extraProps.push_back(SceneBuilder::orientedBlock(mx, my, yawDeg(s), 0.12, 3.4, 5.75, 5.85, 1, 2, 1,
                                                             constMat(Palette::LampPost)));
            extraProps.push_back(SceneBuilder::orientedBlock(xa, ya, yawDeg(s), 0.5, 0.4, 5.6, 5.75, 1, 1, 1,
                                                             constMat(Palette::Lamp)));
            lidarProxy.push_back(extraProps.back());
        }

    // ---------------------------------------------------------------
    // Trees (batched): conifers in the forest / on the hills, broadleaf trees in the city, park, suburb
    // ---------------------------------------------------------------
    PolyBatch trunks, conesDark, conesMid;
    PolyBatch lidarTrunks, lidarCrowns;   // simplified trees for ray casting (a few cells each)
    std::vector<std::array<double, 6>> crownsA, crownsB, crownsC;
    auto conifer = [&](double x, double y, double sc) {
        if (isReserved(x, y) || nearNet(x, y, 7.0))
            return;
        const double z0 = gz + std::max(0.0, terrainHeight(x, y));
        trunks.cylinder(x, y, z0 - 0.3, 0.16 * sc, 2.4 * sc);
        PolyBatch& b = rng.next() < 0.5 ? conesDark : conesMid;
        b.cone(x, y, z0 + 1.0 * sc, 1.9 * sc, 2.9 * sc);
        b.cone(x, y, z0 + 2.5 * sc, 1.45 * sc, 2.6 * sc);
        b.cone(x, y, z0 + 3.9 * sc, 0.95 * sc, 2.3 * sc);
        lidarTrunks.cylinder(x, y, z0 - 0.3, 0.16 * sc, 1.6 * sc, 4);
        lidarCrowns.cone(x, y, z0 + 1.0 * sc, 1.8 * sc, 5.2 * sc, 6);
        m_circles.push_back({ x, y, 0.25, ObstacleKind::Tree });
    };
    auto broadleaf = [&](double x, double y, double sc) {
        if (isReserved(x, y) || nearNet(x, y, 7.0))
            return;
        const double z0 = gz + std::max(0.0, terrainHeight(x, y));
        trunks.cylinder(x, y, z0 - 0.3, 0.2 * sc, 3.3 * sc);
        const double pick = rng.next();
        auto& crowns = pick < 0.4 ? crownsA : (pick < 0.7 ? crownsB : crownsC);
        crowns.push_back({ x, y, z0 + 3.9 * sc, 2.1 * sc, 2.1 * sc, 1.8 * sc });
        crowns.push_back({ x + 0.9 * sc, y + 0.5 * sc, z0 + 4.7 * sc, 1.4 * sc, 1.4 * sc, 1.2 * sc });
        crowns.push_back({ x - 0.7 * sc, y - 0.8 * sc, z0 + 4.4 * sc, 1.3 * sc, 1.3 * sc, 1.1 * sc });
        lidarTrunks.cylinder(x, y, z0 - 0.3, 0.2 * sc, 2.5 * sc, 4);
        lidarCrowns.cylinder(x, y, z0 + 2.3 * sc, 2.0 * sc, 3.2 * sc, 6);
        m_circles.push_back({ x, y, 0.25, ObstacleKind::Tree });
    };

    // along the road outside the city: dense forest, sparser elsewhere
    for (const auto& pc : r.pieces())
    {
        if (pc.zone == RoadModel::Zone::City)
            continue;
        const bool forest = pc.zone == RoadModel::Zone::Forest;
        const bool suburb = pc.zone == RoadModel::Zone::Suburb;
        const double latMin = suburb ? 36.0 : (forest ? hw + 3.0 : hw + 5.0);
        for (int side : { -1, 1 })
            for (double s = pc.s0 + rng.range(0, 4); s < pc.s0 + pc.len;
                 s += forest ? rng.range(2.5, 5.0) : rng.range(6.0, 12.0))
            {
                const double lat = side * rng.range(latMin, forest ? 75.0 : 60.0);
                double x, y;
                P(s, lat, x, y);
                if (underHill(x, y))
                    continue;
                double ps, pl;
                if (r.project(x, y, ps, pl) &&
                    (std::abs(pl) < latMin - 0.5 || (r.zoneAt(ps) == RoadModel::Zone::City && std::abs(pl) < 135.0)))
                    continue;   // too close to another part of the road / inside the city
                if (lakeE(x, y) < 1.5)
                    continue;
                if (suburb || rng.next() < 0.18)
                    broadleaf(x, y, rng.range(0.8, 1.25));
                else
                    conifer(x, y, rng.range(0.8, 1.4));
            }
    }
    // park inside the loop (around the lake)
    {
        double b[4];
        r.bounds(b);
        for (int i = 0; i < 1100; ++i)
        {
            const double x = rng.range(b[0], b[1]), y = rng.range(b[2], b[3]);
            if (!r.insideLoop(x, y) || lakeE(x, y) < 1.45 || underHill(x, y))
                continue;
            double s, lat;
            if (r.project(x, y, s, lat) &&
                (std::abs(lat) < 40.0 || (r.zoneAt(s) == RoadModel::Zone::City && std::abs(lat) < 135.0)))
                continue;
            broadleaf(x, y, rng.range(0.8, 1.3));
        }
    }
    // hills outside the loop: scattered conifers
    {
        double b[4];
        r.bounds(b);
        for (int i = 0; i < 1600; ++i)
        {
            const double x = rng.range(b[0] - 230, b[1] + 230), y = rng.range(b[2] - 230, b[3] + 230);
            if (r.insideLoop(x, y) || underHill(x, y))
                continue;
            double s, lat;
            if (r.project(x, y, s, lat) &&
                (std::abs(lat) < 70.0 || (r.zoneAt(s) == RoadModel::Zone::City && std::abs(lat) < 140.0)))
                continue;
            conifer(x, y, rng.range(1.0, 1.7));
        }
    }

    // ---------------------------------------------------------------
    // City blocks: buildings (two rows, some glass towers), sidewalks + curbs, street lights, street trees
    // ---------------------------------------------------------------
    std::vector<double> edges = { 2.0 };
    for (double si : kIntersections)
    {
        edges.push_back(si - kCrossHalf);
        edges.push_back(si + kCrossHalf);
    }
    edges.push_back(cityTo - 2.0);

    std::vector<Grid> buildings, cityProps;
    PolyBatch lampPoles;
    int bIndex = 0;
    for (size_t bk = 0; bk + 1 < edges.size(); bk += 2)
    {
        const double s0 = edges[bk], s1 = edges[bk + 1];   // block from s0 to s1
        if (s1 - s0 < 4.0)
            continue;

        for (double side : { -1.0, 1.0 })
        {
            // sidewalk + curb
            const double la = side * kWalkIn, lb = side * kWalkOut;
            cityProps.push_back(SceneBuilder::ribbon(r, s0, s1, std::min(la, lb), std::max(la, lb), gz + 0.12, 2.0,
                                                     Palette::Sidewalk));
            cityProps.push_back(SceneBuilder::wallAlong(r, s0, s1, side * (kWalkIn + 0.08), 0.16, gz, gz + 0.13, 4.0,
                                                        Palette::Concrete));

            // two rows of buildings
            // rows: main street front, behind it, facing the back street, beyond the back street
            for (int row = 0; row < 4; ++row)
            {
                double s = s0 + 1.0;
                while (s + 6.0 < s1 - 1.0)
                {
                    const double w = std::min(rng.range(9.0, 15.0), s1 - 1.0 - s);
                    const double d = row == 3 ? rng.range(12.0, 20.0) : rng.range(8.0, 12.0);
                    const bool tower = (row == 1 || row == 3) && rng.next() < (row == 3 ? 0.5 : 0.35);
                    const double h = row == 0   ? rng.range(10.0, 26.0)
                                     : row == 2 ? rng.range(12.0, 30.0)
                                                : (tower ? rng.range(45.0, 90.0) : rng.range(20.0, 44.0));
                    const double latIn = row == 0 ? 14.5 : row == 1 ? 30.0 : row == 2 ? 56.5 : 88.0;
                    const double latC = side * (latIn + 0.5 * d);
                    double cx, cy;
                    P(s + 0.5 * w, latC, cx, cy);
                    if (lakeE(cx, cy) < 1.6)
                    {
                        s += w + 3.0;
                        continue;
                    }
                    const double yaw = yawDeg(s + 0.5 * w);

                    const int nx = std::max(2, int(std::round(w / 2.2)));
                    const int ny = std::max(2, int(std::round(d / 2.2)));
                    const int nz = std::max(2, int(std::round(h / 3.4)));
                    const int wall = tower ? int(Palette::Concrete)
                                           : ((bIndex++ % 3 == 0) ? int(Palette::Brick) : int(Palette::Concrete));

                    buildings.push_back(SceneBuilder::orientedBlock(cx, cy, yaw, w, d, gz, gz + h, nx, ny, nz,
                        [=](int i, int j, int k, double, double, double) {
                            if (k == 0 || k == nz - 1) return wall;
                            const bool faceX = (i == 0 || i == nx - 1);
                            const bool faceY = (j == 0 || j == ny - 1);
                            if (tower) return (faceX || faceY) ? int(Palette::Window) : wall;   // curtain wall
                            if (faceY && i % 2 == 1) return int(Palette::Window);
                            if (faceX && j % 2 == 1) return int(Palette::Window);
                            return wall;
                        }));
                    // rooftop units
                    if (rng.next() < 0.6)
                    {
                        double ux, uy;
                        P(s + 0.5 * w + rng.range(-0.2, 0.2) * w, latC + rng.range(-0.2, 0.2) * d, ux, uy);
                        buildings.push_back(SceneBuilder::orientedBlock(ux, uy, yaw, rng.range(2.0, 4.0),
                                                                        rng.range(2.0, 3.5), gz + h,
                                                                        gz + h + rng.range(1.2, 2.5), 1, 1, 1,
                                                                        constMat(Palette::Concrete)));
                    }
                    m_boxes.push_back({ cx, cy, 0.5 * w, 0.5 * d, yaw / kRad2Deg, ObstacleKind::Building });
                    lidarProxy.push_back(SceneBuilder::orientedBlock(cx, cy, yaw, w, d, gz, gz + h, 1, 1, 1,
                                                                     constMat(tower ? int(Palette::Window) : wall)));
                    s += w + rng.range(2.0, 5.0);
                }
            }

            // street lights and street trees on the sidewalk, alternating
            int n = 0;
            for (double s = s0 + 3.0; s < s1 - 2.0; s += 12.0, ++n)
            {
                if (n % 2 == 0)
                {
                    double x, y, xa, ya, mx, my;
                    P(s, side * 10.4, x, y);
                    P(s, side * 7.0, xa, ya);
                    P(s, side * 8.7, mx, my);
                    lampPoles.cylinder(x, y, gz, 0.09, 6.0, 10);
                    m_circles.push_back({ x, y, 0.12, ObstacleKind::StreetLight });
                    cityProps.push_back(SceneBuilder::orientedBlock(mx, my, yawDeg(s), 0.12, 3.4, 5.75, 5.85, 1, 2, 1,
                                                                    constMat(Palette::LampPost)));
                    cityProps.push_back(SceneBuilder::orientedBlock(xa, ya, yawDeg(s), 0.5, 0.4, 5.6, 5.75, 1, 1, 1,
                                                                    constMat(Palette::Lamp)));
                    lidarProxy.push_back(cityProps[cityProps.size() - 2]);
                    lidarProxy.push_back(cityProps.back());
                }
                else
                {
                    double x, y;
                    P(s, side * 12.0, x, y);
                    broadleaf(x, y, 0.7);
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Suburb: houses with gable roofs, sidewalks, street lights
    // ---------------------------------------------------------------
    {
        const int roofs[] = { Palette::Roof, Palette::RoofGray, Palette::Roof };
        for (double side : { -1.0, 1.0 })
        {
            const double la = side * kWalkIn, lb = side * kSuburbWalkOut;
            cityProps.push_back(SceneBuilder::ribbon(r, subFrom, subTo, std::min(la, lb), std::max(la, lb), gz + 0.12,
                                                     2.0, Palette::Sidewalk));
            cityProps.push_back(SceneBuilder::wallAlong(r, subFrom, subTo, side * (kWalkIn + 0.08), 0.16, gz,
                                                        gz + 0.13, 4.0, Palette::Concrete));
            for (double s = subFrom + 12.0; s < subTo - 12.0; s += rng.range(19.0, 25.0))
            {
                if (side > 0 && s > gasS0 - 8.0 && s < gasS1 + 6.0)
                    continue;   // gas station
                const double L = rng.range(9.0, 12.0), W = rng.range(7.0, 9.0);
                const double wallH = rng.range(5.0, 6.2), gable = rng.range(2.2, 3.0);
                const double latC = side * rng.range(20.0, 26.0);
                double cx, cy;
                P(s, latC, cx, cy);
                if (nearNet(cx, cy, 18.0))
                    continue;   // the ring road's underpass
                const double yaw = yawDeg(s);
                const double c = std::cos(yaw / kRad2Deg), sn = std::sin(yaw / kRad2Deg);
                const int wm = rng.next() < 0.33 ? int(Palette::Brick) : int(Palette::Stucco);
                // body whose top follows the gable (closed gable ends)
                buildings.push_back(SceneBuilder::morphedHex(4, 2, 2,
                    [&](double u, double v, double w, double out[3]) {
                        const double lx = (u - 0.5) * L, ly = (v - 0.5) * W;
                        out[0] = cx + c * lx - sn * ly;
                        out[1] = cy + sn * lx + c * ly;
                        out[2] = gz + w * (wallH + gable * (1.0 - std::abs(2.0 * v - 1.0)));
                    },
                    [wm](int i, int j, int k, double, double, double) {
                        (void)j;
                        return (k == 0 && (i == 1 || i == 2)) ? int(Palette::Window) : wm;
                    }));
                buildings.push_back(SceneBuilder::gableRoof(cx, cy, yaw, L, W, gz + wallH, gable,
                                                            roofs[static_cast<int>(rng.next() * 3) % 3]));
                m_boxes.push_back({ cx, cy, 0.5 * L, 0.5 * W, yaw / kRad2Deg, ObstacleKind::Building });
                lidarProxy.push_back(SceneBuilder::orientedBlock(cx, cy, yaw, L, W, gz, gz + wallH + 0.5 * gable,
                                                                 1, 1, 1, constMat(wm)));
                // garden tree
                double tx, ty;
                P(s + 0.5 * L + 3.0, side * 15.5, tx, ty);
                broadleaf(tx, ty, rng.range(0.7, 1.0));
            }
            for (double s = subFrom + 8.0; s < subTo; s += 30.0)
            {
                double x, y, mx, my, xa, ya;
                P(s, side * 10.6, x, y);
                P(s, side * 9.0, mx, my);
                P(s, side * 7.3, xa, ya);
                lampPoles.cylinder(x, y, gz, 0.09, 6.0, 10);
                m_circles.push_back({ x, y, 0.12, ObstacleKind::StreetLight });
                cityProps.push_back(SceneBuilder::orientedBlock(mx, my, yawDeg(s), 0.12, 3.3, 5.75, 5.85, 1, 2, 1,
                                                                constMat(Palette::LampPost)));
                cityProps.push_back(SceneBuilder::orientedBlock(xa, ya, yawDeg(s), 0.5, 0.4, 5.6, 5.75, 1, 1, 1,
                                                                constMat(Palette::Lamp)));
                lidarProxy.push_back(cityProps[cityProps.size() - 2]);
                lidarProxy.push_back(cityProps.back());
            }
        }
    }

    // highway: lamps on the median barrier
    for (double s = hwyFrom + 20.0; s < hwyTo; s += 45.0)
    {
        if (nearTunnel(s, 8.0))
            continue;
        double x, y;
        P(s, 0.0, x, y);
        if (nearNet(x, y, 25.0))
            continue;   // under the viaduct
        lampPoles.cylinder(x, y, gz + 0.85, 0.1, 9.3, 10);
        for (double side : { -1.0, 1.0 })
        {
            double mx, my, xa, ya;
            P(s, side * 2.2, mx, my);
            P(s, side * 4.4, xa, ya);
            cityProps.push_back(SceneBuilder::orientedBlock(mx, my, yawDeg(s), 0.12, 4.4, 10.0, 10.1, 1, 2, 1,
                                                            constMat(Palette::LampPost)));
            cityProps.push_back(SceneBuilder::orientedBlock(xa, ya, yawDeg(s), 0.6, 0.4, 9.85, 10.0, 1, 1, 1,
                                                            constMat(Palette::Lamp)));
            lidarProxy.push_back(cityProps[cityProps.size() - 2]);
            lidarProxy.push_back(cityProps.back());
        }
    }
    cityProps.push_back(lampPoles.grid(Palette::LampPost));
    lidarProxy.push_back(cityProps.back());   // sidewalks are NOT LiDAR geometry: groundMaterial() covers them

    {
        // the glass of the buildings is a separate PID (reflective PBR material)
        buildings.insert(buildings.end(), extraBuildings.begin(), extraBuildings.end());
        if (extraWhite.pts->GetNumberOfPoints() > 0)
            buildings.push_back(extraWhite.grid(Palette::NpcWhite));
        Grid all = SceneBuilder::merge(buildings);
        Grid glass, rest;
        splitMaterial(all, Palette::Window, glass, rest);
        m_pidGrids[PidBuildings] = rest;
        m_pidGrids[PidGlass] = glass;
    }
    cityProps.insert(cityProps.end(), extraProps.begin(), extraProps.end());
    if (extraPoles.pts->GetNumberOfPoints() > 0)
    {
        cityProps.push_back(extraPoles.grid(Palette::LampPost));
        lidarProxy.push_back(cityProps.back());
    }
    m_pidGrids[PidLights] = SceneBuilder::merge(cityProps);

    // ---------------------------------------------------------------
    // Traffic lights (city), road signs, corner chevrons, highway gantries
    // ---------------------------------------------------------------
    {
        std::vector<Grid> signals;
        PolyBatch poles, signPoles;
        m_lamps.clear();
        // lamp polygon in the plane spanned by 'right' (horizontal) and z, centered at c
        auto lampQuad = [&](int j, int dir, int slot, const double c[3], const double right[2], double h) {
            SignalLamp L{ j, dir, slot, {}, {} };
            const double du[2] = { right[0] * h, right[1] * h };
            L.pts = { { c[0] - du[0], c[1] - du[1], c[2] - h }, { c[0] + du[0], c[1] + du[1], c[2] - h },
                      { c[0] + du[0], c[1] + du[1], c[2] + h }, { c[0] - du[0], c[1] - du[1], c[2] + h } };
            L.polySizes = { 4 };
            m_lamps.push_back(L);
        };
        // left-turn arrow: triangle head + bar, pointing along 'left'
        auto lampArrow = [&](int j, int dir, const double c[3], const double left[2]) {
            SignalLamp L{ j, dir, 2, {}, {} };
            auto at = [&](double u, double v) {
                return std::array<double, 3>{ c[0] + left[0] * u, c[1] + left[1] * u, c[2] + v };
            };
            L.pts = { at(0.12, 0.0), at(0.0, 0.10), at(0.0, -0.10),
                      at(0.0, 0.035), at(-0.12, 0.035), at(-0.12, -0.035), at(0.0, -0.035) };
            L.polySizes = { 3, 4 };
            m_lamps.push_back(L);
        };
        // unit vectors of the road frame at s: along +s and toward +lat
        auto frame = [&](double s, double fwd[2], double latV[2]) {
            const double h = r.heading(s);
            fwd[0] = std::cos(h);
            fwd[1] = std::sin(h);
            latV[0] = std::sin(h);
            latV[1] = -std::cos(h);
        };

        for (size_t jj = 0; jj < m_intersections.size(); ++jj)
        {
            const int j = static_cast<int>(jj);
            const double si = m_intersections[jj];
            // one signal per direction on the near side, arm over all three lanes of that direction
            for (double side : { 1.0, -1.0 })   // +1: player direction (right half, approaching from smaller s)
            {
                const int dir = side > 0 ? 1 : -1;
                const double ss = si - side * (kCrosswalk + 2.2);
                const double yaw = yawDeg(ss);
                double fwd[2], latV[2];
                frame(ss, fwd, latV);
                double x, y;
                P(ss, side * (hw + 1.0), x, y);
                poles.cylinder(x, y, gz, 0.1, 6.35, 12);
                m_circles.push_back({ x, y, 0.14, ObstacleKind::TrafficLight });
                double ax, ay;
                P(ss, side * (0.5 * (hw + 1.0) + 0.5), ax, ay);
                signals.push_back(SceneBuilder::orientedBlock(ax, ay, yaw, 0.14, hw, 6.1, 6.25, 1, 6, 1,
                                                              constMat(Palette::Trim)));
                // Korean horizontal 4-lamp heads over every lane: red | yellow | left arrow | green
                // (left to right as the driver sees them)
                const double driverLeft[2] = { -side * latV[0], -side * latV[1] };
                for (int lane = 0; lane < RoadModel::kLanes; ++lane)
                {
                    double hx, hy;
                    P(ss, side * r.laneOffset(lane), hx, hy);
                    signals.push_back(SceneBuilder::orientedBlock(hx, hy, yaw, 0.32, 1.30, 5.50, 5.95, 1, 4, 1,
                                                                  constMat(Palette::Trim)));
                    // sun visor over the lamps
                    signals.push_back(SceneBuilder::orientedBlock(hx - side * 0.24 * fwd[0], hy - side * 0.24 * fwd[1],
                                                                  yaw, 0.16, 1.30, 5.95, 5.99, 1, 1, 1,
                                                                  constMat(Palette::Trim)));
                    for (int k = 0; k < 4; ++k)
                    {
                        double c[3];
                        P(ss - side * 0.17, side * (r.laneOffset(lane) - 0.45 + 0.30 * k), c[0], c[1]);
                        c[2] = 5.725;
                        if (k == 2)
                            lampArrow(j, dir, c, driverLeft);
                        else
                            lampQuad(j, dir, k, c, latV, 0.11);
                    }
                }
                // small 3-lamp head low on the same pole for the cross street, facing it
                {
                    const double latC = side * (hw + 1.0);
                    double hx, hy;
                    P(ss, latC, hx, hy);
                    signals.push_back(SceneBuilder::orientedBlock(hx, hy, yaw, 0.95, 0.30, 3.65, 4.05, 3, 1, 1,
                                                                  constMat(Palette::Trim)));
                    const int slots[3] = { 0, 1, 3 };
                    for (int k = 0; k < 3; ++k)
                    {
                        double c[3];
                        P(ss - side * 0.30 * (k - 1), latC + side * 0.16, c[0], c[1]);
                        c[2] = 3.85;
                        lampQuad(j, 0, slots[k], c, fwd, 0.10);
                    }
                }
            }
        }
        // round speed signs (forest, highway, suburb), right-hand side
        auto addSign = [&](double s) {
            if (nearTunnel(s, 10.0))
                return;
            double x, y;
            P(s, hw + 1.6, x, y);
            signPoles.cylinder(x, y, gz, 0.05, 2.4, 8);
            const double h = r.heading(s);
            signals.push_back(SceneBuilder::orientedBlock(x - 0.03 * std::cos(h), y - 0.03 * std::sin(h), h * kRad2Deg,
                                                          0.04, 0.8, 1.8, 2.6, 1, 1, 1, constMat(Palette::SignRed)));
            signals.push_back(SceneBuilder::orientedBlock(x - 0.06 * std::cos(h), y - 0.06 * std::sin(h), h * kRad2Deg,
                                                          0.03, 0.6, 1.9, 2.5, 1, 1, 1, constMat(Palette::LaneWhite)));
            m_circles.push_back({ x, y, 0.08, ObstacleKind::Sign });
        };
        for (double s = forestFrom + 15.0; s < forestTo; s += 70.0) addSign(s);
        for (double s = hwyFrom + 60.0; s < hwyTo; s += 90.0) addSign(s);
        for (double s = subFrom + 25.0; s < subTo; s += 80.0) addSign(s);

        // chevrons on the outer side of every corner
        for (const auto& pc : r.pieces())
        {
            if (pc.zone != RoadModel::Zone::Corner)
                continue;
            for (double s = pc.s0 + 6.0; s < pc.s0 + pc.len - 4.0; s += 16.0)
            {
                double x, y;
                P(s, -(hw + 2.4), x, y);
                signPoles.cylinder(x, y, gz, 0.05, 1.8, 8);
                signals.push_back(SceneBuilder::orientedBlock(x, y, yawDeg(s), 0.05, 0.8, 1.1, 1.8, 1, 3, 1,
                    [](int, int j, int, double, double, double) {
                        return j == 1 ? int(Palette::Trim) : int(Palette::LaneYellow);
                    }));
                m_circles.push_back({ x, y, 0.08, ObstacleKind::Sign });
            }
        }

        // highway gantries: green boards over the lanes of each direction
        for (int dir : { 1, -1 })
        {
            const double s = dir > 0 ? hwyFrom + 35.0 : hwyTo - 35.0;
            const double yaw = yawDeg(s);
            for (double side : { -1.0, 1.0 })
            {
                double x, y;
                P(s, side * (hw + 2.2), x, y);
                poles.cylinder(x, y, gz, 0.2, 7.4, 12);
                m_circles.push_back({ x, y, 0.25, ObstacleKind::Sign });
            }
            double bx, by, gx, gy;
            P(s, 0.0, bx, by);
            signals.push_back(SceneBuilder::orientedBlock(bx, by, yaw, 0.35, 2.0 * hw + 4.4, 6.9, 7.3, 1, 8, 1,
                                                          constMat(Palette::Metal)));
            P(s, dir * 4.5, gx, gy);
            signals.push_back(SceneBuilder::orientedBlock(gx, gy, yaw, 0.12, 8.0, 5.3, 6.9, 1, 4, 2,
                [](int, int j, int k, double, double, double) {
                    return (k == 1 && (j == 1 || j == 2)) ? int(Palette::LaneWhite) : int(Palette::SignGreen);
                }));
        }

        signals.push_back(poles.grid(Palette::Trim));
        signals.push_back(signPoles.grid(Palette::LampPost));
        m_pidGrids[PidPoles] = SceneBuilder::merge(signals);
    }

    // ---------------------------------------------------------------
    // Highway tunnel (walls, vault, portals, lights) and the hill over it
    // ---------------------------------------------------------------
    m_tunnelHill = buildTunnel();
    // ring road viaduct / underpass (own PID) + the light strips under the cover slab
    m_pidGrids[PidBridge] = SceneBuilder::merge(ring.bridge);
    lidarProxy.insert(lidarProxy.end(), ring.lidar.begin(), ring.lidar.end());
    if (!ring.lights.empty())
    {
        std::vector<Grid> lights(ring.lights.begin(), ring.lights.end());
        if (m_tunnelLights)
            lights.push_back(m_tunnelLights);
        m_tunnelLights = SceneBuilder::merge(lights);
    }

    // trees (after every placement that adds street / garden trees)
    {
        std::vector<Grid> parts = { trunks.grid(Palette::Trunk), conesDark.grid(Palette::LeafDark),
                                    conesMid.grid(Palette::Leaf) };
        if (!crownsA.empty()) parts.push_back(SceneBuilder::blobGroup(crownsA, Palette::Leaf, 10));
        if (!crownsB.empty()) parts.push_back(SceneBuilder::blobGroup(crownsB, Palette::LeafLight, 10));
        if (!crownsC.empty()) parts.push_back(SceneBuilder::blobGroup(crownsC, Palette::LeafDark, 10));
        parts.insert(parts.end(), extraVeg.begin(), extraVeg.end());   // farm fields
        m_pidGrids[PidTrees] = SceneBuilder::merge(parts);
    }

    // ---------------------------------------------------------------
    // Terrain (hills, lake) — also part of the LiDAR scene
    // ---------------------------------------------------------------
    buildTerrain();

    // ---------------------------------------------------------------
    // Traffic (moving cars, simulated by DriveSim)
    // ---------------------------------------------------------------
    m_npcSpawns = {
        // same direction as the player
        { 70.0, 2, 6.0, Palette::NpcBlue,  +1 },
        { 135.0, 0, 8.0, Palette::NpcWhite, +1 },
        { 160.0, 1, 5.5, Palette::NpcGreen, +1 },
        { 225.0, 2, 6.5, Palette::NpcWhite, +1 },
        { 290.0, 0, 7.5, Palette::NpcBlue,  +1 },
        { 345.0, 1, 6.0, Palette::NpcGreen, +1 },
        { hwyFrom + 60.0,  1, 11.0, Palette::NpcBlue,  +1 },
        { hwyFrom + 140.0, 2, 9.0,  Palette::NpcWhite, +1 },
        { hwyFrom + 220.0, 0, 13.0, Palette::NpcGreen, +1 },
        { subFrom + 40.0,  2, 7.0,  Palette::NpcWhite, +1 },
        { subFrom + 160.0, 1, 8.0,  Palette::NpcBlue,  +1 },
        // oncoming traffic
        { 90.0,  0,  9.0, Palette::NpcWhite, -1 },
        { 170.0, 1,  8.0, Palette::NpcBlue,  -1 },
        { 250.0, 2,  7.0, Palette::NpcGreen, -1 },
        { 330.0, 0, 10.0, Palette::NpcWhite, -1 },
        { forestFrom + 80.0,  1, 10.0, Palette::NpcBlue,  -1 },
        { forestFrom + 230.0, 0, 11.0, Palette::NpcGreen, -1 },
        { hwyFrom + 100.0, 2, 14.0, Palette::NpcWhite, -1 },
        { hwyFrom + 300.0, 1, 15.0, Palette::NpcBlue,  -1 },
        { subFrom + 100.0, 0,  9.0, Palette::NpcGreen, -1 },
    };
    // street network: about one car per 170 m, alternating directions
    {
        const int paints[] = { Palette::NpcWhite, Palette::NpcBlue, Palette::NpcGreen, Palette::Body };
        int k = 0;
        for (size_t r = 0; r < m_net.size(); ++r)
        {
            const double L = m_net[r].road.length();
            const int cnt = std::max(1, static_cast<int>(L / 170.0));
            for (int i = 0; i < cnt; ++i, ++k)
                m_npcSpawns.push_back({ (i + 0.5) * L / cnt, (k / 2) % m_net[r].road.lanes, 0.0, paints[k % 4],
                                        k % 2 ? -1 : 1, static_cast<int>(r) + 1 });
        }
    }
    for (const auto& sp : m_npcSpawns)
        m_npcGrids.push_back(VehicleShapes::buildSedan(sp.paint));

    // ---------------------------------------------------------------
    // LiDAR-visible surface (world coords): simple objects as they are + box proxies + hills
    // ---------------------------------------------------------------
    std::vector<vtkSmartPointer<vtkUnstructuredGrid>> lidarGrids = {
        m_pidGrids[PidPoles], m_pidGrids[PidWall], lidarTrunks.grid(Palette::Trunk),
        lidarCrowns.grid(Palette::Leaf), SceneBuilder::merge(lidarProxy), m_pidGrids[PidTunnel],
        SceneBuilder::fromPolyData(m_tunnelHill, Palette::Grass) };
    vtkNew<vtkAppendPolyData> append;
    for (auto& g : lidarGrids)
    {
        vtkNew<vtkDataSetSurfaceFilter> surface;
        surface->SetInputData(g);
        surface->Update();
        auto pd = vtkSmartPointer<vtkPolyData>::New();
        pd->ShallowCopy(surface->GetOutput());
        pd->GetPointData()->Initialize();
        append->AddInputData(pd);
    }
    append->Update();
    // triangles only: vtkTriangle::IntersectWithLine is much cheaper than the generic polygon test
    vtkNew<vtkTriangleFilter> tri;
    tri->SetInputConnection(append->GetOutputPort());
    tri->PassLinesOff();
    tri->PassVertsOff();
    tri->Update();
    m_obstacleSurface = vtkSmartPointer<vtkPolyData>::New();
    m_obstacleSurface->DeepCopy(tri->GetOutput());
}

void World::buildTerrain()
{
    const RoadModel& r = m_road;
    const double gz = r.groundZ;
    double b[4];
    r.bounds(b);
    const double step = 10.0;
    const double x0 = b[0] - 600.0, x1 = b[1] + 600.0, y0 = b[2] - 600.0, y1 = b[3] + 600.0;
    const int nx = static_cast<int>((x1 - x0) / step) + 1, ny = static_cast<int>((y1 - y0) / step) + 1;

    std::vector<double> H(static_cast<size_t>(nx) * ny);
    vtkNew<vtkPoints> pts;
    pts->SetDataTypeToDouble();
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetName("Colors");
    colors->SetNumberOfComponents(3);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
        {
            const double x = x0 + i * step, y = y0 + j * step;
            const double h = terrainHeight(x, y);
            H[static_cast<size_t>(j) * nx + i] = h;
            // slightly below the road / ground plane so flat ground never fights with the road
            pts->InsertNextPoint(x, y, gz - 0.06 + h);
        }

    // colors: grass with variation, dry patches, darker forest floor, rock on steep / high ground,
    // sand at the lake shore
    auto colorAt = [&](double x, double y, double h, double slope, unsigned char rgb[3]) {
            const double n = fbm(x * 0.02, y * 0.02);
            double c[3] = { 0.27 + 0.10 * n, 0.40 + 0.12 * n, 0.18 + 0.05 * n };   // grass
            const double dry = smoothstep(0.55, 0.75, fbm(x * 0.006 + 40.0, y * 0.006));
            const double drygrass[3] = { 0.48, 0.47, 0.26 };
            for (int k = 0; k < 3; ++k) c[k] += (drygrass[k] - c[k]) * 0.6 * dry;
            const double forest = smoothstep(4.0, 14.0, h) * 0.5;
            const double dark[3] = { 0.13, 0.25, 0.12 };
            for (int k = 0; k < 3; ++k) c[k] += (dark[k] - c[k]) * forest;
            const double rock = std::max(smoothstep(0.45, 0.8, slope), smoothstep(45.0, 70.0, h));
            const double rockC[3] = { 0.47, 0.45, 0.42 };
            for (int k = 0; k < 3; ++k) c[k] += (rockC[k] - c[k]) * rock;
            if (h < 0.05 && lakeE(x, y) < 2.0)
            {
                const double sand[3] = { 0.62, 0.56, 0.42 };
                const double t = smoothstep(0.05, -0.6, h);
                for (int k = 0; k < 3; ++k) c[k] += (sand[k] - c[k]) * t;
            }
            for (int k = 0; k < 3; ++k)
                rgb[k] = static_cast<unsigned char>(std::clamp(c[k], 0.0, 1.0) * 255.0);
    };
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
        {
            const double x = x0 + i * step, y = y0 + j * step;
            const double h = H[static_cast<size_t>(j) * nx + i];
            const double hx = H[static_cast<size_t>(j) * nx + std::min(i + 1, nx - 1)] -
                              H[static_cast<size_t>(j) * nx + std::max(i - 1, 0)];
            const double hy = H[static_cast<size_t>(std::min(j + 1, ny - 1)) * nx + i] -
                              H[static_cast<size_t>(std::max(j - 1, 0)) * nx + i];
            const double slope = std::sqrt(hx * hx + hy * hy) / (2.0 * step);
            unsigned char rgb[3];
            colorAt(x, y, h, slope, rgb);
            colors->InsertNextTypedTuple(rgb);
        }

    // the open trench of the underpass: the coarse terrain grid would bury it ->
    // leave those cells out and lay a fine "skirt" from the retaining walls outward instead
    const int ur = m_underRoad;
    auto inCut = [&](double x, double y, double margin) {
        if (ur < 0)
            return false;
        double rs, rl;
        if (!m_net[ur].road.project(x, y, rs, rl) || std::abs(rl) > netWallLat(ur) + margin)
            return false;
        const int st = netStructure(ur, rs);
        return st == NetTrench || st == NetCovered;
    };
    vtkNew<vtkCellArray> quads;
    for (int j = 0; j + 1 < ny; ++j)
        for (int i = 0; i + 1 < nx; ++i)
        {
            const double xc = x0 + (i + 0.5) * step, yc = y0 + (j + 0.5) * step;
            if (inCut(xc, yc, 7.0))
                continue;
            const vtkIdType a = static_cast<vtkIdType>(j) * nx + i;
            const vtkIdType q[4] = { a, a + 1, a + nx + 1, a + nx };
            quads->InsertNextCell(4, q);
        }
    if (ur >= 0)
    {
        const RoadModel& rr = m_net[ur].road;
        double a = 1e18, b = -1e18;
        for (double s = 0.0; s <= rr.length(); s += 2.0)
        {
            const int st = netStructure(ur, s);
            if (st != NetTrench && st != NetCovered)
                continue;
            a = std::min(a, s);
            b = std::max(b, s);
        }
        if (a < b)
        {
            a = std::max(0.0, a - 6.0);
            b = std::min(rr.length(), b + 6.0);
            const int ns = static_cast<int>((b - a) / 2.5), nl = 10;
            for (int side : { -1, 1 })
            {
                const vtkIdType base = pts->GetNumberOfPoints();
                for (int i = 0; i <= ns; ++i)
                    for (int k = 0; k <= nl; ++k)
                    {
                        const double ss = a + (b - a) * i / ns;
                        const double lat = side * (netWallLat(ur) + 0.3 + 24.0 * k / nl);
                        double x, y;
                        rr.point(ss, lat, x, y);
                        const double h = terrainHeight(x, y);
                        pts->InsertNextPoint(x, y, gz - 0.10 + h);
                        const double sx = (terrainHeight(x + 2.0, y) - terrainHeight(x - 2.0, y)) / 4.0;
                        const double sy = (terrainHeight(x, y + 2.0) - terrainHeight(x, y - 2.0)) / 4.0;
                        unsigned char rgb[3];
                        colorAt(x, y, h, std::sqrt(sx * sx + sy * sy), rgb);
                        colors->InsertNextTypedTuple(rgb);
                    }
                for (int i = 0; i < ns; ++i)
                    for (int k = 0; k < nl; ++k)
                    {
                        const vtkIdType p0 = base + static_cast<vtkIdType>(i) * (nl + 1) + k;
                        const vtkIdType q[4] = { p0, p0 + 1, p0 + nl + 2, p0 + nl + 1 };
                        quads->InsertNextCell(4, q);
                    }
            }
        }
    }

    // LiDAR height field (height above the ground plane; flat ground / lake = 0)
    m_heightField.x0 = x0;
    m_heightField.y0 = y0;
    m_heightField.step = step;
    m_heightField.nx = nx;
    m_heightField.ny = ny;
    m_heightField.h.resize(H.size());
    m_heightField.maxH = 0.0f;
    for (size_t k = 0; k < H.size(); ++k)
    {
        m_heightField.h[k] = static_cast<float>(std::max(0.0, H[k] - 0.06));
        m_heightField.maxH = std::max(m_heightField.maxH, m_heightField.h[k]);
    }

    vtkNew<vtkPolyData> pd;
    pd->SetPoints(pts);
    pd->SetPolys(quads);
    pd->GetPointData()->SetScalars(colors);
    vtkNew<vtkAppendPolyData> withHill;
    withHill->AddInputData(pd);
    if (m_tunnelHill && m_tunnelHill->GetNumberOfPoints() > 0)
        withHill->AddInputData(m_tunnelHill);
    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputConnection(withHill->GetOutputPort());
    normals->SplittingOff();
    normals->Update();
    m_terrain = vtkSmartPointer<vtkPolyData>::New();
    m_terrain->ShallowCopy(normals->GetOutput());


    // lake surface
    vtkNew<vtkPoints> wp;
    vtkNew<vtkCellArray> wc;
    const int n = 96;
    const double k = std::sqrt(kWaterE);
    wp->InsertNextPoint(kLakeX, kLakeY, gz - 0.3);
    for (int i = 0; i < n; ++i)
    {
        const double a = 2.0 * kPi * i / n;
        wp->InsertNextPoint(kLakeX + k * kLakeRx * std::cos(a), kLakeY + k * kLakeRy * std::sin(a), gz - 0.3);
    }
    for (int i = 0; i < n; ++i)
    {
        const vtkIdType t[3] = { 0, 1 + i, 1 + (i + 1) % n };
        wc->InsertNextCell(3, t);
    }
    m_water = vtkSmartPointer<vtkPolyData>::New();
    m_water->SetPoints(wp);
    m_water->SetPolys(wc);
}

int World::groundMaterial(double x, double y) const
{
    const RoadModel& r = m_road;
    double s, lat;
    if (!r.project(x, y, s, lat))
        return Palette::Grass;
    const double alat = std::abs(lat);
    const auto zone = r.zoneAt(s);
    for (const auto& p : m_paved)
        if (s >= p.s0 && s <= p.s1 && lat >= p.lat0 && lat <= p.lat1)
            return p.material;
    if (m_netReady)
    {
        const int m = netMask(x, y);
        if (m == 255)
            return Palette::Asphalt;
        if (m >= 2)
        {
            const NetRoad& nr = m_net[m - 2];
            double rs, rl;
            if (nr.road.project(x, y, rs, rl) && std::abs(rl) <= nr.road.halfWidth + 0.4)
            {
                const double a = std::abs(rl), hw = nr.road.halfWidth;
                if (a > 0.06 && a < 0.20)
                    return Palette::LaneYellow;
                for (int k = 1; k < nr.road.lanes; ++k)
                    if (std::abs(a - k * nr.road.laneWidth) < 0.07 && std::fmod(rs, 7.0) < 3.0)
                        return Palette::LaneWhite;
                if (a > hw + 0.08 && a < hw + 0.22)
                    return Palette::LaneWhite;
                return Palette::Asphalt;
            }
        }
    }

    if (zone == RoadModel::Zone::City)
    {
        for (double si : m_intersections)
        {
            const double ds = s - si;
            if (std::abs(ds) > kStopLine + 1.0)
                continue;
            for (double side : { 1.0, -1.0 })
            {
                if (std::abs(ds - side * kCrosswalk) < 1.5 && alat < 8.85)
                {
                    const double k = std::round((lat + 8.55) / 0.9);
                    if (k >= 0 && k <= 19 && std::abs(lat - (-8.55 + 0.9 * k)) < 0.25)
                        return Palette::LaneWhite;
                }
            }
            if (std::abs(ds + kStopLine) < 0.25 && lat > 0.3 && lat < r.halfWidth)
                return Palette::LaneWhite;
            if (std::abs(ds - kStopLine) < 0.25 && lat < -0.3 && lat > -r.halfWidth)
                return Palette::LaneWhite;
            if (std::abs(ds) < kCrossHalf && alat < kCrossLength)
            {
                if (alat > kWalkIn && alat < kBackLat - kCrossHalf)
                {
                    const double ads = std::abs(ds);
                    if (std::abs(ads - 0.13) < 0.07)
                        return Palette::LaneYellow;
                    if (std::abs(ads - 3.0) < 0.07 && std::fmod(alat - kWalkIn, 7.0) < 3.0)
                        return Palette::LaneWhite;
                }
                return Palette::Asphalt;
            }
        }
    }

    if (alat <= r.halfWidth + 0.4)
    {
        if (!nearIntersection(s, kMarkGap))
        {
            if (std::abs(alat - 0.13) < 0.07)                                   // double yellow
                return Palette::LaneYellow;
            for (int k = 1; k < RoadModel::kLanes; ++k)                           // dashed dividers
                if (std::abs(alat - k * r.laneWidth) < 0.07 && std::fmod(s, 7.0) < 3.0)
                    return Palette::LaneWhite;
            if (std::abs(alat - (r.halfWidth + 0.15)) < 0.07)                   // edge lines
                return Palette::LaneWhite;
        }
        return Palette::Asphalt;
    }

    if (zone == RoadModel::Zone::City && !nearIntersection(s, kCrossHalf) && alat >= kWalkIn && alat <= kWalkOut)
        return Palette::Sidewalk;
    if (zone == RoadModel::Zone::Suburb && alat >= kWalkIn && alat <= kSuburbWalkOut)
        return Palette::Sidewalk;
    if (alat > 30.0 && lakeE(x, y) < kWaterE)
        return Palette::Water;
    return Palette::Grass;
}

bool World::footprintHitsBox(const VehicleModel& v, double cx, double cy, double heading,
                             double halfL, double halfW, double contact[3], const Footprint* fpIn)
{
    const Footprint fp = fpIn ? *fpIn : Footprint{};
    const double c = std::cos(heading), s = std::sin(heading);
    auto insideBox = [&](const double w[3]) {
        const double dx = w[0] - cx, dy = w[1] - cy;
        return std::abs(c * dx + s * dy) < halfL && std::abs(-s * dx + c * dy) < halfW;
    };

    if (forPerimeter(fp, 48, [&](const double l[3]) {
            double w[3];
            v.localToWorld(l, w);
            if (!insideBox(w))
                return false;
            std::copy(w, w + 3, contact);
            return true;
        }))
        return true;

    for (int k = 0; k < 4; ++k)
    {
        const double a = (k == 0 || k == 3) ? -halfL : halfL;
        const double b = (k < 2) ? -halfW : halfW;
        const double w[3] = { cx + c * a - s * b, cy + s * a + c * b, v.z() + kLzMid };
        double l[3];
        v.worldToLocal(w, l);
        if (l[0] > fp.x0 && l[0] < fp.x1 && l[1] > fp.y0 && l[1] < fp.y1)
        {
            std::copy(w, w + 3, contact);
            return true;
        }
    }
    return false;
}

bool World::checkCollision(const VehicleModel& v, double contact[3], ObstacleKind* kind, const Footprint* fpIn,
                           int* prop, int surface, int netRoad) const
{
    const Footprint fp = fpIn ? *fpIn : Footprint{};
    if (prop) *prop = -1;
    double centerLocal[3] = { 2.0, 0.5, 0.0 }, center[3];
    v.localToWorld(centerLocal, center);
    // objects standing on the ground: not when the car is on the viaduct or down in the underpass
    const bool atGround = std::abs(v.z()) < 2.0;

    for (const auto& c : m_circles)
    {
        if (!atGround)
            break;
        if (std::abs(c.x - center[0]) > 4.0 || std::abs(c.y - center[1]) > 4.0)
            continue;
        if (c.prop >= 0 && m_props[c.prop].knocked)
            continue;

        const double w[3] = { c.x, c.y, v.z() };
        double l[3];
        v.worldToLocal(w, l);
        const double qx = std::clamp(l[0], fp.x0, fp.x1);
        const double qy = std::clamp(l[1], fp.y0, fp.y1);
        const double dx = l[0] - qx, dy = l[1] - qy;
        if (dx * dx + dy * dy < c.r * c.r)
        {
            const double q[3] = { qx, qy, 0.6 };
            v.localToWorld(q, contact);
            if (kind) *kind = c.kind;
            if (prop) *prop = c.prop;
            return true;
        }
    }

    for (const auto& b : m_boxes)
    {
        if (!atGround)
            break;
        const double reach = std::max(b.hx, b.hy) + 4.5;
        if (std::abs(b.cx - center[0]) > reach || std::abs(b.cy - center[1]) > reach)
            continue;
        if (b.prop >= 0 && m_props[b.prop].knocked)
            continue;
        if (footprintHitsBox(v, b.cx, b.cy, b.yaw, b.hx, b.hy, contact, &fp))
        {
            if (kind) *kind = b.kind;
            if (prop) *prop = b.prop;
            return true;
        }
    }

    // street network: guard rails, viaduct parapets, retaining walls (on the street or right beside it)
    if (m_netReady)
    {
        for (size_t ri = 0; ri < m_net.size(); ++ri)
        {
            const NetRoad& nr = m_net[ri];
            const int road = static_cast<int>(ri);
            double rs, rl;
            if (!nr.road.project(center[0], center[1], rs, rl) || std::abs(rl) > netWallLat(road) + 4.0)
                continue;
            const double zr = nr.road.elevation(rs);
            const bool onIt = surface == 1 && netRoad == road;
            const bool beside = surface == 3 && std::abs(v.z() - zr) < 2.5;
            const double hw = nr.road.halfWidth;
            if ((onIt || beside) && std::abs(rl) > hw - 4.0)
            {
                ObstacleKind k = ObstacleKind::Guardrail;
                const bool hit = forPerimeter(fp, 32, [&](const double l[3]) {
                    double w[3], ps, pl;
                    v.localToWorld(l, w);
                    if (!nr.road.project(w[0], w[1], ps, pl))
                        return false;
                    const int side = pl > 0 ? 1 : -1;
                    const int st = netStructure(road, ps);
                    const double railLat = st == NetGraded ? hw + 1.0 : st == NetViaduct ? hw + 1.15 : netWallLat(road) - 0.3;
                    const double in = onIt ? railLat - 0.05 : railLat - 0.6;
                    if (std::abs(pl) > in && std::abs(pl) < railLat + 0.6 && netRailAt(road, ps, side))
                    {
                        k = st == NetGraded ? ObstacleKind::Guardrail : ObstacleKind::Wall;
                        std::copy(w, w + 3, contact);
                        return true;
                    }
                    return false;
                });
                if (hit)
                {
                    if (kind) *kind = k;
                    return true;
                }
            }
            // the parapet of the open trench, seen from the ground next to it
            if (surface != 1 && atGround && netStructure(road, rs) == NetTrench && std::abs(rl) > netWallLat(road) - 3.0)
            {
                const bool hit = forPerimeter(fp, 32, [&](const double l[3]) {
                    double w[3], ps, pl;
                    v.localToWorld(l, w);
                    if (!nr.road.project(w[0], w[1], ps, pl) || netStructure(road, ps) != NetTrench)
                        return false;
                    if (std::abs(pl) < netWallLat(road) + 0.3)
                    {
                        std::copy(w, w + 3, contact);
                        return true;
                    }
                    return false;
                });
                if (hit)
                {
                    if (kind) *kind = ObstacleKind::Wall;
                    return true;
                }
            }
        }
    }

    // continuous barriers along the main road: guard rails and the median barrier
    if (surface == 1 || surface == 2)
        return false;
    double s, lat;
    if (m_road.project(center[0], center[1], s, lat))
    {
        const double railLat = m_road.halfWidth + 1.0;
        const bool nearRail = std::abs(lat) > railLat - 3.0 && std::abs(lat) < railLat + 3.0;
        const bool nearMedian = std::abs(lat) < 3.0;
        if ((nearRail && (guardrailAt(s, 1) || guardrailAt(s, -1))) || (nearMedian && medianBarrierAt(s)))
        {
            ObstacleKind k = ObstacleKind::Guardrail;
            const bool hit = forPerimeter(fp, 32, [&](const double l[3]) {
                double w[3], ps, pl;
                v.localToWorld(l, w);
                if (!m_road.project(w[0], w[1], ps, pl))
                    return false;
                const int side = pl > 0 ? 1 : -1;
                if (std::abs(pl) > railLat - 0.05 && std::abs(pl) < railLat + 0.6 && guardrailAt(ps, side))
                {
                    k = inTunnel(ps) ? ObstacleKind::TunnelWall : ObstacleKind::Guardrail;
                    std::copy(w, w + 3, contact);
                    return true;
                }
                if (std::abs(pl) < 0.25 && medianBarrierAt(ps))
                {
                    k = ObstacleKind::Wall;
                    std::copy(w, w + 3, contact);
                    return true;
                }
                return false;
            });
            if (hit)
            {
                if (kind) *kind = k;
                return true;
            }
        }
    }
    return false;
}

// Highway tunnel: two bores (one per direction) through a hill, with tiled walls, a vault, light strips
// and concrete portals. Everything follows the road frame (the tunnel lies in the S-bend).
vtkSmartPointer<vtkPolyData> World::buildTunnel()
{
    const RoadModel& r = m_road;
    const double gz = r.groundZ;
    const double hw = r.halfWidth;
    const double t0 = m_tunnelFrom, t1 = m_tunnelTo;
    const double wallIn = hw + 1.0;     // inner face of the side walls (the guard rail line)
    const double cutHalf = hw + 6.0;    // open cut in front of the portals
    const double wing = 32.0;           // length of the cuts
    const double H = 10.5;              // earth over the road
    const double zHill = gz - 0.06;     // same base as the terrain

    auto outer = [&](double lat) { return lat < 0.0 ? hw + 62.0 : hw + 38.0; };   // mountains outside (lat < 0)
    auto prof = [&](double s, double lat) {
        const double a = std::abs(lat);
        double x, y;
        r.point(s, lat, x, y);
        double h = H * (1.0 - smoothstep(hw + 4.0, outer(lat), a));
        h += 5.0 * (fbm(x * 0.03 + 3.0, y * 0.03 - 1.0) - 0.5) * smoothstep(hw + 6.0, hw + 20.0, a) *
             (1.0 - smoothstep(outer(lat) - 14.0, outer(lat), a));
        if (s < t0) h *= smoothstep(t0 - wing, t0, s);
        if (s > t1) h *= 1.0 - smoothstep(t1, t1 + wing, s);
        return std::max(h, terrainHeight(x, y));
    };
    auto linspace = [](double a, double b, double step) {
        std::vector<double> v;
        const int n = std::max(1, static_cast<int>(std::ceil(std::abs(b - a) / step)));
        for (int i = 0; i <= n; ++i)
            v.push_back(a + (b - a) * i / n);
        return v;
    };
    auto concat = [](std::vector<double> a, const std::vector<double>& b) {
        a.insert(a.end(), b.begin() + 1, b.end());
        return a;
    };

    // ---------------- hill (vertex-colored like the terrain)
    vtkNew<vtkPoints> hp;
    hp->SetDataTypeToDouble();
    vtkNew<vtkCellArray> hq;
    vtkNew<vtkUnsignedCharArray> hc;
    hc->SetName("Colors");
    hc->SetNumberOfComponents(3);
    auto patch = [&](double sA, double sB, const std::vector<double>& lats) {
        const int ns = std::max(2, static_cast<int>(std::ceil((sB - sA) / 4.0)));
        const vtkIdType base = hp->GetNumberOfPoints();
        const int nl = static_cast<int>(lats.size());
        for (int i = 0; i <= ns; ++i)
        {
            const double s = sA + (sB - sA) * i / ns;
            for (double lat : lats)
            {
                double x, y;
                r.point(s, lat, x, y);
                const double h = prof(s, lat);
                hp->InsertNextPoint(x, y, zHill + h);
                const double n = fbm(x * 0.02, y * 0.02);
                const double c[3] = { 0.27 + 0.10 * n, 0.40 + 0.12 * n, 0.18 + 0.05 * n };
                const double dark[3] = { 0.13, 0.25, 0.12 };
                const double f = 0.45 * smoothstep(3.0, 9.0, h);
                unsigned char rgb[3];
                for (int k = 0; k < 3; ++k)
                    rgb[k] = static_cast<unsigned char>(std::clamp(c[k] + (dark[k] - c[k]) * f, 0.0, 1.0) * 255.0);
                hc->InsertNextTypedTuple(rgb);
            }
        }
        for (int i = 0; i < ns; ++i)
            for (int k = 0; k + 1 < nl; ++k)
            {
                const vtkIdType a = base + static_cast<vtkIdType>(i) * nl + k;
                const vtkIdType q[4] = { a, a + 1, a + nl + 1, a + nl };
                hq->InsertNextCell(4, q);
            }
    };
    const std::vector<double> latsNeg = linspace(-outer(-1.0), -cutHalf, 3.0);
    const std::vector<double> latsPos = linspace(cutHalf, outer(1.0), 3.0);
    patch(t0, t1, concat(concat(latsNeg, linspace(-cutHalf, cutHalf, 2.0)), latsPos));
    for (const auto* lats : { &latsNeg, &latsPos })
    {
        patch(t0 - wing, t0, *lats);
        patch(t1, t1 + wing, *lats);
    }
    auto hill = vtkSmartPointer<vtkPolyData>::New();
    hill->SetPoints(hp);
    hill->SetPolys(hq);
    hill->GetPointData()->SetScalars(hc);

    // ---------------- concrete: portals (head walls with the two openings) and the cut walls
    vtkNew<vtkPoints> cp;
    vtkNew<vtkCellArray> cq;
    auto quad = [&](const double a[3], const double b[3], const double c[3], const double d[3]) {
        const vtkIdType ids[4] = { cp->InsertNextPoint(a), cp->InsertNextPoint(b), cp->InsertNextPoint(c),
                                   cp->InsertNextPoint(d) };
        cq->InsertNextCell(4, ids);
    };
    for (double sp : { t0, t1 })
    {
        // regions across the portal: solid | opening (vault) | center wall | opening | solid
        const std::pair<double, double> regions[] = {
            { -cutHalf, -wallIn }, { -wallIn, -0.25 }, { -0.25, 0.25 }, { 0.25, wallIn }, { wallIn, cutHalf } };
        for (const auto& [la, lb] : regions)
        {
            const double mid = std::abs(0.5 * (la + lb));
            const bool opening = mid > 0.25 && mid < wallIn;
            const std::vector<double> lats = linspace(la, lb, 0.3);
            for (size_t i = 0; i + 1 < lats.size(); ++i)
            {
                double p0[3], p1[3], p2[3], p3[3];
                r.point(sp, lats[i], p0[0], p0[1]);
                r.point(sp, lats[i + 1], p1[0], p1[1]);
                p2[0] = p1[0]; p2[1] = p1[1];
                p3[0] = p0[0]; p3[1] = p0[1];
                p0[2] = gz + (opening ? tunnelCeiling(lats[i], hw) : 0.0);
                p1[2] = gz + (opening ? tunnelCeiling(lats[i + 1], hw) : 0.0);
                p2[2] = zHill + prof(sp, lats[i + 1]);
                p3[2] = zHill + prof(sp, lats[i]);
                quad(p0, p1, p2, p3);
            }
        }
    }
    for (double side : { -1.0, 1.0 })
        for (const auto& [sa, sb] : { std::pair<double, double>{ t0 - wing, t0 }, std::pair<double, double>{ t1, t1 + wing } })
        {
            const std::vector<double> ss = linspace(sa, sb, 2.0);
            for (size_t i = 0; i + 1 < ss.size(); ++i)
            {
                double p0[3], p1[3], p2[3], p3[3];
                r.point(ss[i], side * cutHalf, p0[0], p0[1]);
                r.point(ss[i + 1], side * cutHalf, p1[0], p1[1]);
                p2[0] = p1[0]; p2[1] = p1[1];
                p3[0] = p0[0]; p3[1] = p0[1];
                p0[2] = p1[2] = gz - 0.06;
                p2[2] = zHill + prof(ss[i + 1], side * cutHalf);
                p3[2] = zHill + prof(ss[i], side * cutHalf);
                quad(p0, p1, p2, p3);
            }
        }
    vtkNew<vtkPolyData> concrete;
    concrete->SetPoints(cp);
    concrete->SetPolys(cq);

    // ---------------- vault (one per bore)
    vtkNew<vtkPoints> vp;
    vtkNew<vtkCellArray> vq;
    for (double side : { -1.0, 1.0 })
    {
        const std::vector<double> lats = linspace(side * 0.25, side * wallIn, 0.6);
        const std::vector<double> ss = linspace(t0, t1, 3.0);
        const vtkIdType base = vp->GetNumberOfPoints();
        const int nl = static_cast<int>(lats.size());
        for (double s : ss)
            for (double lat : lats)
            {
                double x, y;
                r.point(s, lat, x, y);
                vp->InsertNextPoint(x, y, gz + tunnelCeiling(lat, hw));
            }
        for (size_t i = 0; i + 1 < ss.size(); ++i)
            for (int k = 0; k + 1 < nl; ++k)
            {
                const vtkIdType a = base + static_cast<vtkIdType>(i) * nl + k;
                const vtkIdType q[4] = { a, a + 1, a + nl + 1, a + nl };
                vq->InsertNextCell(4, q);
            }
    }
    vtkNew<vtkPolyData> vault;
    vault->SetPoints(vp);
    vault->SetPolys(vq);

    std::vector<Grid> parts = { SceneBuilder::fromPolyData(concrete, Palette::Concrete),
                                SceneBuilder::fromPolyData(vault, Palette::Concrete) };
    for (double side : { -1.0, 1.0 })
    {
        // tiled side walls with a green band at eye height
        parts.push_back(SceneBuilder::wallAlong(r, t0, t1, side * (wallIn + 0.3), 0.6, gz, gz + 4.8, 3.0,
                                                Palette::TunnelTile));
        parts.push_back(SceneBuilder::wallAlong(r, t0, t1, side * (wallIn - 0.01), 0.02, gz + 1.0, gz + 1.3, 3.0,
                                                Palette::SignGreen));
    }
    // center wall between the bores (on top of the median barrier)
    // (starts on top of the jersey barrier: coplanar overlapping faces would z-fight)
    parts.push_back(SceneBuilder::wallAlong(r, t0, t1, 0.0, 0.5, gz + 0.85, gz + 4.8, 3.0, Palette::TunnelTile));
    m_pidGrids[PidTunnel] = SceneBuilder::merge(parts);

    // light strips under the vault, two rows per bore
    std::vector<Grid> lights;
    for (double s = t0 + 3.0; s < t1 - 2.0; s += 6.0)
        for (double side : { -1.0, 1.0 })
            for (double lat : { 3.0, 6.0 })
            {
                double x, y;
                r.point(s, side * lat, x, y);
                const double z1 = gz + tunnelCeiling(lat, hw) - 0.03;
                lights.push_back(SceneBuilder::orientedBlock(x, y, r.heading(s) * kRad2Deg, 2.6, 0.22, z1 - 0.08, z1,
                                                             1, 1, 1, [](int, int, int, double, double, double) {
                                                                 return int(Palette::Lamp);
                                                             }));
            }
    m_tunnelLights = SceneBuilder::merge(lights);
    return hill;
}

// =====================================================================
// City street network (besides the main loop): junctions + curved streets,
// a viaduct over the highway, an underpass under the suburb road, a mountain road
// =====================================================================
namespace
{
struct NodeDef { double x, y; bool cityLevel; const char* nameKo; };
struct RoadDef
{
    int a, b, lanes;
    double laneW, limitKmh;
    const char* nameKo;
    std::vector<std::array<double, 2>> mid;   // control points between the two junctions
};

// junctions
const NodeDef kNodes[] = {
    { -35.0, 150.0, true,  u8"도심 북문 교차로" },     // 0: connector from the cross street at x = -35
    { -165.0, 150.0, true, u8"도심 서문 교차로" },     // 1: connector from the cross street at x = -165
    { -30.0, 455.0, false, u8"호수 북동 교차로" },     // 2
    { -215.0, 470.0, false, u8"호수 북쪽 교차로" },    // 3
    { -395.0, 330.0, false, u8"호수 서쪽 교차로" },    // 4
    { 15.0, 305.0, false,  u8"공원 동쪽 교차로" },     // 5
    { -25.0, 710.0, false, u8"북쪽 산 삼거리" },        // 6 (outside the loop, beyond the viaduct)
    { 315.0, 300.0, false, u8"동쪽 아파트 삼거리" },    // 7 (outside the loop, beyond the underpass)
    { -300.0, 700.0, false, u8"전망대 (막다른 길)" },   // 8 dead end
};

const RoadDef kRoads[] = {
    { 0, 2, 2, 3.0, 50.0, u8"호숫가로",       { { -28.0, 230.0 }, { -45.0, 310.0 }, { -38.0, 390.0 } } },
    { 2, 3, 2, 3.0, 50.0, u8"호수 북로",      { { -90.0, 482.0 }, { -160.0, 488.0 } } },
    { 3, 4, 2, 3.0, 50.0, u8"서쪽 숲길",      { { -300.0, 452.0 }, { -370.0, 402.0 } } },
    { 4, 1, 2, 3.0, 50.0, u8"호수 남서로",    { { -382.0, 240.0 }, { -300.0, 182.0 } } },
    { 1, 0, 2, 3.0, 50.0, u8"공원 남로",      { { -100.0, 157.0 } } },
    { 2, 5, 2, 3.0, 50.0, u8"공원 동로",      { { 0.0, 420.0 }, { 14.0, 362.0 } } },
    { 5, 0, 2, 3.0, 50.0, u8"공원 사선로",    { { 12.0, 240.0 }, { -10.0, 180.0 } } },
    { 2, 6, 2, 3.0, 60.0, u8"북부 고가도로",  { { -38.0, 520.0 }, { -40.0, 600.0 }, { -35.0, 660.0 } } },
    { 5, 7, 2, 3.0, 60.0, u8"동부 지하차도로", { { 80.0, 302.0 }, { 140.0, 300.0 }, { 220.0, 305.0 } } },
    { 6, 7, 1, 3.5, 60.0, u8"산악도로",       { { 90.0, 760.0 }, { 220.0, 742.0 }, { 305.0, 640.0 }, { 330.0, 480.0 } } },
    { 6, 8, 1, 3.5, 40.0, u8"전망대길",       { { -110.0, 740.0 }, { -210.0, 735.0 } } },
};
}

int World::netStructure(int road, double s) const
{
    const NetRoad& nr = m_net[road];
    if (nr.st.empty())
        return NetGraded;
    const int n = static_cast<int>(nr.st.size());
    const int i = std::clamp(static_cast<int>(std::lround(nr.road.wrap(s) / nr.road.elevationStep())), 0, n - 1);
    return nr.st[i];
}

bool World::netRailAt(int road, double s, int side) const
{
    (void)side;
    const NetRoad& nr = m_net[road];
    const double L = nr.road.length();
    // open near the junctions (streets fan out there)
    if (s < m_nodes[nr.nodeA].radius + 25.0 || s > L - m_nodes[nr.nodeB].radius - 25.0)
        return false;
    const int st = netStructure(road, s);
    if (st != NetGraded)
        return true;
    // graded: guard rails only where the road runs on a high embankment or in a deep cut
    const int n = static_cast<int>(nr.nat.size());
    const int i = std::clamp(static_cast<int>(std::lround(s / nr.road.elevationStep())), 0, n - 1);
    return std::abs(nr.road.elevation(s) - std::max(0.0, nr.nat[i])) > 2.0;
}

bool World::nearNet(double x, double y, double margin) const
{
    if (!m_netReady)
        return false;
    for (const auto& nd : m_nodes)
        if (std::hypot(x - nd.x, y - nd.y) < nd.radius + margin)
            return true;
    for (const auto& nr : m_net)
    {
        double s, lat;
        if (nr.road.project(x, y, s, lat) && std::abs(lat) < nr.road.halfWidth + margin)
            return true;
    }
    return false;
}

int World::netMask(double x, double y) const
{
    const int i = static_cast<int>(std::floor(x - m_maskX0)), j = static_cast<int>(std::floor(y - m_maskY0));
    if (i < 0 || j < 0 || i >= m_maskNx || j >= m_maskNy)
        return 0;
    return m_netMask[static_cast<size_t>(j) * m_maskNx + i];
}

bool World::groundHole(double x, double y) const
{
    return netMask(x, y) == 1;   // trench and covered part of the underpass
}

bool World::netLocate(double x, double y, double zPrev, NetPos& out) const
{
    // every network surface under (x, y); the one closest to the current height wins, and among
    // surfaces at about the same height (two streets leaving a junction) the one whose center is closest
    bool found = false;
    double best = 1e18, bestLat = 1e18;
    for (size_t k = 0; k < m_nodes.size(); ++k)
    {
        const NetNode& nd = m_nodes[k];
        if (std::hypot(x - nd.x, y - nd.y) > nd.radius)
            continue;
        const double d = std::abs(nd.z - zPrev);
        if (d < best)
        {
            best = d;
            bestLat = 0.0;
            out = NetPos{};
            out.node = static_cast<int>(k);
            out.z = nd.z;
            found = true;
        }
    }
    for (size_t r = 0; r < m_net.size(); ++r)
    {
        const NetRoad& nr = m_net[r];
        double s, lat;
        if (!nr.road.project(x, y, s, lat))
            continue;
        const int st = netStructure(static_cast<int>(r), s);
        const double reach = st == NetGraded ? nr.road.halfWidth + 1.2 : nr.road.halfWidth + 1.5;
        if (std::abs(lat) > reach)
            continue;
        const double z = nr.road.elevation(s);
        const double d = std::abs(z - zPrev);
        const bool better = d < best - 0.5 || (d < best + 0.5 && std::abs(lat) < bestLat);
        if (better)
        {
            best = std::min(best, d);
            bestLat = std::abs(lat);
            out = NetPos{};
            out.road = static_cast<int>(r);
            out.s = s;
            out.lat = lat;
            out.z = z;
            found = true;
        }
    }
    return found;
}

void World::buildNetwork()
{
    m_netReady = false;
    m_net.clear();
    m_nodes.clear();
    m_viaductRoad = m_underRoad = -1;
    for (const auto& nd : kNodes)
    {
        NetNode n;
        n.x = nd.x;
        n.y = nd.y;
        n.nameKo = nd.nameKo;
        // junction level: street level downtown, else the smoothed ground around it
        if (!nd.cityLevel)
        {
            double sum = 0.0;
            int cnt = 0;
            for (int i = -2; i <= 2; ++i)
                for (int j = -2; j <= 2; ++j)
                {
                    sum += std::max(0.0, naturalHeight(nd.x + 10.0 * i, nd.y + 10.0 * j));
                    ++cnt;
                }
            n.z = sum / cnt;
        }
        m_nodes.push_back(n);
    }
    for (const auto& rd : kRoads)
    {
        std::vector<std::array<double, 2>> pts;
        pts.push_back({ kNodes[rd.a].x, kNodes[rd.a].y });
        for (const auto& p : rd.mid)
            pts.push_back(p);
        pts.push_back({ kNodes[rd.b].x, kNodes[rd.b].y });
        NetRoad nr;
        nr.road = RoadModel::spline(pts, rd.lanes, rd.laneW);
        nr.nodeA = rd.a;
        nr.nodeB = rd.b;
        nr.limit = rd.limitKmh / 3.6;
        nr.nameKo = rd.nameKo;
        m_nodes[rd.a].ends.push_back({ static_cast<int>(m_net.size()), 0 });
        m_nodes[rd.b].ends.push_back({ static_cast<int>(m_net.size()), 1 });
        m_net.push_back(std::move(nr));
    }
    for (auto& nd : m_nodes)
    {
        double hw = 0.0;
        for (const auto& e : nd.ends)
            hw = std::max(hw, m_net[e.first].road.halfWidth);
        nd.radius = hw + 4.0;
    }

    // crossings with the main loop (over the highway / under the suburb road), found first so that the
    // junction levels can be made reachable within the grade limit
    std::vector<double> crossH(m_net.size(), -1.0), crossS(m_net.size(), -1.0);
    for (size_t r = 0; r < m_net.size(); ++r)
    {
        const RoadModel& road = m_net[r].road;
        double bestH = 1e9, bestS = 1e9;
        for (double s = 0.0; s <= road.length(); s += 0.5)
        {
            double x, y, ms, ml;
            road.point(s, 0.0, x, y);
            if (!m_road.project(x, y, ms, ml) || std::abs(ml) > 3.0)
                continue;
            const auto zone = m_road.zoneAt(ms);
            if (zone == RoadModel::Zone::Highway && std::abs(ml) < bestH)
            {
                bestH = std::abs(ml);
                crossH[r] = s;
            }
            if ((zone == RoadModel::Zone::Suburb || zone == RoadModel::Zone::Corner) && std::abs(ml) < bestS)
            {
                bestS = std::abs(ml);
                crossS[r] = s;
            }
        }
    }
    {
        const double gReach = 0.062;   // a little below the grade limit: room for the vertical curves
        for (int pass = 0; pass < 4; ++pass)
            for (size_t r = 0; r < m_net.size(); ++r)
            {
                const NetRoad& nr = m_net[r];
                const double L = nr.road.length();
                for (int k = 0; k < 2; ++k)
                {
                    const double sc = k == 0 ? crossH[r] : crossS[r];
                    if (sc < 0.0)
                        continue;
                    const double zc = k == 0 ? 7.6 : -6.3, half = k == 0 ? 24.0 : 30.0;
                    for (int e = 0; e < 2; ++e)
                    {
                        NetNode& nd = m_nodes[e == 0 ? nr.nodeA : nr.nodeB];
                        const double d = (e == 0 ? sc - half : L - sc - half) - (nd.radius + 4.0);
                        nd.z = std::clamp(nd.z, zc - gReach * d, zc + gReach * d);
                    }
                }
                // and between the two junctions of every street
                NetNode& a = m_nodes[nr.nodeA];
                NetNode& b = m_nodes[nr.nodeB];
                const double d = L - a.radius - b.radius - 8.0;
                if (std::abs(a.z - b.z) > gReach * d)
                {
                    const double mid = 0.5 * (a.z + b.z), half = 0.5 * gReach * d;
                    a.z = std::clamp(a.z, mid - half, mid + half);
                    b.z = std::clamp(b.z, mid - half, mid + half);
                }
            }
    }

    // elevation profile of every street
    for (size_t r = 0; r < m_net.size(); ++r)
    {
        NetRoad& nr = m_net[r];
        RoadModel& road = nr.road;
        const double L = road.length();
        const int n = std::max(2, static_cast<int>(std::round(L / 5.0)) + 1);
        const double step = L / (n - 1);
        nr.nat.assign(n, 0.0);
        std::vector<double> sm(n), z(n, 0.0);
        std::vector<char> fixed(n, 0);
        for (int i = 0; i < n; ++i)
        {
            double x, y;
            road.point(i * step, 0.0, x, y);
            nr.nat[i] = naturalHeight(x, y);
        }
        for (int i = 0; i < n; ++i)
        {
            double sum = 0.0;
            int cnt = 0;
            for (int j = std::max(0, i - 12); j <= std::min(n - 1, i + 12); ++j)
            {
                sum += std::max(0.0, nr.nat[j]);
                ++cnt;
            }
            sm[i] = sum / cnt;
        }
        auto fix = [&](double sc, double half, double zf) {
            for (int i = 0; i < n; ++i)
                if (std::abs(i * step - sc) <= half)
                {
                    fixed[i] = 1;
                    z[i] = zf;
                }
        };
        fix(0.0, m_nodes[nr.nodeA].radius + 4.0, m_nodes[nr.nodeA].z);
        fix(L, m_nodes[nr.nodeB].radius + 4.0, m_nodes[nr.nodeB].z);
        // crossings with the main loop: over the highway, under the suburb road
        {
            const double sH = crossH[r], sS = crossS[r];
            if (sH >= 0.0)
            {
                fix(sH, 24.0, 7.6);   // 5.5 m clearance + 1.1 m deck
                m_viaductRoad = static_cast<int>(r);
                m_viaductS = sH;
            }
            if (sS >= 0.0)
            {
                fix(sS, 30.0, -6.3);   // 5.2 m clearance under the 1 m cover slab (60 m flat bottom)
                m_underRoad = static_cast<int>(r);
                m_underS = sS;
            }
        }
        // grade limit (7 %, like real urban ramps): envelopes from the fixed points, neighbour passes, gentle curves
        const double g = 0.07;
        for (int i = 0; i < n; ++i)
        {
            if (fixed[i])
                continue;
            double lo = -1e9, hi = 1e9;
            for (int j = 0; j < n; ++j)
                if (fixed[j])
                {
                    const double d = std::abs(i - j) * step;
                    lo = std::max(lo, z[j] - g * d);
                    hi = std::min(hi, z[j] + g * d);
                }
            z[i] = std::clamp(sm[i], lo, hi);
        }
        for (int pass = 0; pass < 6; ++pass)
        {
            for (int i = 1; i < n; ++i)
                if (!fixed[i])
                    z[i] = std::clamp(z[i], z[i - 1] - g * step, z[i - 1] + g * step);
            for (int i = n - 2; i >= 0; --i)
                if (!fixed[i])
                    z[i] = std::clamp(z[i], z[i + 1] - g * step, z[i + 1] + g * step);
        }
        for (int pass = 0; pass < 3; ++pass)
        {
            std::vector<double> t = z;
            for (int i = 1; i + 1 < n; ++i)
                if (!fixed[i])
                    t[i] = 0.25 * z[i - 1] + 0.5 * z[i] + 0.25 * z[i + 1];
            z = t;
        }
        road.setElevation(z, step);
        nr.st.assign(n, NetGraded);
        for (int i = 0; i < n; ++i)
        {
            const double s = i * step;
            if (static_cast<int>(r) == m_viaductRoad && std::abs(s - m_viaductS) < 220.0 &&
                z[i] - std::max(0.0, nr.nat[i]) > 2.5)
                nr.st[i] = NetViaduct;
            if (static_cast<int>(r) == m_underRoad && std::abs(s - m_underS) < 220.0 && z[i] < nr.nat[i] - 0.8)
                nr.st[i] = NetTrench;
        }
        // cut-and-cover tunnel: covered as far as the floor stays >= 4.5 m below the cover slab
        // (slab 1 m thick, its top at the ground) - not only right under the suburb road
        if (static_cast<int>(r) == m_underRoad)
        {
            const int ic = std::clamp(static_cast<int>(std::lround(m_underS / step)), 0, n - 1);
            auto deep = [&](int i) {
                return std::abs(i * step - m_underS) < 16.0 ||
                       (std::abs(i * step - m_underS) < 70.0 && z[i] <= std::max(0.0, nr.nat[i]) - 5.5);
            };
            for (int i = ic; i >= 0 && deep(i); --i)
                nr.st[i] = NetCovered;
            for (int i = ic; i < n && deep(i); ++i)
                nr.st[i] = NetCovered;
        }
    }

    // raster mask (1 m) for the LiDAR worker threads: 1 open trench, 2 + road on the ground, 255 junction
    {
        double b[4] = { 1e18, -1e18, 1e18, -1e18 };
        for (const auto& nr : m_net)
        {
            double c[4];
            nr.road.bounds(c);
            b[0] = std::min(b[0], c[0]); b[1] = std::max(b[1], c[1]);
            b[2] = std::min(b[2], c[2]); b[3] = std::max(b[3], c[3]);
        }
        m_maskX0 = b[0] - 25.0;
        m_maskY0 = b[2] - 25.0;
        m_maskNx = static_cast<int>(b[1] - b[0] + 50.0);
        m_maskNy = static_cast<int>(b[3] - b[2] + 50.0);
        m_netMask.assign(static_cast<size_t>(m_maskNx) * m_maskNy, 0);
        auto mark = [&](double x, double y, unsigned char v) {
            const int i = static_cast<int>(std::floor(x - m_maskX0)), j = static_cast<int>(std::floor(y - m_maskY0));
            if (i >= 0 && j >= 0 && i < m_maskNx && j < m_maskNy)
                m_netMask[static_cast<size_t>(j) * m_maskNx + i] = v;
        };
        for (size_t r = 0; r < m_net.size(); ++r)
        {
            const NetRoad& nr = m_net[r];
            const double L = nr.road.length();
            for (double s = 0.0; s <= L; s += 0.4)
            {
                const int st = netStructure(static_cast<int>(r), s);
                if (st == NetViaduct)
                    continue;
                // open trench and the covered tunnel: no terrain there (the cover slab is a mesh the rays hit)
                const bool cut = st == NetTrench || st == NetCovered;
                const double reach = cut ? nr.road.halfWidth + 1.5 : nr.road.halfWidth + 0.4;
                for (double lat = -reach; lat <= reach; lat += 0.4)
                {
                    double x, y;
                    nr.road.point(s, lat, x, y);
                    mark(x, y, cut ? 1 : static_cast<unsigned char>(2 + r));
                }
            }
        }
        for (const auto& nd : m_nodes)
            for (double y = nd.y - nd.radius; y <= nd.y + nd.radius; y += 0.5)
                for (double x = nd.x - nd.radius; x <= nd.x + nd.radius; x += 0.5)
                    if (std::hypot(x - nd.x, y - nd.y) < nd.radius)
                        mark(x, y, 255);
    }
    m_netReady = true;
}

World::NetParts World::buildNetGeometry()
{
    NetParts out;
    const double gz = m_road.groundZ;
    auto constMat = [](int m) { return [m](int, int, int, double, double, double) { return m; }; };

    // ---- connector streets from the downtown cross streets (x = -35, x = -165) up to the network
    for (double si : { 55.0, 185.0 })
    {
        const double s0 = si - 6.0, s1 = si + 6.0;
        out.roads.push_back(SceneBuilder::ribbon(m_road, s0, s1, kConnLat0 - 0.5, kConnLat1, gz - 0.004, 2.0,
                                                 Palette::Asphalt, 4));
        m_paved.push_back({ s0, s1, kConnLat0, kConnLat1, int(Palette::Asphalt) });
        for (double ds : { -0.13, 0.13 })
            out.marks.push_back(SceneBuilder::ribbon(m_road, si + ds - 0.07, si + ds + 0.07, kConnLat0 + 6.0,
                                                     kConnLat1 - 3.0, gz + 0.004, 1.0, Palette::LaneYellow, 8));
        for (double d = kConnLat0 + 6.0; d + 3.0 < kConnLat1 - 3.0; d += 7.0)
            for (double ds : { -3.0, 3.0 })
                out.marks.push_back(SceneBuilder::ribbon(m_road, si + ds - 0.07, si + ds + 0.07, d, d + 3.0, gz + 0.004,
                                                         1.0, Palette::LaneWhite));
    }

    // ---- junction pads: flat discs at the junction level, a stop line on every street that enters
    for (const auto& nd : m_nodes)
    {
        const int seg = 32;
        out.roads.push_back(SceneBuilder::morphedHex(seg, 1, 1,
            [=](double u, double v, double w, double o[3]) {
                const double a = 2.0 * kPi * u;
                const double rr = nd.radius * (0.02 + 0.98 * v);
                o[0] = nd.x + rr * std::cos(a);
                o[1] = nd.y + rr * std::sin(a);
                o[2] = gz + nd.z - 0.004 - 0.02 * (1.0 - w);
            },
            constMat(Palette::Asphalt)));

        // directions of the streets that enter this junction
        std::vector<std::pair<double, double>> entries;   // (angle, half opening)
        for (const auto& e : nd.ends)
        {
            const RoadModel& rd = m_net[e.first].road;
            const double s = e.second == 0 ? nd.radius : rd.length() - nd.radius;
            double x, y;
            rd.point(s, 0.0, x, y);
            entries.push_back({ std::atan2(y - nd.y, x - nd.x), std::asin(std::min(0.95, (rd.halfWidth + 2.8) / nd.radius)) });
        }
        const int nseg = 72;
        auto open = [&](double a) {
            for (const auto& [ea, half] : entries)
            {
                double d = std::fmod(std::abs(a - ea), 2.0 * kPi);
                if (d > kPi) d = 2.0 * kPi - d;
                if (d < half)
                    return true;
            }
            return false;
        };
        for (int i = 0; i < nseg; ++i)
        {
            const double a0 = 2.0 * kPi * i / nseg, a1 = 2.0 * kPi * (i + 1) / nseg;
            if (open(0.5 * (a0 + a1)))
                continue;
            const double zc = gz + nd.z;
            // sidewalk (2.2 m) and the curb in front of it
            out.props.push_back(SceneBuilder::morphedHex(1, 1, 1,
                [=](double u, double v, double w, double o[3]) {
                    const double a = a0 + (a1 - a0) * u, rr = nd.radius + 0.15 + 2.2 * v;
                    o[0] = nd.x + rr * std::cos(a);
                    o[1] = nd.y + rr * std::sin(a);
                    o[2] = zc + 0.02 + 0.10 * w;
                },
                constMat(Palette::Sidewalk)));
            out.props.push_back(SceneBuilder::morphedHex(1, 1, 1,
                [=](double u, double v, double w, double o[3]) {
                    const double a = a0 + (a1 - a0) * u, rr = nd.radius - 0.02 + 0.18 * v;
                    o[0] = nd.x + rr * std::cos(a);
                    o[1] = nd.y + rr * std::sin(a);
                    o[2] = zc - 0.02 + 0.17 * w;
                },
                constMat(Palette::Concrete)));
        }
    }

    for (size_t ri = 0; ri < m_net.size(); ++ri)
    {
        const NetRoad& nr = m_net[ri];
        const RoadModel& r = nr.road;
        const RoadModel* rp = &nr.road;
        const double hw = r.halfWidth;
        const double L = r.length();
        const double a0 = m_nodes[nr.nodeA].radius - 1.0, a1 = L - m_nodes[nr.nodeB].radius + 1.0;
        const int road = static_cast<int>(ri);

        // ---- surface + markings (not inside the junctions)
        out.roads.push_back(SceneBuilder::ribbon(r, a0, a1, -hw - 0.4, hw + 0.4, gz, 2.0, Palette::Asphalt, 2 * r.lanes + 2));
        {
            // markings: double yellow center, dashed lane dividers, solid edges; stop lines before the junctions
            for (double side : { -1.0, 1.0 })
            {
                out.marks.push_back(SceneBuilder::ribbon(r, a0 + 2.0, a1 - 2.0, side > 0 ? 0.06 : -0.20,
                                                         side > 0 ? 0.20 : -0.06, gz + 0.006, 2.0, Palette::LaneYellow));
                out.marks.push_back(SceneBuilder::ribbon(r, a0 + 2.0, a1 - 2.0, side > 0 ? hw + 0.08 : -hw - 0.22,
                                                         side > 0 ? hw + 0.22 : -hw - 0.08, gz + 0.006, 2.0,
                                                         Palette::LaneWhite));
                for (int k = 1; k < r.lanes; ++k)
                    for (double s = a0 + 4.0; s + 3.0 < a1 - 4.0; s += 7.0)
                        out.marks.push_back(SceneBuilder::ribbon(r, s, s + 3.0, side * k * r.laneWidth - 0.07,
                                                                 side * k * r.laneWidth + 0.07, gz + 0.006, 1.0,
                                                                 Palette::LaneWhite));
            }
            // stop lines: entering the end junction (right half), entering the start junction (left half)
            if (m_nodes[nr.nodeB].ends.size() > 1)
                out.marks.push_back(SceneBuilder::ribbon(r, a1 - 2.6, a1 - 2.1, 0.3, hw, gz + 0.008, 1.0, Palette::LaneWhite));
            if (m_nodes[nr.nodeA].ends.size() > 1)
                out.marks.push_back(SceneBuilder::ribbon(r, a0 + 2.1, a0 + 2.6, -hw, -0.3, gz + 0.008, 1.0, Palette::LaneWhite));
        }

        // structure runs of this street
        const double step = r.elevationStep();
        const int n = static_cast<int>(nr.st.size());
        auto runs = [&](int type) {
            std::vector<std::pair<double, double>> v;
            for (int i = 0; i < n; ++i)
                if (nr.st[i] == type && (i == 0 || nr.st[i - 1] != type))
                {
                    int e = i;
                    while (e < n && nr.st[e] == type)
                        ++e;
                    v.push_back({ std::max(0.0, (i - 0.5) * step), std::min(L, (e - 0.5) * step) });
                }
            return v;
        };

        // ---- guard rails on high embankments / deep cuts
        for (int side : { -1, 1 })
        {
            double a = -1.0;
            for (double s = 0.0; s <= L + 1.0; s += 1.0)
            {
                const bool on = s <= L && netStructure(road, s) == NetGraded && netRailAt(road, s, side);
                if (on && a < 0.0)
                    a = s;
                if (!on && a >= 0.0)
                {
                    if (s - a > 2.0)
                        out.walls.push_back(SceneBuilder::wallAlong(r, a, s - 1.0, side * (hw + 1.0), 0.08, gz + 0.45,
                                                                    gz + 0.8, 3.0, Palette::Metal));
                    a = -1.0;
                }
            }
        }

        // ---- sidewalks and curbs along the city streets (not on the viaduct / in the underpass)
        if (r.lanes >= 2)
        {
            double a = -1.0;
            for (double s = a0; s <= a1 + 1.0; s += 1.0)
            {
                const bool on = s <= a1 && netStructure(road, s) == NetGraded;
                if (on && a < 0.0)
                    a = s;
                if (!on && a >= 0.0)
                {
                    const double b = std::min(a1, s - 1.0);
                    if (b - a > 2.0)
                        for (int side : { -1, 1 })
                        {
                            const double l0 = side * (hw + 0.55), l1 = side * (hw + 2.75);
                            out.props.push_back(SceneBuilder::ribbon(r, a, b, std::min(l0, l1), std::max(l0, l1),
                                                                     gz + 0.12, 2.0, Palette::Sidewalk));
                            out.props.push_back(SceneBuilder::wallAlong(r, a, b, side * (hw + 0.48), 0.16, gz - 0.02,
                                                                        gz + 0.14, 2.0, Palette::Concrete));
                        }
                    a = -1.0;
                }
            }
        }

        // ---- street lights along the streets in the park (every 35 m, alternating sides)
        if (r.lanes >= 2)
        {
            int k = 0;
            for (double s = a0 + 12.0; s < a1 - 12.0; s += 35.0, ++k)
            {
                if (netStructure(road, s) != NetGraded)
                    continue;
                const double side = k % 2 ? 1.0 : -1.0;
                double x, y, mx, my, xa, ya;
                r.point(s, side * (hw + 1.8), x, y);
                r.point(s, side * (hw + 0.2), mx, my);
                r.point(s, side * (hw - 1.4), xa, ya);
                const double z0 = gz + r.elevation(s);
                out.poles.push_back({ x, y, z0, 6.0 });
                const double yaw = r.heading(s) * 180.0 / kPi;
                out.props.push_back(SceneBuilder::orientedBlock(mx, my, yaw, 0.12, 3.4, z0 + 5.85, z0 + 5.95, 1, 2, 1,
                                                                constMat(Palette::LampPost)));
                out.props.push_back(SceneBuilder::orientedBlock(xa, ya, yaw, 0.5, 0.4, z0 + 5.7, z0 + 5.85, 1, 1, 1,
                                                                constMat(Palette::Lamp)));
                m_circles.push_back({ x, y, 0.12, ObstacleKind::StreetLight });
            }
        }

        // ---- viaduct: deck, parapets, piers with caps
        PolyBatch piers;
        for (const auto& [a, b] : runs(NetViaduct))
        {
            out.bridge.push_back(SceneBuilder::wallAlong(r, a, b, 0.0, 2.0 * (hw + 1.6), gz - 1.1, gz - 0.03, 2.0,
                                                         Palette::Concrete));
            out.lidar.push_back(out.bridge.back());
            for (int side : { -1, 1 })
            {
                out.bridge.push_back(SceneBuilder::wallAlong(r, a, b, side * (hw + 1.35), 0.4, gz, gz + 1.05, 2.0,
                                                             Palette::Concrete));
                out.lidar.push_back(out.bridge.back());
            }
            out.lidar.push_back(SceneBuilder::ribbon(r, a, b, -hw - 0.4, hw + 0.4, gz, 2.0, Palette::Asphalt, 4));
            for (double s = a + 8.0; s < b - 6.0; s += 20.0)
            {
                const double deck = gz + r.elevation(s) - 1.1;
                bool any = false;
                for (double pl : { -3.8, 3.8 })
                {
                    double x, y, ms, ml;
                    r.point(s, pl, x, y);
                    if (m_road.project(x, y, ms, ml) && std::abs(ml) < m_road.halfWidth + 3.0)
                        continue;   // no pier on the highway
                    const double g0 = gz + std::max(0.0, naturalHeight(x, y)) - 0.3;
                    if (deck - g0 < 1.0)
                        continue;
                    piers.cylinder(x, y, g0, 0.75, deck - g0, 16);
                    m_circles.push_back({ x, y, 0.8, ObstacleKind::Pier });
                    any = true;
                }
                if (any)
                {
                    double cx, cy;
                    r.point(s, 0.0, cx, cy);
                    out.bridge.push_back(SceneBuilder::orientedBlock(cx, cy, r.heading(s) * 180.0 / kPi, 1.8,
                                                                     2.0 * hw + 2.0, deck - 0.9, deck, 1, 4, 1,
                                                                     constMat(Palette::Concrete)));
                    out.lidar.push_back(out.bridge.back());
                }
            }
        }
        if (piers.pts->GetNumberOfPoints() > 0)
        {
            out.bridge.push_back(piers.grid(Palette::Concrete));
            out.lidar.push_back(out.bridge.back());
        }

        // ---- underpass: floor shoulders, retaining walls, cover slab, lights
        std::vector<std::pair<double, double>> cut = runs(NetTrench);
        for (const auto& c : runs(NetCovered))
            cut.push_back(c);
        std::sort(cut.begin(), cut.end());
        std::vector<std::pair<double, double>> merged;
        for (const auto& c : cut)
        {
            if (!merged.empty() && c.first <= merged.back().second + step)
                merged.back().second = std::max(merged.back().second, c.second);
            else
                merged.push_back(c);
        }
        const double wallLat = hw + 1.5;
        for (const auto& [a, b] : merged)
        {
            out.lidar.push_back(SceneBuilder::ribbon(r, a, b, -hw - 0.4, hw + 0.4, gz, 2.0, Palette::Asphalt, 4));
            for (int side : { -1, 1 })
            {
                out.bridge.push_back(SceneBuilder::ribbon(r, a, b, std::min(side * (hw + 0.4), side * (wallLat - 0.3)),
                                                          std::max(side * (hw + 0.4), side * (wallLat - 0.3)),
                                                          gz - 0.01, 2.0, Palette::Concrete));
                out.lidar.push_back(out.bridge.back());
                const int nseg = std::max(1, static_cast<int>((b - a) / 2.0));
                const double lat = side * wallLat;
                out.bridge.push_back(SceneBuilder::morphedHex(nseg, 1, 1,
                    [=](double u, double v, double w, double o[3]) {
                        const double s = a + (b - a) * u;
                        rp->point(s, lat + (v - 0.5) * 0.6, o[0], o[1]);
                        const double bottom = gz + rp->elevation(s) - 0.3;
                        double top;
                        if (netStructure(road, s) == NetCovered)
                            top = gz + netCoverTop(road, s) - 0.02;
                        else
                        {
                            double ox, oy;
                            rp->point(s, lat + side * 0.6, ox, oy);
                            top = gz + std::max(terrainHeight(ox, oy), rp->elevation(s)) + 0.9;   // displayed ground + parapet
                        }
                        o[2] = bottom + (top - bottom) * w;
                    },
                    constMat(Palette::Concrete)));
                out.lidar.push_back(out.bridge.back());
            }
        }
        for (const auto& [a, b] : runs(NetCovered))
        {
            const double ext = wallLat + 0.6;
            out.bridge.push_back(SceneBuilder::morphedHex(std::max(1, static_cast<int>((b - a) / 2.0)), 4, 1,
                [=](double u, double v, double w, double o[3]) {
                    const double s = a + (b - a) * u;
                    rp->point(s, -ext + 2.0 * ext * v, o[0], o[1]);
                    o[2] = gz + netCoverTop(road, s) - 1.05 + 1.03 * w;
                },
                constMat(Palette::Concrete)));
            out.lidar.push_back(out.bridge.back());
            for (double pl : { -3.0, 3.0 })
                out.lights.push_back(SceneBuilder::morphedHex(std::max(1, static_cast<int>((b - a) / 2.0)), 1, 1,
                    [=](double u, double v, double w, double o[3]) {
                        const double s = a + (b - a) * u;
                        rp->point(s, pl - 0.18 + 0.36 * v, o[0], o[1]);
                        o[2] = gz + netCoverTop(road, s) - 1.12 + 0.05 * w;
                    },
                    constMat(Palette::Lamp)));
        }
    }
    return out;
}
