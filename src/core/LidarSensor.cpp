#include "LidarSensor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <thread>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>

#include "Palette.h"
#include "VehicleModel.h"

namespace
{
constexpr double kDeg = 3.14159265358979323846 / 180.0;

// cheap deterministic noise in [-1, 1] (same scene -> same cloud)
double hashNoise(uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA6Bu ^ (c + 0x1234567u) * 0xC2B2AE35u;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
    return (h & 0xFFFFFF) / double(0xFFFFFF) * 2.0 - 1.0;
}

// smooth 3D value noise in [-1, 1] (surface texture of the intensity: asphalt aggregate, worn paint ...)
double valueNoise(double x, double y, double z)
{
    const double fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy), iz = static_cast<int>(fz);
    double u = x - fx, v = y - fy, w = z - fz;
    u = u * u * (3 - 2 * u); v = v * v * (3 - 2 * v); w = w * w * (3 - 2 * w);
    auto L = [](int a, int b, int c) {
        return hashNoise(static_cast<uint32_t>(a), static_cast<uint32_t>(b), static_cast<uint32_t>(c) * 7919u + 13u);
    };
    const double x00 = L(ix, iy, iz) * (1 - u) + L(ix + 1, iy, iz) * u;
    const double x10 = L(ix, iy + 1, iz) * (1 - u) + L(ix + 1, iy + 1, iz) * u;
    const double x01 = L(ix, iy, iz + 1) * (1 - u) + L(ix + 1, iy, iz + 1) * u;
    const double x11 = L(ix, iy + 1, iz + 1) * (1 - u) + L(ix + 1, iy + 1, iz + 1) * u;
    return (x00 * (1 - v) + x10 * v) * (1 - w) + (x01 * (1 - v) + x11 * v) * w;
}

// How a surface sends the laser back
enum class Scatter
{
    Diffuse,   // Lambertian: concrete, grass, asphalt ...  ~ cos(incidence)
    Retro,     // retro-reflective (glass beads, sign sheeting, reflectors): almost angle independent
    Glossy,    // car paint: diffuse base + specular lobe straight back at normal incidence
    Mirror     // glass, water, metal: almost nothing, except a sharp spike when hit head-on
};

Scatter scatterOf(int m)
{
    switch (m)
    {
    case Palette::LaneWhite: case Palette::LaneYellow: case Palette::SignGreen: case Palette::SignRed:
    case Palette::Barrel: case Palette::Headlight: case Palette::Taillight: case Palette::Pole:
        return Scatter::Retro;
    case Palette::Body: case Palette::Cargo: case Palette::NpcBlue: case Palette::NpcWhite:
    case Palette::NpcGreen: case Palette::Rim: case Palette::EgoSilver: case Palette::EgoBlack:
    case Palette::TunnelTile:
        return Scatter::Glossy;
    case Palette::Glass: case Palette::Window: case Palette::Water: case Palette::Metal:
        return Scatter::Mirror;
    default:
        return Scatter::Diffuse;
    }
}

// relative reflectivity variation of the material's surface (0 = perfectly uniform)
double textureOf(int m)
{
    switch (m)
    {
    case Palette::Asphalt:   return 0.45;
    case Palette::Grass:     return 0.50;
    case Palette::Leaf: case Palette::LeafDark: case Palette::LeafLight: return 0.55;
    case Palette::Trunk:     return 0.30;
    case Palette::LaneWhite: case Palette::LaneYellow: return 0.30;   // worn paint
    case Palette::Sidewalk: case Palette::Concrete: case Palette::Jersey: return 0.25;
    case Palette::Brick:     return 0.35;
    case Palette::Roof: case Palette::RoofGray: return 0.30;
    case Palette::Crate:     return 0.30;
    case Palette::Stucco:    return 0.20;
    default:                 return 0.10;
    }
}

// Ray vs. oriented box (slab method). Returns the entry distance or -1; normal = entry face (world).
double rayBox(const double o[3], const double d[3], const LidarSensor::DynamicBox& b, double tMax,
              double normal[3])
{
    const double c = std::cos(b.heading), s = std::sin(b.heading);
    const double ox = o[0] - b.cx, oy = o[1] - b.cy;
    const double lo[3] = { c * ox + s * oy, -s * ox + c * oy, o[2] };
    const double ld[3] = { c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2] };
    const double mn[3] = { -b.halfL, -b.halfW, b.zMin };
    const double mx[3] = { b.halfL, b.halfW, b.zMax };

    double t0 = 0.0, t1 = tMax;
    int axis = 2;
    double sign = 1.0;
    for (int k = 0; k < 3; ++k)
    {
        if (std::abs(ld[k]) < 1e-12)
        {
            if (lo[k] < mn[k] || lo[k] > mx[k])
                return -1.0;
            continue;
        }
        double ta = (mn[k] - lo[k]) / ld[k];
        double tb = (mx[k] - lo[k]) / ld[k];
        if (ta > tb) std::swap(ta, tb);
        if (ta > t0) { t0 = ta; axis = k; sign = ld[k] > 0 ? -1.0 : 1.0; }
        t1 = std::min(t1, tb);
        if (t0 > t1)
            return -1.0;
    }
    double nl[3] = { 0, 0, 0 };
    nl[axis] = sign;
    normal[0] = c * nl[0] - s * nl[1];
    normal[1] = s * nl[0] + c * nl[1];
    normal[2] = nl[2];
    return t0;
}
}

double LidarSensor::reflectivity(int m)
{
    // calibrated reflectivity 0..1; > 0.8 behaves like retro-reflective material
    switch (m)
    {
    case Palette::Asphalt:     return 0.07;
    case Palette::LaneWhite:   return 0.95;   // road paint with glass beads
    case Palette::LaneYellow:  return 0.80;
    case Palette::Water:       return 0.02;   // water absorbs / mirrors the beam away
    case Palette::Roof:        return 0.25;
    case Palette::RoofGray:    return 0.20;
    case Palette::Stucco:      return 0.50;
    case Palette::Metal:       return 0.60;
    case Palette::SignGreen:   return 0.90;   // retro-reflective sign sheeting
    case Palette::LeafLight:   return 0.26;
    case Palette::Jersey:      return 0.42;
    case Palette::TunnelTile:  return 0.62;
    case Palette::EgoSilver:   return 0.50;
    case Palette::EgoBlack:    return 0.10;
    case Palette::Grass:       return 0.20;
    case Palette::Sidewalk:    return 0.33;
    case Palette::Concrete:    return 0.38;
    case Palette::Brick:       return 0.30;
    case Palette::Window:      return 0.06;   // glass returns little light
    case Palette::Glass:       return 0.05;
    case Palette::Leaf:        return 0.24;
    case Palette::LeafDark:    return 0.20;
    case Palette::Trunk:       return 0.14;
    case Palette::Crate:       return 0.34;
    case Palette::Barrel:      return 0.88;   // reflective bands
    case Palette::WallRed:     return 0.55;
    case Palette::Trim:        return 0.12;
    case Palette::Tire:        return 0.04;
    case Palette::Rim:         return 0.50;
    case Palette::Body:        return 0.32;
    case Palette::Cargo:       return 0.45;
    case Palette::NpcBlue:     return 0.28;
    case Palette::NpcWhite:    return 0.55;
    case Palette::NpcGreen:    return 0.30;
    case Palette::Headlight:   return 0.92;   // reflectors
    case Palette::Taillight:   return 0.92;
    case Palette::LampPost:    return 0.40;
    case Palette::Lamp:        return 0.70;
    case Palette::SignalOff:   return 0.10;
    case Palette::SignalGreen: return 0.60;
    case Palette::SignRed:     return 1.00;   // traffic sign sheeting
    case Palette::Pole:        return 0.85;
    }
    return 0.30;
}

double LidarSensor::computeIntensity(int material, double cosI, double range, int ring, const double p[3],
                                     double noise)
{
    double rho = reflectivity(material);

    // surface texture: two octaves of value noise at the hit point (+ big patches on asphalt)
    const double tex = textureOf(material);
    const double fbm = 0.6 * valueNoise(p[0] / 0.9, p[1] / 0.9, p[2] / 0.9) +
                       0.4 * valueNoise(p[0] / 0.22 + 31.7, p[1] / 0.22, p[2] / 0.22);
    rho *= std::max(0.05, 1.0 + tex * fbm);
    if (material == Palette::Asphalt)
        rho *= 1.0 + 0.3 * valueNoise(p[0] / 7.0, p[1] / 7.0, 3.3);   // repaired / worn patches

    cosI = std::clamp(cosI, 0.0, 1.0);
    double I = 0.0;
    switch (scatterOf(material))
    {
    case Scatter::Diffuse: I = rho * (0.12 + 0.88 * cosI); break;
    case Scatter::Retro:   I = rho * (0.80 + 0.20 * cosI); break;
    case Scatter::Glossy:  I = rho * (0.20 + 0.80 * cosI) + 0.45 * std::pow(cosI, 40.0); break;
    case Scatter::Mirror:  I = rho * (0.30 + 0.70 * cosI) + 0.90 * std::pow(cosI, 150.0); break;
    }

    // near field: the receiver optics do not see the spot well below ~4 m; far: a little attenuation
    const double nearT = std::clamp((range - 1.0) / 3.0, 0.0, 1.0);
    I *= (0.55 + 0.45 * nearT * nearT * (3 - 2 * nearT)) * (1.0 - 0.15 * std::min(range, 200.0) / 200.0);
    // every laser of a real puck has its own gain (factory calibration leaves a few % per ring)
    I *= 1.0 + 0.05 * hashNoise(static_cast<uint32_t>(ring), 31u, 5u);
    // detector noise grows with range (weaker signal)
    I += (0.012 + 0.03 * std::min(range, 200.0) / 200.0) * noise;
    // 8-bit output like the sensor (calibrated reflectivity 0..255)
    return std::round(std::clamp(I, 0.0, 1.0) * 255.0) / 255.0;
}

double LidarSensor::displayValue(double I)
{
    // Calibrated reflectivity is not linear in what we want to see: diffuse surfaces live in
    // 0 .. ~0.6 (Velodyne 0..150), retro-reflectors above. Stretch the diffuse part (gamma) so asphalt,
    // grass, concrete and paint get clearly different colors, and keep the top for retro-reflectors.
    I = std::clamp(I, 0.0, 1.0);
    return I < 0.6 ? 0.82 * std::pow(I / 0.6, 0.6) : 0.82 + 0.18 * (I - 0.6) / 0.4;
}

void LidarSensor::intensityColor(double t, unsigned char rgb[3])
{
    // "turbo"-like rainbow with more stops (smooth, perceptually more even than 5 stops),
    // low end kept bright enough to be seen on dark asphalt / the dark top view
    t = std::clamp(t, 0.0, 1.0);
    static const double stops[9][3] = {
        { 0.24, 0.20, 0.62 }, { 0.26, 0.38, 0.90 }, { 0.16, 0.62, 0.98 }, { 0.10, 0.84, 0.82 },
        { 0.36, 0.97, 0.42 }, { 0.74, 0.96, 0.20 }, { 0.99, 0.76, 0.18 }, { 0.98, 0.44, 0.10 },
        { 0.86, 0.12, 0.08 } };
    const double s = t * 8.0;
    const int i = std::min(7, static_cast<int>(s));
    const double f = s - i;
    for (int k = 0; k < 3; ++k)
        rgb[k] = static_cast<unsigned char>(255.0 * (stops[i][k] * (1 - f) + stops[i + 1][k] * f));
}

double LidarSensor::HeightField::at(double x, double y) const
{
    if (nx < 2 || ny < 2)
        return 0.0;
    const double fx = (x - x0) / step, fy = (y - y0) / step;
    const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
    if (i < 0 || j < 0 || i >= nx - 1 || j >= ny - 1)
        return 0.0;
    const double u = fx - i, v = fy - j;
    const float* r0 = &h[static_cast<size_t>(j) * nx + i];
    const float* r1 = r0 + nx;
    return (r0[0] * (1 - u) + r0[1] * u) * (1 - v) + (r1[0] * (1 - u) + r1[1] * u) * v;
}

void LidarSensor::HeightField::buildTiles()
{
    tnx = (nx + 3) / 4;
    tny = (ny + 3) / 4;
    tileMax.assign(static_cast<size_t>(tnx) * tny, 0.0f);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
        {
            // a cell influences the tiles of both of its corners: dilate by one cell
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di)
                {
                    const int ti = (i + di) / 4, tj = (j + dj) / 4;
                    if (i + di < 0 || j + dj < 0 || ti >= tnx || tj >= tny)
                        continue;
                    float& m = tileMax[static_cast<size_t>(tj) * tnx + ti];
                    m = std::max(m, h[static_cast<size_t>(j) * nx + i]);
                }
        }
}

double LidarSensor::HeightField::tileMaxAt(double x, double y) const
{
    const int ti = static_cast<int>(std::floor((x - x0) / (4.0 * step)));
    const int tj = static_cast<int>(std::floor((y - y0) / (4.0 * step)));
    if (ti < 0 || tj < 0 || ti >= tnx || tj >= tny)
        return 0.0;
    return tileMax[static_cast<size_t>(tj) * tnx + ti];
}

double LidarSensor::occupancyTop(double x, double y) const
{
    const int i = static_cast<int>(std::floor((x - m_occX0) / m_occCell));
    const int j = static_cast<int>(std::floor((y - m_occY0) / m_occCell));
    if (i < 0 || j < 0 || i >= m_occNx || j >= m_occNy)
        return -1e9;
    return m_occTop[static_cast<size_t>(j) * m_occNx + i];
}

LidarSensor::LidarSensor()
{
    m_cloud = vtkSmartPointer<vtkPolyData>::New();
    m_obstacleCloud = vtkSmartPointer<vtkPolyData>::New();
    clear();
}

LidarSensor::~LidarSensor()
{
    if (m_asyncThread.joinable())
    {
        {
            std::lock_guard<std::mutex> lock(m_asyncMutex);
            m_asyncQuit = true;
        }
        m_asyncCv.notify_all();
        m_asyncThread.join();
    }
}

void LidarSensor::setPipelined(bool on)
{
    sync();
    m_pipelined = on;
    if (on && !m_asyncThread.joinable())
        m_asyncThread = std::thread([this] { asyncLoop(); });
}

void LidarSensor::asyncLoop()
{
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(m_asyncMutex);
            m_asyncCv.wait(lock, [this] { return m_asyncQuit || m_asyncBusy; });
            if (m_asyncQuit)
                return;
        }
        const auto t0 = std::chrono::steady_clock::now();
        for (const Job& j : m_running)
            scanColumns(j.vehicle, j.boxes, j.first, j.count);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        m_scanMsAvg = 0.9 * m_scanMsAvg + 0.1 * ms / std::max<size_t>(1, m_running.size());
        {
            std::lock_guard<std::mutex> lock(m_asyncMutex);
            m_asyncBusy = false;
        }
        m_asyncCv.notify_all();
    }
}

void LidarSensor::kick()
{
    if (!m_pipelined || m_queue.empty())
        return;
    sync();
    m_running.swap(m_queue);
    m_queue.clear();
    {
        std::lock_guard<std::mutex> lock(m_asyncMutex);
        m_asyncBusy = true;
    }
    m_asyncCv.notify_all();
}

bool LidarSensor::sync()
{
    if (m_asyncThread.joinable())
    {
        std::unique_lock<std::mutex> lock(m_asyncMutex);
        m_asyncCv.wait(lock, [this] { return !m_asyncBusy; });
    }
    m_running.clear();
    if (!m_pendingData)
        return false;
    m_pendingData = false;
    m_revolution += m_pendingRevolutions;
    m_pendingRevolutions = 0;
    rebuildHits();
    m_outputsDirty = true;
    return true;
}

int LidarSensor::columns() const
{
    return static_cast<int>(std::round(360.0 / config.azimuthStepDeg));
}

void LidarSensor::setObstacleSurface(vtkPolyData* surface)
{
    // build cell structures up front: GetCell() is then safe to call from worker threads
    surface->BuildCells();

    surface->GetBounds(m_bounds);
    for (int k = 0; k < 3; ++k)   // small margin
    {
        m_bounds[2 * k] -= 0.5;
        m_bounds[2 * k + 1] += 0.5;
    }

    // Uniform bins chosen explicitly (the automatic mode caps the bucket count and, for this large
    // scene, made bins several meters wide -> many triangles tested per bin).
    // Target ~10 M bins: tall z bins (8 m, rays are nearly horizontal), xy bins as small as that
    // budget allows (>= 1 m).
    const double ex = m_bounds[1] - m_bounds[0], ey = m_bounds[3] - m_bounds[2], ez = m_bounds[5] - m_bounds[4];
    const int nz = std::max(1, static_cast<int>(std::ceil(ez / 8.0)));
    const double binXY = std::max(1.0, std::sqrt(ex * ey * nz / 10.0e6));
    m_locator = vtkSmartPointer<vtkStaticCellLocator>::New();
    m_locator->SetDataSet(surface);
    m_locator->AutomaticOff();
    m_locator->SetDivisions(std::max(1, static_cast<int>(std::ceil(ex / binXY))),
                            std::max(1, static_cast<int>(std::ceil(ey / binXY))), nz);
    m_locator->BuildLocator();

    // coarse occupancy: highest z of the geometry per 8 m cell, dilated by one cell
    {
        m_occX0 = m_bounds[0];
        m_occY0 = m_bounds[2];
        m_occNx = std::max(1, static_cast<int>(std::ceil(ex / m_occCell)));
        m_occNy = std::max(1, static_cast<int>(std::ceil(ey / m_occCell)));
        std::vector<float> top(static_cast<size_t>(m_occNx) * m_occNy, -1e9f);
        double cb[6];
        for (vtkIdType c = 0; c < surface->GetNumberOfCells(); ++c)
        {
            surface->GetCellBounds(c, cb);
            const int i0 = std::max(0, static_cast<int>((cb[0] - m_occX0) / m_occCell));
            const int i1 = std::min(m_occNx - 1, static_cast<int>((cb[1] - m_occX0) / m_occCell));
            const int j0 = std::max(0, static_cast<int>((cb[2] - m_occY0) / m_occCell));
            const int j1 = std::min(m_occNy - 1, static_cast<int>((cb[3] - m_occY0) / m_occCell));
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i)
                {
                    float& t = top[static_cast<size_t>(j) * m_occNx + i];
                    t = std::max(t, static_cast<float>(cb[5]));
                }
        }
        m_occTop.assign(top.size(), -1e9f);
        for (int j = 0; j < m_occNy; ++j)
            for (int i = 0; i < m_occNx; ++i)
            {
                float m = -1e9f;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di)
                    {
                        const int ii = i + di, jj = j + dj;
                        if (ii >= 0 && jj >= 0 && ii < m_occNx && jj < m_occNy)
                            m = std::max(m, top[static_cast<size_t>(jj) * m_occNx + ii]);
                    }
                m_occTop[static_cast<size_t>(j) * m_occNx + i] = m + 0.3f;
            }
    }

    // per-cell material and normal (Newell's method) for the intensity model
    const vtkIdType n = surface->GetNumberOfCells();
    m_cellMaterial.assign(n, -1);
    m_cellNormal.assign(3 * static_cast<size_t>(n), 0.0f);
    vtkDataArray* mat = surface->GetCellData()->GetArray("ElementScalar");
    vtkNew<vtkIdList> ids;
    for (vtkIdType c = 0; c < n; ++c)
    {
        if (mat)
            m_cellMaterial[c] = static_cast<int>(std::lround(mat->GetTuple1(c)));
        surface->GetCellPoints(c, ids);
        double nn[3] = { 0, 0, 0 };
        const vtkIdType k = ids->GetNumberOfIds();
        for (vtkIdType i = 0; i < k; ++i)
        {
            double a[3], b[3];
            surface->GetPoint(ids->GetId(i), a);
            surface->GetPoint(ids->GetId((i + 1) % k), b);
            nn[0] += (a[1] - b[1]) * (a[2] + b[2]);
            nn[1] += (a[2] - b[2]) * (a[0] + b[0]);
            nn[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        const double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
        for (int j = 0; j < 3; ++j)
            m_cellNormal[3 * c + j] = static_cast<float>(len > 0 ? nn[j] / len : (j == 2 ? 1.0 : 0.0));
    }
}

void LidarSensor::sensorWorldPosition(const VehicleModel& vehicle, double p[3]) const
{
    vehicle.localToWorld(config.mount, p);
}

void LidarSensor::clear()
{
    sync();
    m_queue.clear();
    m_pendingRevolutions = 0;
    m_columns.assign(columns(), {});
    m_hits.clear();
    m_totalPoints = 0;
    m_nextColumn = 0;
    m_columnAccum = 0.0;
    m_revolution = 0;

    for (vtkPolyData* pd : { m_cloud.GetPointer(), m_obstacleCloud.GetPointer() })
    {
        vtkNew<vtkPoints> pts;
        pd->SetPoints(pts);
        vtkNew<vtkCellArray> verts;
        pd->SetVerts(verts);
        pd->GetPointData()->SetScalars(nullptr);
    }
    m_outputsDirty = true;
}

void LidarSensor::scanColumns(const VehicleModel& vehicle, const std::vector<DynamicBox>& boxes,
                              int firstColumn, int count)
{
    const int nCol = columns();
    const int nCh = channels();
    double origin[3];
    sensorWorldPosition(vehicle, origin);
    const double heading = vehicle.heading();
    const double R = config.range;

    // precomputed elevation sin / cos
    std::vector<double> se(nCh), ce(nCh);
    for (int c = 0; c < nCh; ++c)
    {
        se[c] = std::sin(config.elevationsDeg[c] * kDeg);
        ce[c] = std::cos(config.elevationsDeg[c] * kDeg);
    }

    auto worker = [&](int i0, int i1) {
        vtkNew<vtkGenericCell> cell;
        for (int i = i0; i < i1; ++i)
        {
            const int col = (firstColumn + i) % nCol;
            std::vector<Hit>& out = m_columns[col];
            out.clear();

            // azimuth is relative to the vehicle: the sensor is bolted to the roof
            const double az = heading + col * config.azimuthStepDeg * kDeg;
            const double ca = std::cos(az), sa = std::sin(az);

            for (int c = 0; c < nCh; ++c)
            {
                const double dir[3] = { ce[c] * ca, ce[c] * sa, se[c] };

                // ground first (analytic) -> shortens the ray for the locator query
                double tMax = R;
                bool groundHit = false;
                if (dir[2] < -1e-6)
                {
                    const double tg = (m_groundZ - origin[2]) / dir[2];
                    if (tg > 0.0 && tg < tMax &&
                        !(m_groundHole && m_groundHole(origin[0] + dir[0] * tg, origin[1] + dir[1] * tg)))
                    {
                        tMax = tg;
                        groundHit = true;
                    }
                }

                double tHit = tMax;
                bool any = false, ground = false;
                int material = Palette::Asphalt;
                int instance = -1;
                double normal[3] = { 0.0, 0.0, 1.0 };

                // clip the locator query to the scene bounds: rays into the sky stop early
                double tLoc = tMax;
                for (int k = 0; k < 3; ++k)
                {
                    if (std::abs(dir[k]) < 1e-12)
                        continue;
                    const double tb = ((dir[k] > 0 ? m_bounds[2 * k + 1] : m_bounds[2 * k]) - origin[k]) / dir[k];
                    if (tb >= 0.0)
                        tLoc = std::min(tLoc, tb);
                }

                if (m_locator && tLoc > 0.0)
                {
                    // Empty-space skipping: walk a coarse 2D occupancy grid (8 m cells with the highest
                    // z of the geometry inside) and ask the fine cell locator only about the stretches
                    // of the ray that can hit something. Inside the loop course most rays cross a lot of
                    // empty park / lake / sky.
                    const double step = 0.5 * m_occCell;
                    double runStart = -1.0;
                    auto query = [&](double t0, double t1) {
                        t0 = std::max(0.0, t0);
                        t1 = std::min(tLoc, t1);
                        if (t1 <= t0)
                            return false;
                        const double a[3] = { origin[0] + dir[0] * t0, origin[1] + dir[1] * t0, origin[2] + dir[2] * t0 };
                        const double b[3] = { origin[0] + dir[0] * t1, origin[1] + dir[1] * t1, origin[2] + dir[2] * t1 };
                        double t = 0.0, x[3], pcoords[3];
                        int subId = 0;
                        vtkIdType cellId = -1;
                        if (!m_locator->IntersectWithLine(a, b, 1e-4, t, x, pcoords, subId, cellId, cell))
                            return false;
                        tHit = t0 + t * (t1 - t0);
                        any = true;
                        if (cellId >= 0 && cellId < static_cast<vtkIdType>(m_cellMaterial.size()))
                        {
                            material = m_cellMaterial[cellId];
                            for (int k = 0; k < 3; ++k)
                                normal[k] = m_cellNormal[3 * cellId + k];
                        }
                        return true;
                    };
                    bool hitFound = false;
                    for (double t = 0.0; t < tLoc && !hitFound; t += step)
                    {
                        const double tn = std::min(t + step, tLoc);
                        const double zLow = origin[2] + dir[2] * (dir[2] < 0 ? tn : t);
                        const double mx = origin[0] + dir[0] * (0.5 * (t + tn)), my = origin[1] + dir[1] * (0.5 * (t + tn));
                        const bool maybe = zLow <= occupancyTop(mx, my);
                        if (maybe && runStart < 0.0)
                            runStart = t;
                        if (!maybe && runStart >= 0.0)
                        {
                            hitFound = query(runStart - 0.5, t + 0.5);
                            runStart = -1.0;
                        }
                    }
                    if (!hitFound && runStart >= 0.0)
                        query(runStart - 0.5, tLoc);
                }
                for (const auto& box : boxes)
                {
                    const double dx = box.cx - origin[0], dy = box.cy - origin[1];
                    if (dx * dx + dy * dy > (tHit + 3.0) * (tHit + 3.0))
                        continue;
                    double bn[3];
                    const double t = rayBox(origin, dir, box, tHit, bn);
                    if (t > 0.0 && t < tHit)
                    {
                        tHit = t;
                        any = true;
                        material = box.material;
                        instance = box.id;
                        std::copy(bn, bn + 3, normal);
                    }
                }
                // hills: march the height field up to the current hit (3 m steps + bisection)
                if (m_height.nx > 1)
                {
                    const double hz0 = origin[2] - m_groundZ;   // sensor height above the ground plane
                    double tPrev = 0.0;
                    for (double t = 2.0; t < tHit;)
                    {
                        const double hRay = hz0 + dir[2] * t;
                        if (hRay > m_height.maxH)
                        {
                            if (dir[2] >= 0.0) break;   // above every hill and climbing
                            tPrev = t;
                            t += 5.0;
                            continue;
                        }
                        const double px = origin[0] + dir[0] * t, py = origin[1] + dir[1] * t;
                        // inside the underpass cut there is no terrain (walls / floor / cover are meshes);
                        // the coarse height grid would average the floor and the ground beside it
                        if (m_groundHole && m_groundHole(px, py))
                        {
                            tPrev = t;
                            t += 1.0;
                            continue;
                        }
                        if (hRay > m_height.tileMaxAt(px, py) + 0.5)
                        {
                            tPrev = t;
                            t += 8.0;   // nothing this high in this 40 m tile
                            continue;
                        }
                        if (hRay < m_height.at(px, py))
                        {
                            double lo = tPrev, hi = t;
                            for (int it = 0; it < 6; ++it)
                            {
                                const double m = 0.5 * (lo + hi);
                                if (hz0 + dir[2] * m < m_height.at(origin[0] + dir[0] * m, origin[1] + dir[1] * m))
                                    hi = m;
                                else
                                    lo = m;
                            }
                            tHit = hi;
                            any = true;
                            ground = false;
                            instance = -1;
                            {
                                // a road laid on the hills (ring road) is ground, the rest is terrain
                                const double qx0 = origin[0] + dir[0] * hi, qy0 = origin[1] + dir[1] * hi;
                                const bool road = m_roadOnHills && m_roadOnHills(qx0, qy0);
                                material = road && m_groundMaterial ? m_groundMaterial(qx0, qy0) : Palette::Grass;
                                ground = road;
                            }
                            const double qx = origin[0] + dir[0] * hi, qy = origin[1] + dir[1] * hi;
                            const double e = 1.0;
                            const double gx = (m_height.at(qx + e, qy) - m_height.at(qx - e, qy)) / (2 * e);
                            const double gy = (m_height.at(qx, qy + e) - m_height.at(qx, qy - e)) / (2 * e);
                            const double nl = std::sqrt(gx * gx + gy * gy + 1.0);
                            normal[0] = -gx / nl; normal[1] = -gy / nl; normal[2] = 1.0 / nl;
                            break;
                        }
                        tPrev = t;
                        t += 2.0;
                    }
                }
                bool planeGround = false;
                if (!any && groundHit)
                {
                    any = true;
                    ground = true;
                    planeGround = true;
                    tHit = tMax;
                }
                // static road surfaces in the mesh (viaduct deck, underpass floor) are ground too
                if (any && !ground && instance < 0 && normal[2] > 0.85 &&
                    (material == Palette::Asphalt || material == Palette::LaneWhite || material == Palette::LaneYellow))
                    ground = true;
                if (!any)
                    continue;

                Hit hit{};
                hit.range = tHit + config.rangeNoise * hashNoise(col, c, m_revolution);
                for (int k = 0; k < 3; ++k)
                    hit.p[k] = origin[k] + dir[k] * hit.range;
                hit.ground = ground;

                if (ground)
                {
                    // ignore returns from under the vehicle body
                    double l[3];
                    vehicle.worldToLocal(hit.p, l);
                    if (l[0] > -0.3 && l[0] < 4.3 && l[1] > -0.3 && l[1] < 1.3)
                        continue;
                    if (planeGround)
                        material = m_groundMaterial ? m_groundMaterial(hit.p[0], hit.p[1]) : Palette::Asphalt;
                }
                vehicle.worldToLocal(hit.p, hit.local);

                // intensity = reflectivity (with surface texture) x scattering(incidence) x range terms + noise
                const double cosI = std::abs(dir[0] * normal[0] + dir[1] * normal[1] + dir[2] * normal[2]);
                hit.intensity = static_cast<float>(computeIntensity(material, cosI, hit.range, c, hit.p,
                                                                    hashNoise(c, col, m_revolution + 7)));
                hit.ring = static_cast<unsigned char>(c);
                hit.material = static_cast<short>(material);
                hit.instance = static_cast<short>(instance);
                hit.onRoadFrame = false;
                if (!ground && m_project)
                {
                    double rs, rl;
                    if (m_project(hit.p[0], hit.p[1], rs, rl))
                    {
                        hit.onRoadFrame = true;
                        hit.roadS = static_cast<float>(rs);
                        hit.roadLat = static_cast<float>(rl);
                    }
                }

                out.push_back(hit);
            }
        }
    };

    // persistent pool (created once): columns are split into equal parts
    if (!m_pool)
    {
        // leave two cores for the GPU driver / compositor / UI
        const int cores = static_cast<int>(std::thread::hardware_concurrency());
        // pipelined (GUI): the scan runs while the main thread renders -> leave one more core free
        int workers = std::clamp(cores - (m_pipelined ? 4 : 3), 0, 7);
        if (const char* env = std::getenv("LIDAR_THREADS"))   // experiments: total threads
            workers = std::clamp(std::atoi(env) - 1, 0, 15);
        m_pool = std::make_unique<ThreadPool>(workers);
    }
    if (count * nCh < 2000 || m_pool->size() == 1)
    {
        worker(0, count);
        return;
    }
    m_pool->run([&](int part, int parts) {
        worker(count * part / parts, count * (part + 1) / parts);
    });
}

void LidarSensor::rebuildHits()
{
    m_hits.clear();
    m_totalPoints = 0;
    for (const auto& col : m_columns)
    {
        m_totalPoints += col.size();
        for (const Hit& h : col)
            if (!h.ground)
                m_hits.push_back(h);
    }
}

bool LidarSensor::update(const VehicleModel& vehicle, const std::vector<DynamicBox>& boxes, double dt)
{
    const int nCol = columns();
    m_columnAccum += nCol * config.rotationHz * dt;
    int n = static_cast<int>(m_columnAccum);
    if (n <= 0)
        return false;
    n = std::min(n, nCol);
    m_columnAccum -= n;

    if (m_pipelined)
    {
        // remember the state of this moment; the background thread fires these columns later
        m_queue.push_back({ vehicle, boxes, m_nextColumn, n });
        if (m_nextColumn + n >= nCol)
            ++m_pendingRevolutions;
        m_nextColumn = (m_nextColumn + n) % nCol;
        m_pendingData = true;
        return true;
    }

    const auto t0 = std::chrono::steady_clock::now();
    scanColumns(vehicle, boxes, m_nextColumn, n);
    if (m_nextColumn + n >= nCol)
        ++m_revolution;
    m_nextColumn = (m_nextColumn + n) % nCol;
    rebuildHits();
    m_outputsDirty = true;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    m_scanMsAvg = 0.9 * m_scanMsAvg + 0.1 * ms;
    return true;
}

void LidarSensor::fullScan(const VehicleModel& vehicle, const std::vector<DynamicBox>& boxes)
{
    sync();
    scanColumns(vehicle, boxes, 0, columns());
    m_nextColumn = 0;
    ++m_revolution;
    rebuildHits();
    m_outputsDirty = true;
}

void LidarSensor::buildOutputs()
{
    sync();   // never while the background thread writes the columns
    // [0] ground returns (vehicle frame), [1] obstacle returns (world)
    vtkNew<vtkPoints> pts[2];
    vtkNew<vtkCellArray> verts[2];
    vtkNew<vtkUnsignedCharArray> colors[2];
    for (int k = 0; k < 2; ++k)
    {
        colors[k]->SetNumberOfComponents(3);
        colors[k]->SetName("Intensity");
    }

    for (const auto& column : m_columns)
    for (const Hit& h : column)
    {
        const int set = h.ground ? 0 : 1;
        const vtkIdType id = pts[set]->InsertNextPoint(h.ground ? h.local : h.p);
        verts[set]->InsertNextCell(1, &id);
        unsigned char rgb[3];
        intensityColor(displayValue(h.intensity), rgb);
        colors[set]->InsertNextTypedTuple(rgb);
    }

    vtkPolyData* outs[2] = { m_cloud, m_obstacleCloud };
    for (int k = 0; k < 2; ++k)
    {
        outs[k]->SetPoints(pts[k]);
        outs[k]->SetVerts(verts[k]);
        outs[k]->GetPointData()->SetScalars(colors[k]);
        outs[k]->Modified();
    }

    m_outputsDirty = false;
}
