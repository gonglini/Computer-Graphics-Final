#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ThreadPool.h"
#include "VehicleModel.h"

#include <vtkGenericCell.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStaticCellLocator.h>


// Simulated spinning LiDAR modeled on the Velodyne Ultra Puck (VLP-32C).
//
// Like the real sensor it ROTATES: every update() only fires the azimuth columns swept since the
// previous update (10 Hz -> 36 deg per 10 ms). The cost is spread evenly over the frames and the
// point cloud is a rolling buffer of the last full revolution.
//
// Every return carries an INTENSITY (calibrated reflectivity 0..1, Velodyne 0..255):
//   material reflectivity (asphalt low ... retro-reflective paint / signs high)
//   x surface texture (value noise: asphalt aggregate and patches, worn paint, grass, leaves)
//   x scattering: diffuse ~cos, retro-reflective ~const, car paint / glass = specular spike head-on
//   x near-field / range terms x per-laser gain + range-dependent noise, quantized to 8 bit
//
// Ray casting: static geometry with vtkStaticCellLocator::IntersectWithLine, the ground analytically,
// moving cars as oriented boxes (slab test). Columns are split across CPU cores.
class LidarSensor
{
public:
    struct Config
    {
        // VLP-32C laser elevation angles [deg]: -25 .. +15, dense (0.33 deg) around the horizon
        std::vector<double> elevationsDeg = {
            -25.0, -15.639, -11.31, -8.843, -7.254, -6.148, -5.333, -4.667,
            -4.0, -3.667, -3.333, -3.0, -2.667, -2.333, -2.0, -1.667,
            -1.333, -1.0, -0.667, -0.333, 0.0, 0.333, 0.667, 1.0,
            1.333, 1.667, 2.333, 3.333, 4.667, 7.0, 10.333, 15.0 };
        double rotationHz = 10.0;        // 5 .. 20 Hz on the real sensor
        double azimuthStepDeg = 0.2;     // horizontal resolution at 10 Hz
        double range = 200.0;            // [m]
        double rangeNoise = 0.03;        // +-3 cm accuracy
        double mount[3] = { 0.85, 0.5, 1.62 };  // vehicle local coords (roof puck)
    };

    struct Hit
    {
        double p[3];       // world position
        double local[3];   // position in the vehicle frame at the moment it was measured
        double range;
        float intensity;   // 0..1 (x255 = Velodyne calibrated reflectivity)
        bool ground;
        unsigned char ring;  // laser channel 0..31 (0 = lowest beam)
        short material;      // Palette index of the surface that was hit (ground truth)
        short instance;      // traffic car index, -1 for static geometry
        bool onRoadFrame;    // roadS / roadLat valid (non-ground hits near the road)
        float roadS, roadLat;  // position in the road frame, computed by the worker threads
    };

    // moving object seen by the LiDAR (traffic car): oriented box
    struct DynamicBox
    {
        double cx, cy, heading, halfL, halfW, zMin, zMax;
        int material;      // Palette index (paint)
        int id;            // instance id (traffic car index)
    };

    LidarSensor();
    ~LidarSensor();

    // static geometry; its cell array "ElementScalar" holds the Palette material per cell
    void setObstacleSurface(vtkPolyData* surface);
    void setGroundZ(double z) { m_groundZ = z; }
    // material of the ground at (x, y): asphalt, lane paint, grass ...
    void setGroundMaterial(std::function<int(double, double)> fn) { m_groundMaterial = std::move(fn); }
    // places where the ground plane is missing (open trench of an underpass): rays go past it
    void setGroundHole(std::function<bool(double, double)> fn) { m_groundHole = std::move(fn); }
    // a road laid on the hills (returns are ground, asphalt)
    void setRoadOnHills(std::function<bool(double, double)> fn) { m_roadOnHills = std::move(fn); }

    // hills: regular height grid (height above the ground plane), intersected by ray marching
    // (much cheaper than putting thousands of large terrain triangles into the cell locator)
    struct HeightField
    {
        double x0 = 0, y0 = 0, step = 10;
        int nx = 0, ny = 0;
        std::vector<float> h;   // nx * ny, row-major in y
        float maxH = 0;
        // coarse tiles (4 x 4 cells) with their maximum height: flat ground is skipped quickly
        int tnx = 0, tny = 0;
        std::vector<float> tileMax;
        double at(double x, double y) const;
        double tileMaxAt(double x, double y) const;
        void buildTiles();
    };
    // world (x, y) -> road frame (station, lateral); evaluated for every obstacle return on the
    // worker threads so perception does not have to
    void setRoadProjector(std::function<bool(double, double, double&, double&)> fn) { m_project = std::move(fn); }

    void setHeightField(HeightField hf)
    {
        m_height = std::move(hf);
        m_height.buildTiles();
    }

    // forget everything (empty cloud, sensor angle 0)
    void clear();
    // rotate the sensor by dt and fire the swept columns; returns true if new data arrived
    bool update(const VehicleModel& vehicle, const std::vector<DynamicBox>& dynamicBoxes, double dt);
    // one full revolution at once (start-up)
    void fullScan(const VehicleModel& vehicle, const std::vector<DynamicBox>& dynamicBoxes);

    // Pipelined mode (GUI): update() only records which columns to fire with the vehicle / traffic
    // state of that moment; kick() scans them on a background thread while the frame is rendered;
    // sync() waits for that work and publishes the results (hits, revolution counter, cloud).
    // Perception therefore sees data that is one frame old, like a real sensor pipeline.
    void setPipelined(bool on);
    bool pipelined() const { return m_pipelined; }
    void kick();
    bool sync();

    // NON-ground returns of the last full revolution (what perception needs)
    const std::vector<Hit>& hits() const { return m_hits; }
    // all returns (ground + obstacles) of the last revolution
    size_t totalPoints() const { return m_totalPoints; }
    // rolling buffer (one revolution, per azimuth column) and the revolution counter:
    // a data recorder grabs one frame each time revolution() changes
    const std::vector<std::vector<Hit>>& columnBuffer() const { return m_columns; }
    unsigned revolution() const { return m_revolution; }

    // rebuild the render geometry from the rolling buffer (call at most once per rendered frame)
    void buildOutputs();
    bool outputsDirty() const { return m_outputsDirty; }

    // ground returns in the VEHICLE frame (draw with the vehicle transform), colored by intensity
    vtkPolyData* cloud() const { return m_cloud; }
    // obstacle returns in world coords, colored by intensity
    vtkPolyData* obstacleCloud() const { return m_obstacleCloud; }

    int channels() const { return static_cast<int>(config.elevationsDeg.size()); }
    int columns() const;
    int rayCount() const { return channels() * columns(); }               // per revolution
    double pointsPerSecond() const { return rayCount() * config.rotationHz; }
    double lastScanMs() const { return m_scanMsAvg; }                      // per update()
    void sensorWorldPosition(const VehicleModel& vehicle, double p[3]) const;

    // VeloView-like intensity color map (blue = weak ... red = strong), t = displayValue(intensity)
    static void intensityColor(double t, unsigned char rgb[3]);
    // intensity 0..1 -> color map position (stretches the diffuse range 0..0.6)
    static double displayValue(double intensity);
    static double reflectivity(int material);
    // full return model: material reflectivity x surface texture x scattering type (diffuse / retro /
    // glossy / mirror) at the incidence angle x near/far range terms x per-laser gain + noise, 8 bit
    static double computeIntensity(int material, double cosI, double range, int ring, const double p[3],
                                   double noise);

    Config config;

private:
    void scanColumns(const VehicleModel& vehicle, const std::vector<DynamicBox>& boxes,
                     int firstColumn, int count);
    void rebuildHits();

    vtkSmartPointer<vtkStaticCellLocator> m_locator;
    // coarse 2D occupancy (empty-space skipping for the locator queries)
    double occupancyTop(double x, double y) const;
    double m_occX0 = 0, m_occY0 = 0, m_occCell = 8.0;
    int m_occNx = 0, m_occNy = 0;
    std::vector<float> m_occTop;
    std::unique_ptr<ThreadPool> m_pool;
    std::vector<int> m_cellMaterial;     // per obstacle-surface cell
    std::vector<float> m_cellNormal;     // 3 per cell
    double m_bounds[6] = { -1e9, 1e9, -1e9, 1e9, -1e9, 1e9 };   // obstacle surface bounds
    std::function<int(double, double)> m_groundMaterial;
    std::function<bool(double, double)> m_groundHole;
    std::function<bool(double, double)> m_roadOnHills;
    HeightField m_height;
    std::function<bool(double, double, double&, double&)> m_project;

    vtkSmartPointer<vtkPolyData> m_cloud;
    vtkSmartPointer<vtkPolyData> m_obstacleCloud;

    std::vector<std::vector<Hit>> m_columns;   // rolling buffer: one revolution

    struct Job
    {
        VehicleModel vehicle;
        std::vector<DynamicBox> boxes;
        int first, count;
    };
    void asyncLoop();
    bool m_pipelined = false;
    std::vector<Job> m_queue, m_running;
    int m_pendingRevolutions = 0;
    bool m_pendingData = false;
    std::thread m_asyncThread;
    std::mutex m_asyncMutex;
    std::condition_variable m_asyncCv;
    bool m_asyncBusy = false, m_asyncQuit = false;
    std::vector<Hit> m_hits;
    size_t m_totalPoints = 0;
    int m_nextColumn = 0;
    double m_columnAccum = 0.0;
    unsigned m_revolution = 0;
    bool m_outputsDirty = true;
    double m_groundZ = -0.1;
    double m_scanMsAvg = 0.0;
};
