#include "DataRecorder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <sstream>

#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

#include "DriveSim.h"
#include "Palette.h"

namespace fs = std::filesystem;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

std::string frameName(int frame, const char* ext)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%06d%s", frame, ext);
    return buf;
}

const char* kindName(ObstacleKind k)
{
    switch (k)
    {
    case ObstacleKind::Barrel:       return "barrel";
    case ObstacleKind::Cone:         return "cone";
    case ObstacleKind::Crate:        return "crate";
    case ObstacleKind::Wall:         return "wall";
    case ObstacleKind::Building:     return "building";
    case ObstacleKind::Tree:         return "tree";
    case ObstacleKind::TrafficLight: return "traffic_light";
    case ObstacleKind::Sign:         return "sign";
    case ObstacleKind::StreetLight:  return "street_light";
    case ObstacleKind::ParkedCar:    return "parked_car";
    case ObstacleKind::Car:          return "car";
    case ObstacleKind::Guardrail:    return "guardrail";
    case ObstacleKind::TunnelWall:   return "tunnel_wall";
    case ObstacleKind::Pier:         return "pier";
    case ObstacleKind::Ground:       return "ground";
    }
    return "unknown";
}

// one point as written to disk
struct PointRec
{
    float x, y, z, intensity;
    uint16_t ring, semantic, instance;   // instance: traffic car index + 1, 0 = none
};

// 3D box label (LiDAR frame)
struct BoxRec
{
    const char* type;
    int id;
    double x, y, z, l, w, h, yaw;
};

// LiDAR frame -> world: sensor axes in world coordinates
struct SensorPose
{
    double origin[3];
    double heading;
};
}

const char* DataRecorder::className(int c)
{
    static const char* names[ClassCount] = { "unlabeled", "road", "lane_marking", "sidewalk", "terrain",
                                             "building", "vegetation", "pole", "traffic_sign", "car",
                                             "obstacle", "barrier", "water" };
    return c >= 0 && c < ClassCount ? names[c] : "unlabeled";
}

int DataRecorder::semanticClass(int material, bool ground, int instance)
{
    if (instance >= 0)
        return Car;
    if (ground)
    {
        switch (material)
        {
        case Palette::Asphalt:    return Road;
        case Palette::LaneWhite:
        case Palette::LaneYellow: return LaneMarking;
        case Palette::Sidewalk:   return Sidewalk;
        case Palette::Grass:      return Terrain;
        case Palette::Water:      return Water;
        }
        return Unlabeled;
    }
    switch (material)
    {
    case Palette::TunnelTile:                                                   return Barrier;
    case Palette::Concrete: case Palette::Brick: case Palette::Window:
    case Palette::Stucco: case Palette::Roof: case Palette::RoofGray:           return Building;
    case Palette::Leaf: case Palette::LeafDark: case Palette::Trunk:
    case Palette::LeafLight:                                                    return Vegetation;
    case Palette::Grass:                                                        return Terrain;   // hills
    case Palette::Metal: case Palette::Jersey:                                  return Barrier;
    case Palette::SignGreen: case Palette::LaneYellow:                          return TrafficSign;
    case Palette::LampPost: case Palette::Lamp: case Palette::Trim: case Palette::Pole: return Pole;
    case Palette::SignRed: case Palette::SignalOff: case Palette::SignalGreen:
    case Palette::LaneWhite:                                                    return TrafficSign;
    case Palette::NpcBlue: case Palette::NpcWhite: case Palette::NpcGreen:
    case Palette::Rim: case Palette::Body: case Palette::EgoSilver:
    case Palette::EgoBlack:                                                     return Car;
    case Palette::Barrel: case Palette::Crate:                                  return Obstacle;
    case Palette::WallRed:                                                      return Barrier;
    }
    return Unlabeled;
}

DataRecorder::DataRecorder() = default;

DataRecorder::~DataRecorder() { stop(); }

size_t DataRecorder::pending() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jobs.size();
}

std::string DataRecorder::lastError() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_error;
}

void DataRecorder::fail(const std::string& what)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_error.empty())
        m_error = what;
}

void DataRecorder::enqueue(std::function<void()> job)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_jobs.push_back(std::move(job));
    }
    m_cv.notify_one();
}

void DataRecorder::writerLoop()
{
    for (;;)
    {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this] { return m_quit || !m_jobs.empty(); });
            if (m_jobs.empty())
                return;   // quit and nothing left
            job = std::move(m_jobs.front());
        }
        job();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_jobs.pop_front();   // popped after the work: pending() counts the running job too
        }
    }
}

bool DataRecorder::start(const std::string& dirUtf8, const DriveSim& sim, const Options& options,
                         const CameraInfo* camera)
{
    stop();
    m_framesCsv.close();
    m_poses.close();
    m_events.close();
    m_framesCsv.clear();
    m_poses.clear();
    m_events.clear();
    m_options = options;
    m_dir = dirUtf8;
    m_frame = 0;
    m_dropped = 0;
    m_bytes = 0;
    m_error.clear();
    m_lastRevolution = sim.lidar.revolution();
    m_startTime = sim.time;
    m_timeOffset = 0.0;
    m_lastTime = -1.0;

    const fs::path root = fs::u8path(dirUtf8);
    std::error_code ec;
    for (const char* sub : { "velodyne", "labels", "pcd", "boxes", "image_2" })
    {
        const bool needed = (std::strcmp(sub, "velodyne") == 0 || std::strcmp(sub, "labels") == 0) ? options.kitti
                          : std::strcmp(sub, "pcd") == 0   ? options.pcd
                          : std::strcmp(sub, "boxes") == 0 ? options.boxes
                          : camera != nullptr;
        if (needed)
            fs::create_directories(root / sub, ec);
    }
    if (ec || !fs::is_directory(root))
    {
        m_error = "cannot create " + dirUtf8;
        return false;
    }

    m_framesCsv.open(root / "frames.csv");
    m_poses.open(root / "poses.txt");
    m_events.open(root / "events.csv");
    if (!m_framesCsv || !m_poses || !m_events)
    {
        m_error = "cannot write into " + dirUtf8;
        return false;
    }
    m_framesCsv << "frame,time_s,x,y,heading_rad,speed_mps,steer_rad,cmd_throttle,cmd_brake,cmd_steer_rad,"
                   "autopilot,ap_state,lane,station_m,lateral_m,points,boxes\n";
    m_events << "frame,time_s,event,object,speed_kmh,grade,x,y\n";

    writeMeta(sim, camera);

    m_quit = false;
    m_thread = std::thread([this] { writerLoop(); });
    m_active = true;
    return true;
}

void DataRecorder::stop()
{
    if (!m_thread.joinable())
    {
        m_active = false;
        return;
    }
    m_active = false;
    enqueue([this] {
        m_framesCsv.close();
        m_poses.close();
        m_events.close();
    });
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_quit = true;
    }
    m_cv.notify_one();
    m_thread.join();
}

void DataRecorder::writeMeta(const DriveSim& sim, const CameraInfo* camera)
{
    const LidarSensor& L = sim.lidar;
    std::ostringstream j;
    j.precision(6);
    j << "{\n";
    j << "  \"generator\": \"LidarDrive (VTK) simulator\",\n";
    j << "  \"frame_rate_hz\": " << L.config.rotationHz << ",\n";
    j << "  \"lidar\": {\n";
    j << "    \"model\": \"Velodyne VLP-32C (Ultra Puck), simulated\",\n";
    j << "    \"channels\": " << L.channels() << ",\n";
    j << "    \"elevations_deg\": [";
    for (size_t i = 0; i < L.config.elevationsDeg.size(); ++i)
        j << (i ? ", " : "") << L.config.elevationsDeg[i];
    j << "],\n";
    j << "    \"rotation_hz\": " << L.config.rotationHz << ",\n";
    j << "    \"azimuth_resolution_deg\": " << L.config.azimuthStepDeg << ",\n";
    j << "    \"max_range_m\": " << L.config.range << ",\n";
    j << "    \"range_noise_m\": " << L.config.rangeNoise << ",\n";
    j << "    \"mount_in_vehicle_frame\": [" << L.config.mount[0] << ", " << L.config.mount[1] << ", "
      << L.config.mount[2] << "],\n";
    j << "    \"frame\": \"x forward, y left, z up, origin at the sensor\",\n";
    j << "    \"intensity\": \"0..1 calibrated reflectivity (x255 = Velodyne units)\"\n";
    j << "  },\n";
    if (camera)
    {
        const double fy = 0.5 * camera->height / std::tan(0.5 * camera->vfovDeg * kPi / 180.0);
        j << "  \"camera\": {\n";
        j << "    \"image_size\": [" << camera->width << ", " << camera->height << "],\n";
        j << "    \"K\": [[" << fy << ", 0, " << 0.5 * camera->width << "], [0, " << fy << ", "
          << 0.5 * camera->height << "], [0, 0, 1]],\n";
        j << "    \"vertical_fov_deg\": " << camera->vfovDeg << ",\n";
        j << "    \"mount_in_vehicle_frame\": [" << camera->mount[0] << ", " << camera->mount[1] << ", "
          << camera->mount[2] << "],\n";
        j << "    \"pitch_down_deg\": " << camera->pitchDownDeg << ",\n";

        // T_cam_lidar (OpenCV camera: x right, y down, z forward)
        // vehicle frame: forward = -x, right = +y; LiDAR point p -> vehicle = diag(-1,-1,1) p + mount
        const double p = camera->pitchDownDeg * kPi / 180.0;
        const double fwd[3] = { -std::cos(p), 0.0, -std::sin(p) };
        const double right[3] = { 0.0, 1.0, 0.0 };
        const double down[3] = { std::sin(p), 0.0, -std::cos(p) };
        const double* rows[3] = { right, down, fwd };
        const double A[3] = { -1.0, -1.0, 1.0 };
        double T[3][4];
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                T[r][c] = rows[r][c] * A[c];
            T[r][3] = 0.0;
            for (int c = 0; c < 3; ++c)
                T[r][3] += rows[r][c] * (L.config.mount[c] - camera->mount[c]);
        }
        j << "    \"T_cam_lidar\": [";
        for (int r = 0; r < 3; ++r)
        {
            j << (r ? ", " : "") << "[";
            for (int c = 0; c < 4; ++c)
                j << (c ? ", " : "") << T[r][c];
            j << "]";
        }
        j << ", [0, 0, 0, 1]]\n";
        j << "  },\n";

        // KITTI style calib.txt as well
        std::ofstream calib(fs::u8path(m_dir) / "calib.txt");
        calib << "P2: " << fy << " 0 " << 0.5 * camera->width << " 0 0 " << fy << " " << 0.5 * camera->height
              << " 0 0 0 1 0\n";
        calib << "Tr_velo_to_cam:";
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                calib << " " << T[r][c];
        calib << "\n";
    }
    j << "  \"vehicle_frame\": \"front bumper at x=0, rear at x=4, forward = -x, right = +y, z up\",\n";
    j << "  \"world_frame\": \"z up, road along -x, ground at z=" << sim.world.road().groundZ << "\",\n";
    j << "  \"semantic_classes\": {";
    for (int c = 0; c < ClassCount; ++c)
        j << (c ? ", " : "") << "\"" << c << "\": \"" << className(c) << "\"";
    j << "},\n";
    j << "  \"files\": {\n";
    j << "    \"velodyne/*.bin\": \"float32 x, y, z, intensity (KITTI)\",\n";
    j << "    \"labels/*.label\": \"uint32 per point: semantic class (low 16 bits) | instance id << 16 "
         "(SemanticKITTI)\",\n";
    j << "    \"pcd/*.pcd\": \"PCD v0.7 binary: x y z intensity (float32) ring label (uint16)\",\n";
    j << "    \"boxes/*.txt\": \"type id x y z l w h yaw num_points (LiDAR frame, box center, yaw about z)\",\n";
    j << "    \"image_2/*.png\": \"front camera\",\n";
    j << "    \"poses.txt\": \"KITTI odometry: 3x4 row-major pose of the LiDAR in the world per frame\",\n";
    j << "    \"frames.csv\": \"per frame vehicle state and driver / autopilot commands\",\n";
    j << "    \"events.csv\": \"crashes\"\n";
    j << "  }\n";
    j << "}\n";

    std::ofstream meta(fs::u8path(m_dir) / "meta.json");
    meta << j.str();
}

int DataRecorder::poll(const DriveSim& sim)
{
    if (!m_active)
        return -1;
    const unsigned rev = sim.lidar.revolution();
    if (rev == m_lastRevolution)
        return -1;
    m_lastRevolution = rev;

    if (pending() > static_cast<size_t>(m_options.maxQueue))
    {
        ++m_dropped;   // the disk cannot keep up: skip this frame rather than stall the simulation
        return -1;
    }

    // the simulation was reset (R) while recording: continue the time line, note it in events.csv
    double t = sim.time - m_startTime + m_timeOffset;
    if (t < m_lastTime)
    {
        m_timeOffset += m_lastTime - t + 0.1;
        t = sim.time - m_startTime + m_timeOffset;
        char ev[128];
        std::snprintf(ev, sizeof(ev), "%d,%.3f,reset,,,,%.3f,%.3f\n", m_frame, t, sim.vehicle.x(), sim.vehicle.y());
        const std::string e(ev);
        enqueue([this, e] { m_events << e; });
    }
    m_lastTime = t;

    const int frame = m_frame++;
    const LidarSensor& L = sim.lidar;
    const VehicleModel& v = sim.vehicle;
    const double* m = L.config.mount;

    // ---- points (vehicle frame at measurement time -> LiDAR frame) ----
    auto points = std::make_shared<std::vector<PointRec>>();
    points->reserve(L.totalPoints() + 16);
    for (const auto& column : L.columnBuffer())
        for (const auto& h : column)
        {
            PointRec p;
            p.x = static_cast<float>(-(h.local[0] - m[0]));
            p.y = static_cast<float>(-(h.local[1] - m[1]));
            p.z = static_cast<float>(h.local[2] - m[2]);
            p.intensity = h.intensity;
            p.ring = h.ring;
            const int sem = semanticClass(h.material, h.ground, h.instance);
            p.semantic = static_cast<uint16_t>(sem);
            p.instance = static_cast<uint16_t>(h.instance + 1);
            points->push_back(p);
        }

    // ---- sensor pose ----
    double so[3];
    L.sensorWorldPosition(v, so);
    const double hd = v.heading();
    const double ch = std::cos(hd), sh = std::sin(hd);

    // ---- ground-truth boxes within 80 m (LiDAR frame) ----
    auto boxes = std::make_shared<std::vector<BoxRec>>();
    const double gz = sim.world.road().groundZ;
    auto addBox = [&](const char* type, int id, double cx, double cy, double z0, double z1, double l, double w,
                      double yawWorld) {
        const double dx = cx - so[0], dy = cy - so[1];
        if (dx * dx + dy * dy > 80.0 * 80.0)
            return;
        boxes->push_back({ type, id, ch * dx + sh * dy, -sh * dx + ch * dy, 0.5 * (z0 + z1) - so[2], l, w, z1 - z0,
                           wrap(yawWorld - hd) });
    };
    for (size_t i = 0; i < sim.npcs.size(); ++i)
    {
        const auto& n = sim.npcs[i];
        if (n.alive)
            addBox("Car", static_cast<int>(i), n.cx, n.cy, gz + n.z, 1.22 + n.z, 2.0 * DriveSim::kNpcHalfL,
                   2.0 * DriveSim::kNpcHalfW, n.heading);
    }
    for (const auto& b : sim.world.boxes())
    {
        if (b.kind == ObstacleKind::ParkedCar)
            addBox("ParkedCar", -1, b.cx, b.cy, gz, 1.25, 2.0 * b.hx, 2.0 * b.hy, b.yaw);
    }
    // loose course objects at their current pose (they may have been knocked away)
    for (const auto& pr : sim.world.props())
    {
        const bool lying = std::abs(std::sin(pr.tilt)) > 0.7;
        const double hz = lying ? pr.halfW : 0.5 * pr.height;
        const double len = lying ? pr.height : 2.0 * pr.halfL;
        const double yaw = lying ? pr.tiltDir + pr.spin : pr.yaw0 + pr.spin;
        const char* type = pr.kind == ObstacleKind::Crate ? "Crate" : pr.kind == ObstacleKind::Barrel ? "Barrel" : "TrafficCone";
        addBox(type, -1, pr.pos[0], pr.pos[1], pr.pos[2] - hz, pr.pos[2] + hz, len, 2.0 * pr.halfW, yaw);
    }

    // ---- per-frame text rows (written by the writer thread, in order) ----
    const RoadModel& road = sim.world.road();
    double egoS, lat;
    sim.egoRoadPosition(egoS, lat);
    const int lane = lat < 0.0 ? -1 : road.laneOf(lat);

    char row[512];
    std::snprintf(row, sizeof(row), "%d,%.3f,%.4f,%.4f,%.6f,%.4f,%.5f,%.3f,%.3f,%.5f,%d,%s,%d,%.2f,%.3f,%zu,%zu\n",
                  frame, t, v.x(), v.y(), hd, v.speed(), v.steer(), sim.lastCommand.throttle,
                  sim.lastCommand.brake, sim.lastCommand.steer, sim.autopilotEnabled ? 1 : 0,
                  sim.autopilotEnabled ? sim.pilot.stateName() : "MANUAL", lane, egoS, lat, points->size(),
                  boxes->size());
    char pose[512];
    // columns = LiDAR axes (x forward, y left, z up) in world coordinates
    std::snprintf(pose, sizeof(pose), "%.6f %.6f 0 %.6f %.6f %.6f 0 %.6f 0 0 1 %.6f\n", ch, -sh, so[0], sh, ch,
                  so[1], so[2]);
    std::string rowS(row), poseS(pose);

    const fs::path root = fs::u8path(m_dir);
    const Options opt = m_options;
    enqueue([this, frame, points, boxes, rowS, poseS, root, opt] {
        m_framesCsv << rowS;
        m_poses << poseS;
        const size_t n = points->size();

        if (opt.kitti)
        {
            std::vector<float> xyzi(4 * n);
            std::vector<uint32_t> labels(n);
            for (size_t i = 0; i < n; ++i)
            {
                const PointRec& p = (*points)[i];
                xyzi[4 * i] = p.x; xyzi[4 * i + 1] = p.y; xyzi[4 * i + 2] = p.z; xyzi[4 * i + 3] = p.intensity;
                labels[i] = static_cast<uint32_t>(p.semantic) | (static_cast<uint32_t>(p.instance) << 16);
            }
            std::ofstream bin(root / "velodyne" / frameName(frame, ".bin"), std::ios::binary);
            bin.write(reinterpret_cast<const char*>(xyzi.data()), xyzi.size() * sizeof(float));
            std::ofstream lab(root / "labels" / frameName(frame, ".label"), std::ios::binary);
            lab.write(reinterpret_cast<const char*>(labels.data()), labels.size() * sizeof(uint32_t));
            if (!bin || !lab)
                fail("write failed (disk full?)");
            m_bytes += xyzi.size() * sizeof(float) + labels.size() * sizeof(uint32_t);
        }

        if (opt.pcd)
        {
            std::ostringstream hdr;
            hdr << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity ring label\n"
                   "SIZE 4 4 4 4 2 2\nTYPE F F F F U U\nCOUNT 1 1 1 1 1 1\nWIDTH "
                << n << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << n << "\nDATA binary\n";
            std::vector<char> buf(n * 20);
            for (size_t i = 0; i < n; ++i)
            {
                const PointRec& p = (*points)[i];
                char* d = buf.data() + 20 * i;
                std::memcpy(d, &p.x, 4); std::memcpy(d + 4, &p.y, 4); std::memcpy(d + 8, &p.z, 4);
                std::memcpy(d + 12, &p.intensity, 4); std::memcpy(d + 16, &p.ring, 2);
                std::memcpy(d + 18, &p.semantic, 2);
            }
            std::ofstream pcd(root / "pcd" / frameName(frame, ".pcd"), std::ios::binary);
            const std::string h = hdr.str();
            pcd.write(h.data(), h.size());
            pcd.write(buf.data(), buf.size());
            if (!pcd)
                fail("write failed (disk full?)");
            m_bytes += h.size() + buf.size();
        }

        if (opt.boxes)
        {
            std::ofstream txt(root / "boxes" / frameName(frame, ".txt"));
            txt << "# type id x y z l w h yaw num_points   (LiDAR frame: x forward, y left, z up; m, rad)\n";
            for (const auto& b : *boxes)
            {
                // LiDAR points inside the box (10 cm margin)
                const double c = std::cos(b.yaw), s = std::sin(b.yaw);
                int count = 0;
                for (const auto& p : *points)
                {
                    const double dx = p.x - b.x, dy = p.y - b.y;
                    if (std::abs(c * dx + s * dy) < 0.5 * b.l + 0.1 && std::abs(-s * dx + c * dy) < 0.5 * b.w + 0.1 &&
                        std::abs(p.z - b.z) < 0.5 * b.h + 0.1)
                        ++count;
                }
                char line[256];
                std::snprintf(line, sizeof(line), "%s %d %.3f %.3f %.3f %.3f %.3f %.3f %.4f %d\n", b.type, b.id, b.x,
                              b.y, b.z, b.l, b.w, b.h, b.yaw, count);
                txt << line;
            }
        }
    });
    return frame;
}

void DataRecorder::addImage(int frame, int width, int height, std::vector<unsigned char> rgb)
{
    if (!m_active || width <= 0 || height <= 0 || rgb.size() < size_t(width) * height * 3)
        return;
    const fs::path file = fs::u8path(m_dir) / "image_2" / frameName(frame, ".png");
    auto data = std::make_shared<std::vector<unsigned char>>(std::move(rgb));
    enqueue([this, file, data, width, height] {
        vtkNew<vtkImageData> img;
        img->SetDimensions(width, height, 1);
        img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
        std::memcpy(img->GetScalarPointer(), data->data(), size_t(width) * height * 3);
        vtkNew<vtkPNGWriter> writer;
        writer->SetFileName(file.u8string().c_str());   // VTK expects UTF-8 file names
        writer->SetInputData(img);
        writer->Write();
        m_bytes += data->size() / 3;   // rough (compressed)
    });
}

void DataRecorder::logCrash(const DriveSim& sim)
{
    if (!m_active)
        return;
    char line[256];
    std::snprintf(line, sizeof(line), "%d,%.3f,crash,%s,%.1f,%d,%.3f,%.3f\n", std::max(0, m_frame - 1),
                  sim.time - m_startTime + m_timeOffset, kindName(sim.crash.kind), sim.report.speedKmh, sim.report.grade,
                  sim.vehicle.x(), sim.vehicle.y());
    std::string s(line);
    enqueue([this, s] { m_events << s; m_events.flush(); });
}
