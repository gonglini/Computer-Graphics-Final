#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class DriveSim;

// Dataset recorder: one frame per LiDAR revolution (10 Hz), written in widely used formats.
//
//   <dir>/
//     velodyne/000000.bin   KITTI: float32 x, y, z, intensity (0..1) per point, LiDAR frame
//     labels/000000.label   SemanticKITTI: uint32 per point = semantic class | (instance id << 16)
//     pcd/000000.pcd        PCD v0.7 binary (x y z intensity ring label) -> CloudCompare / PCL / Open3D
//     boxes/000000.txt      3D boxes in the LiDAR frame: class id x y z l w h yaw num_points
//     image_2/000000.png    front camera (GUI only)
//     poses.txt             KITTI odometry: 3x4 pose of the LiDAR in the world, one line per frame
//     frames.csv            time, vehicle state, driver input, autopilot state per frame
//     events.csv            crashes
//     meta.json             sensor spec, mounting, camera intrinsics, class table, file formats
//
// LiDAR frame (like KITTI's Velodyne): x forward, y left, z up, origin at the sensor.
// Every point is stored in the sensor pose at the moment it was measured (rolling shutter /
// motion distortion is kept, like raw data from a real spinning LiDAR).
//
// The simulation thread only copies the data; files are written by a background thread.
class DataRecorder
{
public:
    struct Options
    {
        bool kitti = true;    // velodyne/*.bin + labels/*.label
        bool pcd = true;      // pcd/*.pcd
        bool boxes = true;    // boxes/*.txt
        int maxQueue = 40;    // frames waiting for the disk; more -> drop (counted)
    };

    struct CameraInfo
    {
        int width = 0, height = 0;
        double vfovDeg = 60.0;          // vertical field of view
        double mount[3] = { 0, 0, 0 };  // vehicle frame
        double pitchDownDeg = 0.0;
    };

    // semantic classes written to labels/*.label and the pcd 'label' field
    enum Class
    {
        Unlabeled = 0, Road = 1, LaneMarking = 2, Sidewalk = 3, Terrain = 4, Building = 5,
        Vegetation = 6, Pole = 7, TrafficSign = 8, Car = 9, Obstacle = 10, Barrier = 11, Water = 12, ClassCount
    };
    static const char* className(int c);
    static int semanticClass(int material, bool ground, int instance);

    DataRecorder();
    ~DataRecorder();

    // creates <dir> (UTF-8 path) and its sub folders; camera may be null (headless)
    bool start(const std::string& dirUtf8, const DriveSim& sim, const Options& options,
               const CameraInfo* camera);
    // finishes all pending writes
    void stop();
    bool active() const { return m_active; }

    // call after every simulation step: captures a frame when the LiDAR completed a revolution.
    // Returns the new frame index, or -1.
    int poll(const DriveSim& sim);

    // camera image for frame 'frame' (RGB, rows bottom-up as VTK returns them)
    void addImage(int frame, int width, int height, std::vector<unsigned char> rgb);

    void logCrash(const DriveSim& sim);

    int frames() const { return m_frame; }
    int dropped() const { return m_dropped; }
    size_t pending() const;
    double megabytesWritten() const { return m_bytes.load() / (1024.0 * 1024.0); }
    const std::string& directory() const { return m_dir; }
    std::string lastError() const;

private:
    void enqueue(std::function<void()> job);
    void writerLoop();
    void writeMeta(const DriveSim& sim, const CameraInfo* camera);
    void fail(const std::string& what);

    Options m_options;
    std::string m_dir;
    bool m_active = false;
    int m_frame = 0;
    int m_dropped = 0;
    unsigned m_lastRevolution = 0;
    double m_startTime = 0.0;
    double m_timeOffset = 0.0;   // keeps the recording time increasing across simulation resets
    double m_lastTime = -1.0;

    std::ofstream m_framesCsv, m_poses, m_events;

    std::thread m_thread;
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<std::function<void()>> m_jobs;
    bool m_quit = false;
    std::atomic<unsigned long long> m_bytes{ 0 };
    std::string m_error;
};
