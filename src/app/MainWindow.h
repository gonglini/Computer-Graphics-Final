#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QSet>
#include <QString>
#include <QTimer>

#include <array>
#include <map>
#include <utility>
#include <vector>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "core/DataRecorder.h"
#include "core/DriveSim.h"
#include "Quaternion.h"

#include <QPoint>

class QAction;
class QComboBox;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QTreeWidget;
class QTreeWidgetItem;
class QVTKOpenGLNativeWidget;

class vtkActor;
class vtkCallbackCommand;
class vtkCellPicker;
class vtkGenericOpenGLRenderWindow;
class vtkLight;
class vtkLookupTable;
class vtkMatrix4x4;
class vtkObject;
class vtkPolyData;
class vtkRegularPolygonSource;
class vtkUnsignedCharArray;
class vtkRenderPass;
class vtkRenderer;
class vtkScalarBarActor;
class vtkSkybox;
class vtkTextActor;
class vtkTexture;
class vtkTransform;

class PidActorManager;

// Scripted run for screenshots / recording (command line options, see main.cpp)
struct DemoOptions
{
    bool enabled = false;
    bool autopilot = false;
    bool throttle = false;          // hold W
    double steer = 0.0;             // -1..1 held steering (+ = left)
    bool hideLidar = false;
    bool edges = false;     // mesh edges on
    int selectPid = -1;     // PID highlighted (picking demo)
    double orbitYawDeg = 0.0;       // initial user camera rotation (as if dragged)
    double orbitPitchDeg = 0.0;
    double zoom = 1.0;              // camera distance factor (wide shots)
    int camera = 0;
    bool shadows = true;
    bool pbr = true;
    bool hq = true;                 // SSAO + FXAA + fog / surface detail
    int startPlace = 0;             // 0 city, 1 obstacle course, 2 highway, 3 suburb
    int vehicle = 0;                // VehicleShapes::Type
    std::vector<std::pair<double, QString>> shots;  // (seconds after start, png path)
    // frame sequence for clips: from 'framesFrom' for 'framesDur' seconds at 15 fps into framesDir/fNNNN.png
    QString framesDir;
    double framesFrom = 0.0, framesDur = 0.0;
    double quitAfter = -1.0;
    bool record = false;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    void runDemo(const DemoOptions& demo);
    void setLidarStep(double deg)
    {
        m_sim.lidar.config.azimuthStepDeg = deg;
        m_sim.reset();
    }
    void saveScreenshot(const QString& path);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    void setSurfaceMode();
    void setWireframeMode();
    void onTreeItemChanged(QTreeWidgetItem* item, int column);
    void onCurrentTreeItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);

    void startMotion();
    void stopMotion();
    void resetMotion();
    void updateMotion();

    void setAutopilot(bool on);
    void setCameraMode(int mode);
    void setRecording(bool on);
    void setStartPlace(int index);
    void setVehicleType(int type);
    void applyVisualToggles();

private:
    enum CameraMode { CamChase = 0, CamTop = 1, CamFree = 2, CamCinematic = 3, CamFirst = 4 };

    // first person: driver's eye inside the cab (left-hand drive), cockpit props
    void setupCockpit();
    void updateCockpit();
    void setFirstPersonLook(bool on);
    std::array<double, 3> m_eyeLocal = { 1.02, 0.27, 1.20 };
    std::vector<vtkSmartPointer<vtkActor>> m_cockpitActors;
    vtkSmartPointer<vtkTransform> m_wheelTransform;
    int m_lastThirdPersonMode = 0;

    void setupUi();
    void setupGuide();
    void setupVtk();
    void setupPalette();
    void setupLidarVisuals();
    void setupTraffic();
    void setupHud();
    void setupFrontCamera();
    void setupLidarView();
    void setupMinimap();
    void setupEnvironment();
    void setupShadows();
    void setupGhost();

    void updateVehicleActors();
    void updateShadows();
    void updateMainCamera(double dt);
    void updateFrontCamera();
    void updateLidarView();
    void updateMinimap();
    void updateHud();
    void updateGhost();
    void updateSun();
    void refreshAll();

    void startCrashAnimation();
    void finishCrashAnimation();
    void showDamage(bool on);

    VehicleModel::Command manualCommand() const;

    void handlePick(int displayX, int displayY);
    void selectTreeItemByPid(int pid);

private:
    // Qt
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    QTreeWidget* m_treeWidget = nullptr;
    QDockWidget* m_guideDock = nullptr;
    QDockWidget* m_crashDock = nullptr;
    class QTextBrowser* m_crashBrowser = nullptr;
    void showCrashReport();
    void clearCrashReport();
    QAction* m_autopilotAction = nullptr;
    QAction* m_lidarAction = nullptr;
    QAction* m_skyAction = nullptr;
    QAction* m_shadowAction = nullptr;
    QAction* m_pbrAction = nullptr;
    QAction* m_hqAction = nullptr;
    QAction* m_ghostAction = nullptr;
    QAction* m_edgesAction = nullptr;
    QAction* m_textureAction = nullptr;
    // texture mapping (procedural asphalt / grass / concrete)
    void setupTextures();
    void applyTextures(bool on);
    vtkSmartPointer<class vtkTexture> m_texAsphalt, m_texGrass, m_texConcrete;
    QAction* m_helpAction = nullptr;
    QComboBox* m_cameraCombo = nullptr;
    QDoubleSpinBox* m_cruiseSpin = nullptr;
    QLabel* m_sectionLabel = nullptr;
    QLabel* m_stateLabel = nullptr;
    QString m_lastStateText;
    QString m_lastSectionText;

    // simulation
    DriveSim m_sim;
    QTimer* m_motionTimer = nullptr;
    double m_motionDt = 0.016;     // fallback for the very first frame
    double m_simAccum = 0.0;       // real time not yet simulated (fixed 10 ms steps)
    QElapsedTimer m_hudClock;      // HUD text is refreshed at 10 Hz
    QElapsedTimer m_cloudClock;    // LiDAR render geometry is rebuilt at ~30 Hz
    QSet<int> m_keys;
    QElapsedTimer m_frameClock;
    double m_fps = 0.0;
    // per-tick work time (simulation + render), for spotting stutter
    double m_workMsAvg = 0.0, m_workMsMax = 0.0, m_workMsMaxWindow = 0.0, m_intervalMsMax = 0.0,
           m_intervalMsMaxWindow = 0.0;
    int m_workWindowTicks = 0;
    double m_simMs = 0.0, m_cloudMs = 0.0, m_renderMs = 0.0;   // per-frame breakdown (smoothed)

    // VTK
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;         // layer 0: main scene
    vtkSmartPointer<vtkRenderer> m_overlayRenderer;  // layer 1: LiDAR points/beams, same camera + depth
    vtkSmartPointer<vtkRenderer> m_hudRenderer;      // layer 2: 2D overlay
    vtkSmartPointer<vtkRenderer> m_frontCamRenderer; // layer 2: inset
    vtkSmartPointer<vtkRenderer> m_lidarRenderer;    // layer 2: inset
    vtkSmartPointer<vtkRenderer> m_minimapRenderer;  // layer 2: inset

    vtkSmartPointer<vtkCellPicker> m_cellPicker;

    PidActorManager* m_pidActorManager = nullptr;
    vtkSmartPointer<vtkLookupTable> m_paletteLut;

    vtkSmartPointer<vtkTransform> m_vehicleTransform;

    // traffic
    std::vector<vtkSmartPointer<vtkTransform>> m_npcTransforms;
    std::vector<vtkSmartPointer<vtkActor>> m_npcActors;

    // LiDAR
    vtkSmartPointer<vtkActor> m_cloudActor;
    vtkSmartPointer<vtkActor> m_obstacleCloudActor;
    vtkSmartPointer<vtkActor> m_lookaheadActor;
    vtkSmartPointer<vtkActor> m_lidarViewCloudActor;
    vtkSmartPointer<vtkActor> m_lidarViewObstacleActor;
    vtkSmartPointer<vtkActor> m_lidarViewLookahead;
    vtkSmartPointer<vtkTransform> m_lookaheadTransform;

    // minimap
    vtkSmartPointer<vtkTransform> m_minimapEgoTransform;

    // HUD
    vtkSmartPointer<vtkTextActor> m_hudText;
    vtkSmartPointer<vtkTextActor> m_bannerText;
    vtkSmartPointer<vtkTextActor> m_helpText;
    vtkSmartPointer<vtkScalarBarActor> m_scalarBar;
    vtkSmartPointer<vtkLookupTable> m_damageLut;

    // front camera inset (from the original example)
    vtkSmartPointer<vtkLight> m_frontCamHeadlight;
    std::map<int, vtkSmartPointer<vtkActor>> m_frontCamActors;
    std::array<double, 3> m_frontCamMount = { 0.56, 0.5, 1.42 };
    double m_frontCamViewAngle = 60.0;
    double m_frontCamPitch = 8.0;

    // environment / effects
    vtkSmartPointer<vtkSkybox> m_skybox;
    vtkSmartPointer<vtkActor> m_terrainActor;
    vtkSmartPointer<vtkActor> m_waterActor;
    vtkSmartPointer<vtkRenderPass> m_ssaoPass;
    // fog (aerial perspective) + procedural surface detail, injected into the VTK shaders
    void setAtmosphere(vtkActor* actor, double detail, bool on);
    vtkSmartPointer<vtkLight> m_sunLight;
    vtkSmartPointer<vtkLight> m_headLight;
    vtkSmartPointer<vtkTexture> m_skyTexture;
    vtkSmartPointer<vtkTexture> m_iblTexture;

    // planar projected shadows (sun direction -> ground plane)
    vtkSmartPointer<vtkMatrix4x4> m_shadowMatrix;
    vtkSmartPointer<vtkTransform> m_vehicleShadowTransform;
    std::vector<vtkSmartPointer<vtkTransform>> m_npcShadowTransforms;
    std::vector<vtkSmartPointer<vtkActor>> m_shadowActors;
    double m_sunDir[3] = { 0.0, 0.0, 1.0 };   // unit vector toward the sun

    // ghost replay of the previous run
    vtkSmartPointer<vtkTransform> m_ghostTransform;
    std::vector<vtkSmartPointer<vtkActor>> m_ghostActors;
    std::vector<DriveSim::Pose> m_ghostPath;
    size_t m_ghostIndex = 0;

    // camera: quaternion arcball on top of a per-mode base frame
    //   orientation = frame (follows the vehicle heading / fixed in Free mode) * user rotation
    int m_cameraMode = CamChase;
    bool m_cameraInitialized = false;
    double m_camPos[3] = { 0, 0, 0 };
    double m_camFocal[3] = { 0, 0, 0 };
    double m_camUp[3] = { 0, 0, 1 };
    Quat m_userRot;                  // mouse rotation (arcball)
    Quat m_lastFrame;                // base frame used in the last update
    Quat m_freeFrame;                // fixed frame for Free mode
    double m_zoom = 1.0;
    bool m_dragging = false;
    bool m_dragMoved = false;
    QPoint m_lastMouse, m_pressMouse;
    QElapsedTimer m_idleClock;       // time since the last mouse interaction
    QAction* m_autoReturnAction = nullptr;

    void arcballDrag(const QPoint& from, const QPoint& to);
    void resetView();
    void pickAt(const QPoint& widgetPos);

    // data acquisition (dataset recording)
    DataRecorder m_recorder;
    QAction* m_recordAction = nullptr;
    QAction* m_recPcdAction = nullptr;
    QAction* m_recCamAction = nullptr;
    QAction* m_openDataAction = nullptr;
    QComboBox* m_startCombo = nullptr;
    vtkSmartPointer<vtkTextActor> m_frontCamLabel;
    int m_pendingImageFrame = -1;   // camera image to grab after the next Render()
    QString m_lastDataDir;
    void captureCameraImage(int frame);

    // scripted demo
    DemoOptions m_demo;
    double m_demoTime = 0.0;
    size_t m_demoShot = 0;

    // traffic light lamps (one polydata, cell colors switched every frame) + tunnel light strips
    void setupSignals();
    void updateSignals();
    vtkSmartPointer<vtkPolyData> m_lampPoly;
    std::vector<std::pair<int, int>> m_lampCells;   // first cell, cell count per lamp
    std::vector<int> m_lampLit;                     // cached state (-1 unknown, 0 off, 1 on)

    // vehicle selection
    QComboBox* m_vehicleCombo = nullptr;
    vtkSmartPointer<vtkTransform> m_cockpitTransform;
    std::vector<vtkSmartPointer<vtkRegularPolygonSource>> m_ringSources;
    std::vector<vtkSmartPointer<vtkUnstructuredGrid>> m_ghostGrids;
    void applyVehicleSpec();
    void applyStartPlace(int index);   // 0-3 main loop, 4 ring road before the viaduct, 5 before the underpass

    // crash
    double m_crashAnimTime = -1.0;
    double m_crashOrbit = 0.0;
    int m_lastFrameIdx = -1;
    double m_crashHeading = 0.0;   // car heading when it was wrecked (orbit frame)
    // impact effects: camera shake (amplitude from the crash pulse in g) and flying fragments
    double m_shakeTime = -1.0;
    double m_shakeAmp = 0.0;
    struct Debris
    {
        double p[3], v[3];
        double spin, phase, tilt;
        bool rest;
    };
    std::vector<Debris> m_debris;
    vtkSmartPointer<class vtkPolyData> m_debrisPoly;
    vtkSmartPointer<vtkActor> m_debrisActor;
    void setupDebris();
    // game-style instrument cluster + vignette (HUD renderer)
    void setupGauges();
    void updateGauges();
    vtkSmartPointer<class vtkPolyData> m_gaugePoly, m_vignettePoly;
    vtkSmartPointer<class vtkActor2D> m_gaugeActor, m_vignetteActor;
    vtkSmartPointer<class vtkTextActor> m_speedText, m_speedUnitText, m_gearText, m_rpmText, m_damageText;
    bool m_hudDebug = false;   // F3: technical lines (LiDAR, FPS, timings)
    // impacts while driving: effects + report; damage colors for a few seconds (or always with K)
    unsigned m_seenImpact = 0;
    QElapsedTimer m_reportClock;      // live refresh of the crash analysis panel
    double m_lastImpactTime = -100.0;
    bool m_damageShown = false;
    bool m_strainToggle = false;
    void onImpact();
    void updateDamageView();
    std::vector<class QAction*> m_vehicleActions;
    void spawnDebris();
    void clearDebris();
    void updateDebris(double dt);
    void applyCameraShake();
};
