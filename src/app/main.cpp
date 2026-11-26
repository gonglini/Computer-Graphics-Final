#include <QApplication>
#include <QFont>
#include <QPalette>
#include <QStyleFactory>
#include <QCommandLineParser>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include <memory>

#include "MainWindow.h"

// Optional scripted demo (used for screenshots and recordings):
//   LidarDrive.exe --autopilot --camera 0 --shot 6:a.png --shot 12:b.png --quit 13
//   LidarDrive.exe --throttle --shot 5:crash.png

// Dark, flat "game launcher" look for the Qt chrome: Fusion style, dark palette, rounded controls,
// one accent color. Only the look of the window frame changes; the VTK scene is untouched.
static void applyModernTheme(QApplication& app)
{
    app.setStyle(QStyleFactory::create("Fusion"));
    QFont f(QStringLiteral("Malgun Gothic"), 9);
    f.setStyleStrategy(QFont::PreferAntialias);
    app.setFont(f);

    QPalette p;
    const QColor bg(0x12, 0x15, 0x1c), base(0x0d, 0x10, 0x16), panel(0x18, 0x1c, 0x25), text(0xe6, 0xea, 0xf2);
    const QColor accent(0x3b, 0x82, 0xf6);
    p.setColor(QPalette::Window, bg);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, panel);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, panel);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::ToolTipBase, panel);
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::Highlight, accent);
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::Link, QColor(0x60, 0xa5, 0xfa));
    p.setColor(QPalette::PlaceholderText, QColor(0x7a, 0x84, 0x94));
    p.setColor(QPalette::Disabled, QPalette::Text, QColor(0x5c, 0x64, 0x70));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x5c, 0x64, 0x70));
    app.setPalette(p);

    app.setStyleSheet(QStringLiteral(R"(
QMainWindow, QDialog { background: #12151c; }
QMenuBar { background: #0e1117; color: #cfd6e2; padding: 2px 6px; border-bottom: 1px solid #1f2532; }
QMenuBar::item { padding: 5px 12px; border-radius: 6px; background: transparent; }
QMenuBar::item:selected { background: #1f2735; color: #ffffff; }
QMenu { background: #161a22; color: #e6eaf2; border: 1px solid #2a3140; border-radius: 8px; padding: 6px; }
QMenu::item { padding: 6px 22px 6px 14px; border-radius: 6px; }
QMenu::item:selected { background: #2b3a55; }
QMenu::separator { height: 1px; background: #2a3140; margin: 5px 8px; }
QToolBar { background: #12151c; border: none; border-bottom: 1px solid #1f2532; padding: 4px 6px; spacing: 4px; }
QToolBar::separator { width: 1px; background: #2a3140; margin: 4px 6px; }
QToolButton { color: #d5dbe6; background: transparent; border: 1px solid transparent; border-radius: 7px;
              padding: 4px 9px; }
QToolButton:hover { background: #1f2735; border-color: #2c3647; }
QToolButton:pressed { background: #263248; }
QToolButton:checked { background: rgba(59,130,246,0.22); border-color: #3b82f6; color: #ffffff; }
QLabel { color: #aeb7c6; }
QComboBox, QSpinBox, QDoubleSpinBox { background: #1a1f29; color: #e6eaf2; border: 1px solid #2a3140;
              border-radius: 7px; padding: 3px 8px; min-height: 20px; }
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { border-color: #3b82f6; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView { background: #161a22; color: #e6eaf2; border: 1px solid #2a3140;
              selection-background-color: #2b3a55; outline: none; }
QDockWidget { color: #cfd6e2; titlebar-close-icon: none; }
QDockWidget::title { background: #161a22; padding: 7px 10px; border-bottom: 1px solid #222a38; text-align: left; }
QTreeWidget, QTreeView, QListView { background: #0f1218; color: #d8dee9; border: none; outline: none; }
QTreeView::item { padding: 3px 2px; border-radius: 5px; }
QTreeView::item:hover { background: #1a2130; }
QTreeView::item:selected { background: #24324a; color: #ffffff; }
QHeaderView::section { background: #161a22; color: #9aa4b2; border: none; padding: 5px; }
QTextBrowser, QTextEdit { background: #0f1218; color: #dfe5ee; border: none; selection-background-color: #2b3a55; }
QTabBar::tab { background: #12151c; color: #9aa4b2; padding: 6px 14px; border: none; border-top: 2px solid transparent; }
QTabBar::tab:selected { background: #161a22; color: #ffffff; border-top: 2px solid #3b82f6; }
QTabBar::tab:hover { color: #e6eaf2; }
QStatusBar { background: #0b0e13; color: #8f9aab; border-top: 1px solid #1f2532; }
QStatusBar QLabel { color: #8f9aab; padding: 0 6px; }
QScrollBar:vertical { background: transparent; width: 9px; margin: 2px; }
QScrollBar::handle:vertical { background: #2c3443; border-radius: 4px; min-height: 30px; }
QScrollBar::handle:vertical:hover { background: #3b4658; }
QScrollBar:horizontal { background: transparent; height: 9px; margin: 2px; }
QScrollBar::handle:horizontal { background: #2c3443; border-radius: 4px; min-width: 30px; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
QCheckBox { color: #d8dee9; spacing: 6px; }
QToolTip { background: #161a22; color: #e6eaf2; border: 1px solid #2f3a4d; border-radius: 6px; padding: 5px; }
QSplitter::handle { background: #12151c; }
QMainWindow::separator { background: #1a1f29; width: 3px; height: 3px; }
)"));
}

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication app(argc, argv);
    applyModernTheme(app);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addOption({ "autopilot", "Start with the autopilot enabled." });
    parser.addOption({ "throttle", "Hold full throttle (crash test)." });
    parser.addOption({ "camera", "Camera mode 0=Chase 1=Top 2=Free 3=Cinematic.", "mode", "0" });
    parser.addOption({ "no-shadows", "Disable shadows." });
    parser.addOption({ "steer", "Held steering -1..1 (+ = left).", "value", "0" });
    parser.addOption({ "hide-lidar", "Do not draw the LiDAR points / beams." });
    parser.addOption({ "lidar-step", "Horizontal LiDAR resolution in degrees (default 0.2 = VLP-32C @10 Hz).", "deg" });
    parser.addOption({ "orbit", "Initial camera rotation yaw,pitch in degrees (e.g. 120,20).", "yaw,pitch" });
    parser.addOption({ "zoom", "Camera distance factor (e.g. 6 for a wide shot).", "factor", "1" });
    parser.addOption({ "start", "Start place: 0 city, 1 obstacle course, 2 highway, 3 suburb, 4 ring road (viaduct), "
                                "5 ring road (underpass).", "place", "0" });
    parser.addOption({ "no-pbr", "Disable PBR materials." });
    parser.addOption({ "vehicle", "Vehicle: 0 truck, 1 sedan, 2 SUV, 3 sports car.", "type", "0" });
    parser.addOption({ "no-hq", "Disable SSAO / FXAA / fog." });
    parser.addOption({ "edges", "Show the mesh edges." });
    parser.addOption({ "select", "Highlight a PID (picking demo).", "pid" });
    parser.addOption({ "frames", "Save a 15 fps frame sequence: start:duration:dir", "spec" });
    parser.addOption({ "shot", "Save a screenshot: seconds:path.png (repeatable).", "spec" });
    parser.addOption({ "quit", "Quit after N seconds.", "seconds" });
    parser.addOption({ "record", "Record a dataset (F9) while the demo runs." });
    parser.process(app);

    // on the heap: the window owns the whole simulation (LiDAR buffers etc.), too big for the 1 MB stack
    auto window = std::make_unique<MainWindow>();
    MainWindow& w = *window;
    if (parser.isSet("lidar-step"))
        w.setLidarStep(parser.value("lidar-step").toDouble());
    w.show();

    const bool demo = parser.isSet("autopilot") || parser.isSet("throttle") || parser.isSet("shot") || parser.isSet("frames") || parser.isSet("select") || parser.isSet("edges") ||
                      parser.isSet("start") || parser.isSet("vehicle");
    if (demo)
    {
        DemoOptions opt;
        opt.autopilot = parser.isSet("autopilot");
        opt.throttle = parser.isSet("throttle");
        opt.steer = parser.value("steer").toDouble();
        opt.hideLidar = parser.isSet("hide-lidar");
        opt.edges = parser.isSet("edges");
        if (parser.isSet("select"))
            opt.selectPid = parser.value("select").toInt();
        if (parser.isSet("orbit"))
        {
            const QStringList yp = parser.value("orbit").split(',');
            opt.orbitYawDeg = yp.value(0).toDouble();
            opt.orbitPitchDeg = yp.value(1).toDouble();
        }
        opt.camera = parser.value("camera").toInt();
        opt.zoom = parser.value("zoom").toDouble();
        opt.shadows = !parser.isSet("no-shadows");
        opt.startPlace = parser.value("start").toInt();
        opt.vehicle = parser.value("vehicle").toInt();
        opt.pbr = !parser.isSet("no-pbr");
        opt.hq = !parser.isSet("no-hq");
        opt.record = parser.isSet("record");
        if (parser.isSet("frames"))
        {
            const QStringList f = parser.value("frames").split(':');
            if (f.size() >= 3)
            {
                opt.framesFrom = f[0].toDouble();
                opt.framesDur = f[1].toDouble();
                opt.framesDir = f.mid(2).join(':');
            }
        }
        for (const QString& s : parser.values("shot"))
        {
            const int c = s.indexOf(':');
            if (c > 0)
                opt.shots.push_back({ s.left(c).toDouble(), s.mid(c + 1) });
        }
        opt.quitAfter = parser.isSet("quit") ? parser.value("quit").toDouble() : -1.0;

        QTimer::singleShot(500, &w, [&w, opt]() { w.runDemo(opt); });
    }

    return app.exec();
}
