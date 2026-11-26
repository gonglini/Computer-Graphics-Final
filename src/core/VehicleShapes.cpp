#include "VehicleShapes.h"

#include <algorithm>
#include <cmath>

#include "Palette.h"

namespace
{
using namespace SceneBuilder;

double smooth(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

bool in(double x, double a, double b) { return x >= a && x <= b; }

// ---------------------------------------------------------------- truck cab
constexpr double kCabLen = 1.25;
constexpr double kCabBottom = 0.18;

// roof line of the cab along x: hood -> windshield -> roof
double cabTop(double x)
{
    if (x < 0.30) return 0.82 + 0.08 * (x / 0.30);
    if (x < 0.68) return 0.90 + 0.50 * smooth((x - 0.30) / 0.38);
    return 1.40 - 0.03 * (x - 0.68) / (kCabLen - 0.68);
}

Grid truckCabBody()
{
    const int nx = 16, ny = 10, nz = 12;
    return morphedHex(nx, ny, nz,
        [](double u, double v, double w, double p[3]) {
            const double x = kCabLen * u;
            const double side = std::abs(2.0 * v - 1.0);
            // rounded front corners (plan view)
            p[0] = x + 0.08 * std::pow(side, 4.0) * std::pow(1.0 - u, 3.0);
            // slightly narrower toward the roof (tumblehome)
            const double hw = 0.5 - 0.06 * smooth((w - 0.75) / 0.25);
            p[1] = 0.5 + (2.0 * v - 1.0) * hw;
            p[2] = kCabBottom + w * (cabTop(x) - kCabBottom);
        },
        [=](int i, int j, int k, double uc, double vc, double wc) {
            const double x = kCabLen * uc;
            const double z = kCabBottom + wc * (cabTop(x) - kCabBottom);
            const bool front = (i == 0);
            const bool side = (j == 0 || j == ny - 1);
            const bool top = (k == nz - 1);

            if (k == 0) return int(Palette::Trim);
            if (front && in(z, 0.50, 0.68) && (vc < 0.28 || vc > 0.72)) return int(Palette::Headlight);
            if (front && in(z, 0.28, 0.66) && in(vc, 0.34, 0.66)) return int(Palette::Trim);  // grille
            if (top && in(x, 0.32, 0.66) && in(vc, 0.06, 0.94)) return int(Palette::Glass);     // windshield
            if (side && in(z, 0.98, 1.33) && in(x, 0.36, 1.17) && !in(x, 0.78, 0.87))
                return int(Palette::Glass);                                                     // side windows
            return int(Palette::Body);
        });
}

// ---------------------------------------------------------------- cargo box
constexpr double kCargoX0 = 1.32, kCargoX1 = 4.0, kCargoZ0 = 0.30, kCargoZ1 = 1.55;

Grid truckCargoBox()
{
    const int nx = 30, ny = 10, nz = 12;
    return morphedHex(nx, ny, nz,
        [](double u, double v, double w, double p[3]) {
            p[0] = kCargoX0 + (kCargoX1 - kCargoX0) * u;
            const double hw = 0.5 - 0.025 * smooth((w - 0.85) / 0.15);
            p[1] = 0.5 + (2.0 * v - 1.0) * hw;
            p[2] = kCargoZ0 + (kCargoZ1 - kCargoZ0) * w;
        },
        [=](int i, int j, int k, double, double vc, double wc) {
            const double z = kCargoZ0 + (kCargoZ1 - kCargoZ0) * wc;
            const bool rear = (i == nx - 1);
            const bool side = (j == 0 || j == ny - 1);
            if (k == 0) return int(Palette::Trim);
            if (rear && in(z, 0.40, 0.56) && (vc < 0.2 || vc > 0.8)) return int(Palette::Taillight);
            if (rear && in(vc, 0.45, 0.55)) return int(Palette::Trim);   // door seam
            if (side && k == 2) return int(Palette::Body);                 // stripe
            return int(Palette::Cargo);
        });
}

Grid wheels(const std::vector<double>& axlesX, double radius, double width, double zc)
{
    std::vector<std::array<double, 3>> c;
    for (double x : axlesX)
    {
        c.push_back({ x, -0.02, zc });
        c.push_back({ x, 1.02, zc });
    }
    return merge({ cylinderGroup(c, radius, width, Palette::Tire, 'Y', 24),
                   cylinderGroup(c, radius * 0.58, width + 0.02, Palette::Rim, 'Y', 16) });
}

// ---------------------------------------------------------------- sedan
double sedanTop(double x)
{
    if (x < 0.15) return 0.62 + 0.10 * (x / 0.15);
    if (x < 0.95) return 0.72 + 0.06 * (x - 0.15) / 0.80;
    if (x < 1.55) return 0.78 + 0.44 * smooth((x - 0.95) / 0.60);
    if (x < 2.75) return 1.22;
    if (x < 3.40) return 1.22 - 0.34 * smooth((x - 2.75) / 0.65);
    return 0.88 - 0.04 * (x - 3.40) / 0.60;
}
constexpr double kSedanBottom = 0.20;

// ---------------------------------------------------------------- generic car body
// Silhouette given by the roof line top(x); glass / lights placed by x and z ranges.
struct CarProfile
{
    double (*top)(double);
    double bottom, shoulder, roof, tumble;   // tumble: narrowing toward the roof
    double wx0, wx1;                         // windshield (on the top surface)
    double rx0, rx1;                         // rear window (top surface)
    bool rearFaceGlass;                      // hatchback / SUV: glass on the rear face
    double zWin, sx0, sx1, px0, px1;         // side windows above zWin from sx0 to sx1, B-pillar px0..px1
    double head0, head1, tail0, tail1;       // light heights
    int paint;
};

// part of the body between x0 and x1 (the two parts meet without a gap)
Grid carPart(const CarProfile& c, double x0, double x1, int nx, int ny, int nz)
{
    const bool front = x0 <= 0.0, rear = x1 >= 4.0;
    return morphedHex(nx, ny, nz,
        [=](double u, double v, double w, double p[3]) {
            const double x = x0 + (x1 - x0) * u;
            const double ug = x / 4.0;
            const double side = std::pow(std::abs(2.0 * v - 1.0), 4.0);
            p[0] = x + 0.12 * side * std::pow(1.0 - ug, 3.0) - 0.12 * side * std::pow(ug, 3.0);
            p[2] = c.bottom + w * (c.top(x) - c.bottom);
            const double hw = 0.5 - c.tumble * std::clamp((p[2] - c.shoulder) / (c.roof - c.shoulder), 0.0, 1.0);
            p[1] = 0.5 + (2.0 * v - 1.0) * hw;
        },
        [=](int i, int j, int k, double uc, double vc, double wc) {
            const double x = x0 + (x1 - x0) * uc;
            const double z = c.bottom + wc * (c.top(x) - c.bottom);
            const bool sideFace = (j == 0 || j == ny - 1);
            const bool topFace = (k == nz - 1);
            const bool frontFace = front && i == 0, rearFace = rear && i == nx - 1;
            if (k == 0) return int(Palette::Trim);
            if (frontFace && in(z, c.head0, c.head1) && (vc < 0.3 || vc > 0.7)) return int(Palette::Headlight);
            if (frontFace && in(vc, 0.3, 0.7) && z < c.head1) return int(Palette::Trim);   // grille
            if (rearFace && in(z, c.tail0, c.tail1) && (vc < 0.3 || vc > 0.7)) return int(Palette::Taillight);
            if (rearFace && c.rearFaceGlass && z > c.zWin + 0.04) return int(Palette::Glass);
            if (topFace && (in(x, c.wx0, c.wx1) || in(x, c.rx0, c.rx1))) return int(Palette::Glass);
            if (sideFace && z > c.zWin && in(x, c.sx0, c.sx1) && !in(x, c.px0, c.px1)) return int(Palette::Glass);
            return c.paint;
        });
}

double suvTop(double x)
{
    if (x < 0.55) return 0.86 + 0.08 * (x / 0.55);
    if (x < 1.15) return 0.94 + 0.52 * smooth((x - 0.55) / 0.60);
    if (x < 3.85) return 1.46 - 0.02 * (x - 1.15) / 2.70;
    return 1.44 - 0.10 * smooth((x - 3.85) / 0.15);
}

double sportsTop(double x)
{
    if (x < 0.25) return 0.44 + 0.10 * (x / 0.25);
    if (x < 1.30) return 0.54 + 0.08 * (x - 0.25) / 1.05;
    if (x < 1.85) return 0.62 + 0.36 * smooth((x - 1.30) / 0.55);
    if (x < 2.45) return 0.98;
    if (x < 3.60) return 0.98 - 0.26 * smooth((x - 2.45) / 1.15);
    return 0.72 - 0.02 * (x - 3.60) / 0.40;
}

// roof LiDAR at (lx, roof height R) and front camera at (cx, roof height Rc), like the truck's
Grid sensorPack(double lx, double R, double cx, double Rc)
{
    return merge({
        hexBlock({ lx - 0.10, 0.40, R - 0.03 }, { lx + 0.10, 0.60, R + 0.02 }, 1, 1, 1, Palette::Trim),   // base
        cylinderGroup({ { lx, 0.5, R + 0.09 } }, 0.13, 0.14, Palette::Trim, 'Z', 24),                      // puck
        cylinderGroup({ { lx, 0.5, R + 0.175 } }, 0.11, 0.03, Palette::Glass, 'Z', 24),
        hexBlock({ cx - 0.05, 0.44, Rc - 0.02 }, { cx + 0.05, 0.56, Rc + 0.06 }, 1, 1, 1, Palette::Trim), // camera
        cylinderGroup({ { cx - 0.07, 0.5, Rc + 0.02 } }, 0.04, 0.06, Palette::Glass, 'X', 16) });          // lens
}

VehicleShapes::Parts buildCar(const CarProfile& c, int nxTotal, int ny, int nz, double wheelR, double wheelW, double lx, double cx)
{
    VehicleShapes::Parts p;
    const int nHalf = nxTotal / 2;
    const double mirrorZ = c.shoulder + 0.03;
    p.cab = merge({ carPart(c, 0.0, 2.0, nHalf, ny, nz),
                    hexBlock({ -0.07, 0.05, c.bottom }, { 0.04, 0.95, c.bottom + 0.16 }, 1, 10, 2, Palette::Trim),
                    hexBlock({ c.wx0 + 0.05, -0.12, mirrorZ }, { c.wx0 + 0.15, 0.02, mirrorZ + 0.10 }, 1, 1, 1, Palette::Trim),
                    hexBlock({ c.wx0 + 0.05, 0.98, mirrorZ }, { c.wx0 + 0.15, 1.12, mirrorZ + 0.10 }, 1, 1, 1, Palette::Trim) });
    p.cargo = merge({ carPart(c, 2.0, 4.0, nxTotal - nHalf, ny, nz),
                      hexBlock({ 3.96, 0.05, c.bottom }, { 4.05, 0.95, c.bottom + 0.16 }, 1, 6, 1, Palette::Trim) });
    p.wheels = wheels({ 0.62, 3.0 }, wheelR, wheelW, wheelR - 0.1);
    p.axles = { 0.62, 3.0 };
    p.wheelZ = wheelR - 0.1;
    p.wheelR = wheelR;
    p.sensors = sensorPack(lx, c.top(lx), cx, c.top(cx));
    return p;
}

const CarProfile kSedan = { sedanTop, 0.20, 0.80, 1.22, 0.12, 0.97, 1.55, 2.75, 3.38, false,
                            0.84, 1.00, 3.30, 2.05, 2.25, 0.45, 0.68, 0.55, 0.80, Palette::EgoSilver };
const CarProfile kSuv = { suvTop, 0.30, 0.98, 1.46, 0.07, 0.57, 1.13, -1.0, -1.0, true,
                          1.00, 1.18, 3.75, 2.15, 2.32, 0.62, 0.82, 0.70, 1.00, Palette::EgoBlack };
const CarProfile kSports = { sportsTop, 0.12, 0.58, 0.98, 0.16, 1.32, 1.83, 2.50, 3.10, false,
                             0.64, 1.40, 2.90, 9.0, -9.0, 0.36, 0.50, 0.48, 0.64, Palette::Body };
}


namespace VehicleShapes
{

const Spec& spec(int type)
{
    static const Spec kSpecs[TypeCount] = {
        { "TRUCK",  u8"트럭 (3.5톤)", 3500.0, 190e3, 3.0, 4.50, 40.0,  8.0, 2.50,
          { 0.85, 0.5, 1.62 }, { 0.56, 0.5, 1.42 }, { 1.02, 0.27, 1.20 }, 650.0, 3200.0, 3800.0, 0.38, 1.05, 0.42 },
        { "SEDAN",  u8"세단",        1550.0, 135e3, 4.2, 0.70, 58.0,  9.5, 2.38,
          { 2.00, 0.5, 1.44 }, { 1.49, 0.5, 1.24 }, { 1.85, 0.27, 1.02 }, 750.0, 5800.0, 6500.0, 0.32, 0.55, 0.45 },
        { "SUV",    u8"SUV",         2200.0, 185e3, 3.8, 1.10, 52.0,  9.0, 2.38,
          { 2.00, 0.5, 1.67 }, { 1.09, 0.5, 1.48 }, { 1.45, 0.27, 1.24 }, 700.0, 5200.0, 6000.0, 0.37, 0.75, 0.46 },
        { "SPORTS", u8"스포츠카",    1450.0, 330e3, 8.0, 0.62, 80.0, 11.0, 2.38,
          { 2.15, 0.5, 1.20 }, { 1.79, 0.5, 1.00 }, { 1.94, 0.27, 0.80 }, 900.0, 7000.0, 7800.0, 0.34, 0.45, 0.48 },
    };
    return kSpecs[std::clamp(type, 0, int(TypeCount) - 1)];
}

Parts buildVehicle(int type)
{
    switch (type)
    {
    case TypeSedan:  return buildCar(kSedan, 36, 10, 10, 0.28, 0.18, 2.00, 1.60);
    case TypeSuv:    return buildCar(kSuv, 36, 10, 11, 0.36, 0.22, 2.00, 1.20);
    case TypeSports: return buildCar(kSports, 40, 10, 8, 0.30, 0.24, 2.15, 1.90);
    default:         return buildTruck();
    }
}

Truck buildTruck()
{
    Truck t;

    t.cab = merge({
        truckCabBody(),
        hexBlock({ -0.07, 0.03, 0.20 }, { 0.05, 0.97, 0.38 }, 1, 10, 2, Palette::Trim),  // bumper
        hexBlock({ 0.42, -0.13, 1.00 }, { 0.52, 0.00, 1.12 }, 1, 1, 1, Palette::Trim),   // mirrors
        hexBlock({ 0.42, 1.00, 1.00 }, { 0.52, 1.13, 1.12 }, 1, 1, 1, Palette::Trim) });

    t.cargo = merge({
        truckCargoBox(),
        hexBlock({ 0.15, 0.18, 0.14 }, { 3.95, 0.82, 0.30 }, 8, 2, 1, Palette::Trim),    // chassis
        hexBlock({ 3.94, 0.05, 0.18 }, { 4.04, 0.95, 0.32 }, 1, 6, 1, Palette::Trim) }); // rear bumper

    t.wheels = wheels({ 0.5, 2.7, 3.3 }, 0.30, 0.20, 0.20);
    t.axles = { 0.5, 2.7, 3.3 };
    t.wheelZ = 0.20;
    t.wheelR = 0.30;

    t.sensors = merge({
        hexBlock({ 0.75, 0.40, 1.37 }, { 0.95, 0.60, 1.42 }, 1, 1, 1, Palette::Trim),      // LiDAR base
        cylinderGroup({ { 0.85, 0.5, 1.49 } }, 0.13, 0.14, Palette::Trim, 'Z', 24),        // LiDAR puck
        cylinderGroup({ { 0.85, 0.5, 1.575 } }, 0.11, 0.03, Palette::Glass, 'Z', 24),
        hexBlock({ 0.62, 0.44, 1.38 }, { 0.72, 0.56, 1.46 }, 1, 1, 1, Palette::Trim),      // camera housing
        cylinderGroup({ { 0.60, 0.5, 1.42 } }, 0.04, 0.06, Palette::Glass, 'X', 16) });    // lens

    return t;
}

Grid buildSedan(int paint)
{
    const int nx = 18, ny = 6, nz = 7;
    Grid body = morphedHex(nx, ny, nz,
        [](double u, double v, double w, double p[3]) {
            const double x = 4.0 * u;
            const double side = std::pow(std::abs(2.0 * v - 1.0), 4.0);
            p[0] = x + 0.12 * side * std::pow(1.0 - u, 3.0) - 0.12 * side * std::pow(u, 3.0);
            p[2] = kSedanBottom + w * (sedanTop(x) - kSedanBottom);
            const double hw = 0.5 - 0.12 * std::clamp((p[2] - 0.80) / 0.42, 0.0, 1.0);
            p[1] = 0.5 + (2.0 * v - 1.0) * hw;
        },
        [=](int i, int j, int k, double uc, double vc, double wc) {
            const double x = 4.0 * uc;
            const double z = kSedanBottom + wc * (sedanTop(x) - kSedanBottom);
            const bool side = (j == 0 || j == ny - 1);
            const bool top = (k == nz - 1);
            if (k == 0) return int(Palette::Trim);
            if (i == 0 && in(z, 0.45, 0.68) && (vc < 0.3 || vc > 0.7)) return int(Palette::Headlight);
            if (i == 0 && in(vc, 0.3, 0.7) && z < 0.6) return int(Palette::Trim);
            if (i == nx - 1 && in(z, 0.55, 0.80) && (vc < 0.3 || vc > 0.7)) return int(Palette::Taillight);
            if (top && (in(x, 0.97, 1.55) || in(x, 2.75, 3.38))) return int(Palette::Glass);
            if (side && z > 0.84 && in(x, 1.0, 3.3) && !in(x, 2.05, 2.25)) return int(Palette::Glass);
            return paint;
        });

    return merge({ body, wheels({ 0.75, 3.25 }, 0.28, 0.18, 0.18) });
}

} // namespace VehicleShapes
