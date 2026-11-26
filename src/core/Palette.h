#pragma once

// Material palette. Every mesh stores a palette index per cell in "ElementScalar";
// the application maps it through one indexed lookup table.
namespace Palette
{
enum Index
{
    Body = 0,     // truck cab paint
    Glass,
    Headlight,
    Cargo,        // cargo box paint
    Tire,
    Rim,
    Trim,         // bumpers, chassis, dark plastic
    Asphalt,
    LaneWhite,
    Pole,
    Barrel,
    Crate,
    WallRed,
    Concrete,
    Window,
    Brick,
    Trunk,
    Leaf,
    LeafDark,
    LampPost,
    Lamp,
    NpcBlue,
    NpcWhite,
    Taillight,
    Grass,
    NpcGreen,
    Sidewalk,
    SignalOff,    // unlit signal lamp
    SignalGreen,
    SignRed,
    LaneYellow,   // center (median) line
    Water,
    Roof,         // red tiles
    RoofGray,
    Stucco,       // house walls
    Metal,        // guard rails, gantry
    SignGreen,    // highway sign boards
    LeafLight,
    Jersey,       // concrete median barrier
    EgoSilver,    // player car paints (vehicle selection)
    EgoBlack,
    TunnelTile,   // tunnel walls (light ceramic tiles)
    Count
};

inline const double* color(int i)
{
    static const double kColors[Count][3] = {
        { 0.78, 0.10, 0.08 },  // Body
        { 0.10, 0.15, 0.22 },  // Glass
        { 1.00, 0.95, 0.78 },  // Headlight
        { 0.93, 0.76, 0.16 },  // Cargo
        { 0.07, 0.07, 0.08 },  // Tire
        { 0.72, 0.73, 0.76 },  // Rim
        { 0.20, 0.20, 0.22 },  // Trim
        { 0.31, 0.32, 0.34 },  // Asphalt
        { 0.95, 0.95, 0.92 },  // LaneWhite
        { 0.95, 0.48, 0.10 },  // Pole
        { 0.92, 0.30, 0.06 },  // Barrel
        { 0.58, 0.40, 0.20 },  // Crate
        { 0.82, 0.10, 0.10 },  // WallRed
        { 0.64, 0.64, 0.67 },  // Concrete
        { 0.30, 0.50, 0.68 },  // Window
        { 0.58, 0.32, 0.25 },  // Brick
        { 0.36, 0.23, 0.13 },  // Trunk
        { 0.18, 0.48, 0.18 },  // Leaf
        { 0.11, 0.34, 0.14 },  // LeafDark
        { 0.42, 0.42, 0.45 },  // LampPost
        { 1.00, 0.92, 0.62 },  // Lamp
        { 0.13, 0.32, 0.78 },  // NpcBlue
        { 0.90, 0.90, 0.92 },  // NpcWhite
        { 0.85, 0.04, 0.04 },  // Taillight
        { 0.30, 0.42, 0.22 },  // Grass
        { 0.14, 0.58, 0.36 },  // NpcGreen
        { 0.58, 0.57, 0.55 },  // Sidewalk
        { 0.18, 0.12, 0.10 },  // SignalOff
        { 0.25, 1.00, 0.45 },  // SignalGreen
        { 0.85, 0.08, 0.08 },  // SignRed
        { 0.96, 0.74, 0.10 },  // LaneYellow
        { 0.10, 0.22, 0.30 },  // Water
        { 0.55, 0.22, 0.16 },  // Roof
        { 0.30, 0.31, 0.34 },  // RoofGray
        { 0.88, 0.84, 0.74 },  // Stucco
        { 0.70, 0.72, 0.74 },  // Metal
        { 0.05, 0.42, 0.22 },  // SignGreen
        { 0.36, 0.58, 0.20 },  // LeafLight
        { 0.58, 0.57, 0.55 },  // Jersey
        { 0.62, 0.64, 0.68 },  // EgoSilver
        { 0.07, 0.08, 0.09 },  // EgoBlack
        { 0.80, 0.79, 0.74 },  // TunnelTile
    };
    return kColors[i];
}
}
