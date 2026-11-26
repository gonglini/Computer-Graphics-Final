#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// The road: a closed loop (~2.1 km) described by arc length s, divided into 6 lanes
// (3 per direction, right-hand traffic).
//
//   s = distance along the centerline in the player's driving direction, 0 .. length()
//   lat = signed lateral offset, + = to the RIGHT of the driving direction
//   ego-direction lanes  : lat +1.5, +4.5, +7.5   (lane 0 next to the median)
//   oncoming lanes       : lat -1.5, -4.5, -7.5
//
// Layout (clockwise seen from above, the player starts at s = 17 heading -x):
//   city      straight 420 m, 6 intersections (grid)          s   0 .. 420
//   corner    right turn R 110
//   forest    S-chicane (R 150, +-25 deg), obstacle course
//   corner
//   highway   long sweeping S (R 400, +-15 deg), guard rails, median barrier
//   corner
//   suburb    S-chicane (R 220, +-15 deg), houses
//   corner    -> back to the city
// The centerline is made of exact straight / circular-arc pieces; every query is analytic.
struct RoadModel
{
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr int kLanes = 3;           // per direction

    enum class Zone { City, Corner, Forest, Highway, Suburb, Ring };

    struct Piece
    {
        Zone zone;
        double s0, len;
        double x0, y0, h0;   // start pose
        double k;            // curvature (+ = left turn), 0 = straight
    };

    double laneWidth = 3.0;
    int lanes = kLanes;                 // per direction
    double halfWidth = kLanes * 3.0;   // median to the outer edge line
    double groundZ = -0.1;

    RoadModel();
    // An open street through control points (x, y): a centripetal Catmull-Rom spline, turned into
    // short circular arcs (exact queries as for the loop). Starts at the first point, ends at the last.
    static RoadModel spline(const std::vector<std::array<double, 2>>& points, int lanesPerDir, double laneW);
    bool closed() const { return m_closed; }

    // height of the road surface above the ground plane along s (bridges, ramps, underpasses);
    // empty profile = flat (the main loop)
    double elevation(double s) const;
    double grade(double s) const;   // d elevation / ds
    void setElevation(std::vector<double> z, double step);
    const std::vector<double>& elevationProfile() const { return m_elev; }
    double elevationStep() const { return m_elevStep; }

    double length() const { return m_length; }
    const std::vector<Piece>& pieces() const { return m_pieces; }

    // zone ranges [from, to) in s
    double zoneFrom(Zone z) const;
    double zoneTo(Zone z) const;
    Zone zoneAt(double s) const;

    double wrap(double s) const
    {
        if (!m_closed)
            return std::clamp(s, 0.0, m_length);
        s = std::fmod(s, m_length);
        return s < 0.0 ? s + m_length : s;
    }
    // a - b along the loop, in (-L/2, L/2]   (open street: plain difference)
    double diff(double a, double b) const
    {
        if (!m_closed)
            return a - b;
        double d = wrap(a - b);
        return d > 0.5 * m_length ? d - m_length : d;
    }

    // centerline pose at s
    void pose(double s, double& x, double& y, double& heading) const;
    // world point at (s, lat)
    void point(double s, double lat, double& x, double& y) const;
    // heading of a car driving along the road (dir +1: player direction, -1: oncoming)
    double heading(double s, int dir = 1) const;
    double curvature(double s) const;

    // world (x, y) -> (s, lat). Returns false if the point is farther than ~130 m from the road.
    bool project(double x, double y, double& s, double& lat) const;

    // lane centers
    double laneOffset(int lane) const { return (lane + 0.5) * laneWidth; }
    // ego-direction lane index of a lateral offset (clamped to 0 .. kLanes-1)
    int laneOf(double lat) const
    {
        return std::clamp(static_cast<int>(std::floor(lat / laneWidth)), 0, lanes - 1);
    }

    // centerline polygon (for inside / outside tests)
    bool insideLoop(double x, double y) const;
    void bounds(double b[4]) const { std::copy(m_bounds, m_bounds + 4, b); }

private:
    struct Empty {};
    explicit RoadModel(Empty) {}
    void addStraight(Zone z, double len);
    void addArc(Zone z, double radius, double angleDeg);   // angle > 0: left, < 0: right
    void addChicane(Zone z, double radius, double angleDeg, bool leftFirst);
    void buildIndex();
    std::vector<double> m_elev;
    double m_elevStep = 5.0;
    const Piece& pieceAt(double s) const;
    bool m_closed = true;

    std::vector<Piece> m_pieces;
    double m_length = 0.0;
    double m_cx = 0.0, m_cy = 0.0, m_ch = 0.0;   // construction cursor

    // projection index: centerline samples every 1 m + a coarse grid -> nearest sample
    std::vector<double> m_sx, m_sy;
    double m_ds = 1.0;
    double m_bounds[4] = { 0, 0, 0, 0 };   // centerline xmin xmax ymin ymax
    double m_gx0 = 0, m_gy0 = 0, m_cell = 8.0;
    int m_gnx = 0, m_gny = 0;
    std::vector<int> m_grid;
};
