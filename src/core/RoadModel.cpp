#include "RoadModel.h"

#include <limits>

namespace
{
constexpr double kDeg = RoadModel::kPi / 180.0;
constexpr double kProjectRange = 130.0;
}

RoadModel::RoadModel()
{
    // start: (20, 0) heading -x, the city straight
    m_cx = 20.0;
    m_cy = 0.0;
    m_ch = kPi;

    const double R = 110.0;
    addStraight(Zone::City, 420.0);
    addArc(Zone::Corner, R, -90.0);
    // forest: forward length 80 + 4 * 150 * sin 25
    addStraight(Zone::Forest, 50.0);
    addChicane(Zone::Forest, 150.0, 25.0, true);
    addStraight(Zone::Forest, 30.0);
    addArc(Zone::Corner, R, -90.0);
    // highway: forward length must equal the city straight (420)
    {
        const double chicaneFwd = 4.0 * 400.0 * std::sin(15.0 * kDeg);
        addStraight(Zone::Highway, 0.5 * (420.0 - chicaneFwd));
        addChicane(Zone::Highway, 400.0, 15.0, true);
        addStraight(Zone::Highway, 0.5 * (420.0 - chicaneFwd));
    }
    addArc(Zone::Corner, R, -90.0);
    // suburb: forward length must equal the forest
    {
        const double forestFwd = 80.0 + 4.0 * 150.0 * std::sin(25.0 * kDeg);
        const double chicaneFwd = 4.0 * 220.0 * std::sin(15.0 * kDeg);
        const double rest = forestFwd - chicaneFwd;
        addStraight(Zone::Suburb, 0.45 * rest);
        addChicane(Zone::Suburb, 220.0, 15.0, true);
        addStraight(Zone::Suburb, 0.55 * rest);
    }
    addArc(Zone::Corner, R, -90.0);

    buildIndex();
}

RoadModel RoadModel::spline(const std::vector<std::array<double, 2>>& P, int lanesPerDir, double laneW)
{
    RoadModel r{ Empty{} };
    r.m_closed = false;
    r.lanes = lanesPerDir;
    r.laneWidth = laneW;
    r.halfWidth = lanesPerDir * laneW;
    // dense samples of a centripetal Catmull-Rom spline (end points duplicated)
    std::vector<std::array<double, 2>> q;
    const int n = static_cast<int>(P.size());
    auto pt = [&](int i) { return P[std::clamp(i, 0, n - 1)]; };
    for (int i = 0; i + 1 < n; ++i)
    {
        const auto p0 = i == 0 ? std::array<double, 2>{ 2 * P[0][0] - P[1][0], 2 * P[0][1] - P[1][1] } : pt(i - 1);
        const auto p1 = pt(i), p2 = pt(i + 1);
        const auto p3 = i + 2 >= n ? std::array<double, 2>{ 2 * P[n - 1][0] - P[n - 2][0], 2 * P[n - 1][1] - P[n - 2][1] }
                                   : pt(i + 2);
        auto knot = [](const std::array<double, 2>& a, const std::array<double, 2>& b) {
            return std::sqrt(std::max(1e-9, std::hypot(b[0] - a[0], b[1] - a[1])));
        };
        const double t0 = 0.0, t1 = t0 + knot(p0, p1), t2 = t1 + knot(p1, p2), t3 = t2 + knot(p2, p3);
        const int m = std::max(4, static_cast<int>(std::hypot(p2[0] - p1[0], p2[1] - p1[1]) / 0.5));
        for (int k = 0; k < m; ++k)
        {
            const double t = t1 + (t2 - t1) * k / m;
            std::array<double, 2> o;
            for (int d = 0; d < 2; ++d)
            {
                const double a1 = (t1 - t) / (t1 - t0) * p0[d] + (t - t0) / (t1 - t0) * p1[d];
                const double a2 = (t2 - t) / (t2 - t1) * p1[d] + (t - t1) / (t2 - t1) * p2[d];
                const double a3 = (t3 - t) / (t3 - t2) * p2[d] + (t - t2) / (t3 - t2) * p3[d];
                const double b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
                const double b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
                o[d] = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
            }
            q.push_back(o);
        }
    }
    q.push_back(P[n - 1]);
    // resample every ~2 m along the curve, then one circular arc per sample interval
    std::vector<std::array<double, 2>> u{ q[0] };
    double acc = 0.0;
    for (size_t i = 1; i < q.size(); ++i)
    {
        acc += std::hypot(q[i][0] - q[i - 1][0], q[i][1] - q[i - 1][1]);
        if (acc >= 2.0 || i + 1 == q.size())
        {
            u.push_back(q[i]);
            acc = 0.0;
        }
    }
    auto headingAt = [&](size_t i) {
        const size_t a = i == 0 ? 0 : i - 1, b = std::min(u.size() - 1, i + 1);
        return std::atan2(u[b][1] - u[a][1], u[b][0] - u[a][0]);
    };
    r.m_cx = u[0][0];
    r.m_cy = u[0][1];
    r.m_ch = headingAt(0);
    for (size_t i = 1; i < u.size(); ++i)
    {
        const double len = std::hypot(u[i][0] - u[i - 1][0], u[i][1] - u[i - 1][1]);
        double dh = headingAt(i) - r.m_ch;
        while (dh > kPi) dh -= 2.0 * kPi;
        while (dh < -kPi) dh += 2.0 * kPi;
        if (std::abs(dh) < 1e-5)
            r.addStraight(Zone::Ring, len);
        else
        {
            // arc with this heading change and this chord length
            const double radius = len / (2.0 * std::sin(std::abs(dh) / 2.0));
            r.addArc(Zone::Ring, radius, dh * 180.0 / kPi);
        }
    }
    r.buildIndex();
    return r;
}

double RoadModel::elevation(double s) const
{
    if (m_elev.empty())
        return 0.0;
    const double u = wrap(s) / m_elevStep;
    const int n = static_cast<int>(m_elev.size());
    const int i = static_cast<int>(std::floor(u));
    const double f = u - i;
    auto at = [&](int k) { return m_closed ? m_elev[((k % n) + n) % n] : m_elev[std::clamp(k, 0, n - 1)]; };
    const double p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    // Catmull-Rom spline through the samples: smooth vertical curves, grades stay at the sample slopes
    return 0.5 * (2.0 * p1 + (-p0 + p2) * f + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * f * f * f);
}

double RoadModel::grade(double s) const
{
    if (m_elev.empty())
        return 0.0;
    return (elevation(s + 1.0) - elevation(s - 1.0)) / 2.0;
}

void RoadModel::setElevation(std::vector<double> z, double step)
{
    m_elev = std::move(z);
    m_elevStep = step;
}

void RoadModel::addStraight(Zone z, double len)
{
    m_pieces.push_back({ z, m_length, len, m_cx, m_cy, m_ch, 0.0 });
    m_cx += len * std::cos(m_ch);
    m_cy += len * std::sin(m_ch);
    m_length += len;
}

void RoadModel::addArc(Zone z, double radius, double angleDeg)
{
    const double a = angleDeg * kDeg;
    const double k = (a > 0 ? 1.0 : -1.0) / radius;
    const double len = std::abs(a) * radius;
    m_pieces.push_back({ z, m_length, len, m_cx, m_cy, m_ch, k });
    const double h1 = m_ch + a;
    m_cx += (std::sin(h1) - std::sin(m_ch)) / k;
    m_cy -= (std::cos(h1) - std::cos(m_ch)) / k;
    m_ch = h1;
    m_length += len;
}

void RoadModel::addChicane(Zone z, double radius, double angleDeg, bool leftFirst)
{
    // +a, -2a, +a: heading and lateral position are back where they started
    const double sgn = leftFirst ? 1.0 : -1.0;
    addArc(z, radius, sgn * angleDeg);
    addArc(z, radius, -2.0 * sgn * angleDeg);
    addArc(z, radius, sgn * angleDeg);
}

const RoadModel::Piece& RoadModel::pieceAt(double s) const
{
    s = wrap(s);
    auto it = std::upper_bound(m_pieces.begin(), m_pieces.end(), s,
                               [](double v, const Piece& p) { return v < p.s0; });
    return it == m_pieces.begin() ? m_pieces.front() : *(it - 1);
}

void RoadModel::pose(double s, double& x, double& y, double& heading) const
{
    s = wrap(s);
    const Piece& p = pieceAt(s);
    const double t = s - p.s0;
    if (p.k == 0.0)
    {
        x = p.x0 + t * std::cos(p.h0);
        y = p.y0 + t * std::sin(p.h0);
        heading = p.h0;
        return;
    }
    heading = p.h0 + p.k * t;
    x = p.x0 + (std::sin(heading) - std::sin(p.h0)) / p.k;
    y = p.y0 - (std::cos(heading) - std::cos(p.h0)) / p.k;
}

void RoadModel::point(double s, double lat, double& x, double& y) const
{
    double h;
    pose(s, x, y, h);
    // right-hand normal of the driving direction
    x += lat * std::sin(h);
    y -= lat * std::cos(h);
}

double RoadModel::heading(double s, int dir) const
{
    double x, y, h;
    pose(s, x, y, h);
    return dir > 0 ? h : h + kPi;
}

double RoadModel::curvature(double s) const { return pieceAt(s).k; }

double RoadModel::zoneFrom(Zone z) const
{
    for (const auto& p : m_pieces)
        if (p.zone == z)
            return p.s0;
    return 0.0;
}

double RoadModel::zoneTo(Zone z) const
{
    double to = 0.0;
    bool seen = false;
    for (const auto& p : m_pieces)
    {
        if (p.zone == z)
        {
            to = p.s0 + p.len;
            seen = true;
        }
        else if (seen)
            break;
    }
    return to;
}

RoadModel::Zone RoadModel::zoneAt(double s) const { return pieceAt(s).zone; }

void RoadModel::buildIndex()
{
    const int n = static_cast<int>(std::ceil(m_length / 1.0)) + (m_closed ? 0 : 1);
    m_ds = m_length / (m_closed ? n : n - 1);
    m_sx.resize(n);
    m_sy.resize(n);
    m_bounds[0] = m_bounds[2] = 1e18;
    m_bounds[1] = m_bounds[3] = -1e18;
    for (int i = 0; i < n; ++i)
    {
        double h;
        pose(i * m_ds, m_sx[i], m_sy[i], h);
        m_bounds[0] = std::min(m_bounds[0], m_sx[i]);
        m_bounds[1] = std::max(m_bounds[1], m_sx[i]);
        m_bounds[2] = std::min(m_bounds[2], m_sy[i]);
        m_bounds[3] = std::max(m_bounds[3], m_sy[i]);
    }

    // grid cell -> nearest centerline sample (only cells within the projection range)
    const double margin = kProjectRange + m_cell;
    m_gx0 = m_bounds[0] - margin;
    m_gy0 = m_bounds[2] - margin;
    m_gnx = static_cast<int>(std::ceil((m_bounds[1] - m_bounds[0] + 2 * margin) / m_cell));
    m_gny = static_cast<int>(std::ceil((m_bounds[3] - m_bounds[2] + 2 * margin) / m_cell));
    m_grid.assign(static_cast<size_t>(m_gnx) * m_gny, -1);
    std::vector<double> best(m_grid.size(), margin * margin);
    const int reach = static_cast<int>(std::ceil(margin / m_cell));
    for (int i = 0; i < n; ++i)
    {
        const int ci = static_cast<int>((m_sx[i] - m_gx0) / m_cell);
        const int cj = static_cast<int>((m_sy[i] - m_gy0) / m_cell);
        for (int gj = std::max(0, cj - reach); gj <= std::min(m_gny - 1, cj + reach); ++gj)
            for (int gi = std::max(0, ci - reach); gi <= std::min(m_gnx - 1, ci + reach); ++gi)
            {
                const double cx = m_gx0 + (gi + 0.5) * m_cell, cy = m_gy0 + (gj + 0.5) * m_cell;
                const double d = (cx - m_sx[i]) * (cx - m_sx[i]) + (cy - m_sy[i]) * (cy - m_sy[i]);
                const size_t idx = static_cast<size_t>(gj) * m_gnx + gi;
                if (d < best[idx])
                {
                    best[idx] = d;
                    m_grid[idx] = i;
                }
            }
    }
}

bool RoadModel::project(double x, double y, double& s, double& lat) const
{
    const int gi = static_cast<int>(std::floor((x - m_gx0) / m_cell));
    const int gj = static_cast<int>(std::floor((y - m_gy0) / m_cell));
    if (gi < 0 || gj < 0 || gi >= m_gnx || gj >= m_gny)
        return false;
    const int i0 = m_grid[static_cast<size_t>(gj) * m_gnx + gi];
    if (i0 < 0)
        return false;

    // nearest sample around the cell's nearest sample (the cell is 8 m: +-14 samples is enough)
    const int n = static_cast<int>(m_sx.size());
    int bi = i0;
    double bd = std::numeric_limits<double>::max();
    for (int k = -14; k <= 14; ++k)
    {
        if (!m_closed && (i0 + k < 0 || i0 + k >= n))
            continue;
        const int i = ((i0 + k) % n + n) % n;
        const double d = (x - m_sx[i]) * (x - m_sx[i]) + (y - m_sy[i]) * (y - m_sy[i]);
        if (d < bd)
        {
            bd = d;
            bi = i;
        }
    }
    // refine on the two neighbouring chords
    double bestS = bi * m_ds, bestD = bd;
    for (int side : { -1, 1 })
    {
        if (!m_closed && (bi + side < 0 || bi + side >= n))
            continue;
        const int j = ((bi + side) % n + n) % n;
        const double ax = m_sx[bi], ay = m_sy[bi];
        const double vx = m_sx[j] - ax, vy = m_sy[j] - ay;
        const double vv = vx * vx + vy * vy;
        if (vv <= 0.0)
            continue;
        const double t = std::clamp(((x - ax) * vx + (y - ay) * vy) / vv, 0.0, 1.0);
        const double px = ax + t * vx, py = ay + t * vy;
        const double d = (x - px) * (x - px) + (y - py) * (y - py);
        if (d < bestD)
        {
            bestD = d;
            bestS = (bi + side * t) * m_ds;
        }
    }
    s = wrap(bestS);
    double cx, cy, h;
    pose(s, cx, cy, h);
    // refine on the exact centerline (the chords are 1 m long; far from a curve the error grows):
    // move s until (p - c(s)) is perpendicular to the tangent
    for (int it = 0; it < 2; ++it)
    {
        const double along = (x - cx) * std::cos(h) + (y - cy) * std::sin(h);
        const double left = -(x - cx) * std::sin(h) + (y - cy) * std::cos(h);
        const double k = curvature(s);
        s = wrap(s + along / std::max(0.2, 1.0 - k * left));
        pose(s, cx, cy, h);
    }
    lat = (x - cx) * std::sin(h) - (y - cy) * std::cos(h);
    if (!m_closed)
    {
        // beyond the ends of an open street the perpendicular offset means nothing
        const double along = (x - cx) * std::cos(h) + (y - cy) * std::sin(h);
        if ((s <= 1e-6 && along < -0.5) || (s >= m_length - 1e-6 && along > 0.5))
            return false;
    }
    return std::abs(lat) < kProjectRange;
}

bool RoadModel::insideLoop(double x, double y) const
{
    // even-odd rule on the centerline samples
    bool inside = false;
    const size_t n = m_sx.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        if (((m_sy[i] > y) != (m_sy[j] > y)) &&
            (x < (m_sx[j] - m_sx[i]) * (y - m_sy[i]) / (m_sy[j] - m_sy[i]) + m_sx[i]))
            inside = !inside;
    }
    return inside;
}
