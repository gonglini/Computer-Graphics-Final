#include "CrashDeformer.h"

#include <algorithm>
#include <cmath>

#include <vtkCell.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkGenericCell.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkUnstructuredGrid.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double smooth(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// smooth pseudo-random field in [-1, 1] (dents a few decimeters wide, no single-node spikes)
double dentField(const double p[3])
{
    return std::sin(p[0] * 7.3 + p[1] * 5.1 + 0.4) * std::sin(p[2] * 6.7 + p[1] * 3.3 + 1.2) *
           std::cos(p[0] * 3.1 - p[2] * 4.3);
}
}

void CrashDeformer::addPart(vtkUnstructuredGrid* grid)
{
    Part part;
    part.grid = grid;
    part.rest = vtkSmartPointer<vtkPoints>::New();
    part.rest->DeepCopy(grid->GetPoints());
    part.disp.assign(3 * static_cast<size_t>(grid->GetNumberOfPoints()), 0.0);
    part.target = part.disp;
    part.last = part.disp;

    part.damage = vtkSmartPointer<vtkDoubleArray>::New();
    part.damage->SetName(kArrayName);
    part.damage->SetNumberOfComponents(1);
    part.damage->SetNumberOfTuples(grid->GetNumberOfCells());
    part.damage->Fill(0.0);
    grid->GetCellData()->AddArray(part.damage);

    part.strain = vtkSmartPointer<vtkDoubleArray>::New();
    part.strain->SetName(kStrainArray);
    part.strain->SetNumberOfComponents(1);
    part.strain->SetNumberOfTuples(grid->GetNumberOfCells());
    part.strain->Fill(0.0);
    grid->GetCellData()->AddArray(part.strain);

    // edges of every cell with their rest length (strain = change of the edge lengths)
    vtkNew<vtkGenericCell> cell;
    const vtkIdType nc = grid->GetNumberOfCells();
    part.edgeOffset.reserve(nc + 1);
    part.edgeOffset.push_back(0);
    for (vtkIdType c = 0; c < nc; ++c)
    {
        grid->GetCell(c, cell);
        for (int e = 0; e < cell->GetNumberOfEdges(); ++e)
        {
            vtkCell* edge = cell->GetEdge(e);
            const vtkIdType a = edge->GetPointId(0), b = edge->GetPointId(1);
            double pa[3], pb[3];
            part.rest->GetPoint(a, pa);
            part.rest->GetPoint(b, pb);
            const double len = std::sqrt((pa[0] - pb[0]) * (pa[0] - pb[0]) + (pa[1] - pb[1]) * (pa[1] - pb[1]) +
                                         (pa[2] - pb[2]) * (pa[2] - pb[2]));
            if (len < 1e-6)
                continue;
            part.edgeA.push_back(a);
            part.edgeB.push_back(b);
            part.edgeLen.push_back(len);
        }
        part.edgeOffset.push_back(static_cast<vtkIdType>(part.edgeA.size()));
    }

    m_parts.push_back(std::move(part));
}

double CrashDeformer::addImpact(const double c[3], const double nIn[3], double energyJ, double impactRadius,
                                double massKg)
{
    double n[3] = { nIn[0], nIn[1], nIn[2] };
    const double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    for (double& k : n) k /= nl;

    // body bounds -> center axis (for "outward") and the hood height
    double lo[3] = { 1e9, 1e9, 1e9 }, hi[3] = { -1e9, -1e9, -1e9 };
    for (const auto& part : m_parts)
    {
        double b[6];
        part.rest->GetBounds(b);
        for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], b[2 * k]); hi[k] = std::max(hi[k], b[2 * k + 1]); }
    }
    const double center[3] = { 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2]) };

    // energy -> crush depth: E = 1/2 k x^2. k ~ 1.0 MN/m for 1.5 t (0.6 m at 56 km/h);
    // heavier vehicles have stiffer frames. Already crushed metal: x_total = sqrt(x0^2 + 2E/k).
    const double stiffness = 1.0e6 * std::pow(massKg / 1500.0, 0.8);
    double x0 = 0.0;
    {
        // existing crush along n around the impact point
        double sum = 0.0;
        int cnt = 0;
        for (const auto& part : m_parts)
            for (vtkIdType i = 0; i < part.rest->GetNumberOfPoints(); ++i)
            {
                double p[3];
                part.rest->GetPoint(i, p);
                const double d2 = (p[0] - c[0]) * (p[0] - c[0]) + (p[1] - c[1]) * (p[1] - c[1]) +
                                  (p[2] - c[2]) * (p[2] - c[2]);
                if (d2 > 0.35 * 0.35)
                    continue;
                const double* t = &part.target[3 * i];
                sum += std::max(0.0, t[0] * n[0] + t[1] * n[1] + t[2] * n[2]);
                ++cnt;
            }
        x0 = cnt ? sum / cnt : 0.0;
    }
    const double total = std::sqrt(x0 * x0 + 2.0 * std::max(0.0, energyJ) / stiffness);
    const double depth = std::clamp(total - x0, 0.0, std::max(0.0, 1.25 - x0));
    m_lastDepth = x0 + depth;
    if (depth < 1e-4)
        return 0.0;

    const bool narrow = impactRadius < 0.6;
    const double speedLike = depth / 0.04;              // ~ equivalent impact speed [m/s]
    const double radius = impactRadius + 0.01 * speedLike;
    // length of the structure that folds up (crash rails); the cabin behind it stays stiff
    const double zone = 0.25 + 1.5 * depth;
    const double pitch = 0.30 + 0.10 * depth;           // fold wavelength of the buckling rails
    const bool frontRear = std::abs(n[0]) > 0.5;

    for (auto& part : m_parts)
    {
        const vtkIdType np = part.rest->GetNumberOfPoints();
        for (vtkIdType i = 0; i < np; ++i)
        {
            double p[3];
            part.rest->GetPoint(i, p);
            double* d = &part.last[3 * i];
            d[0] = d[1] = d[2] = 0.0;

            const double v[3] = { p[0] - c[0], p[1] - c[1], p[2] - c[2] };
            const double along = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];   // depth into the body
            if (along < -0.08 || along > zone)
                continue;
            const double a = std::max(0.0, along);

            // lateral footprint: a wall flattens the whole face (flat-topped), a pole a narrow U
            const double perp[3] = { v[0] - along * n[0], v[1] - along * n[1], v[2] - along * n[2] };
            const double r = std::sqrt(perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]);
            const double q = r / radius;
            const double w = narrow ? std::exp(-q * q) : std::exp(-q * q * q * q);
            if (w < 1e-3)
                continue;

            // compression: the bumper moves back by the full depth, the firewall hardly at all
            const double f = 1.0 - a / zone;
            const double comp = depth * w * std::pow(f, 1.25);

            // outward direction (away from the body axis, perpendicular to the crush)
            const double rc[3] = { p[0] - center[0], p[1] - center[1], p[2] - center[2] };
            const double rcn = rc[0] * n[0] + rc[1] * n[1] + rc[2] * n[2];
            double out[3] = { rc[0] - rcn * n[0], rc[1] - rcn * n[1], rc[2] - rcn * n[2] };
            const double ol = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
            if (ol > 1e-6) for (double& k : out) k /= ol;
            else out[0] = out[1] = out[2] = 0.0;

            // accordion folds (alternating in / out along the crush direction) + Poisson bulge
            const double phase = 1.3 * (p[2] - lo[2]);   // folds slightly tilted, not perfectly planar
            const double fold = std::min(0.06, 0.12 * comp) * std::sin(2.0 * kPi * a / pitch + phase);
            const double bulge = 0.10 * comp;
            const double dent = 0.08 * comp * dentField(p);

            for (int k = 0; k < 3; ++k)
                d[k] = (comp + dent) * n[k] + (fold + bulge) * out[k];

            // hood / trunk lid tents upward in the middle of the crushed zone
            if (frontRear)
            {
                const double top = smooth((p[2] - center[2]) / std::max(0.05, hi[2] - center[2]));
                const double mid = 1.0 - std::min(1.0, std::abs(p[1] - center[1]) / (0.5 * (hi[1] - lo[1]) + 1e-6));
                d[2] += 0.35 * depth * w * std::sin(kPi * std::clamp(a / zone, 0.0, 1.0)) * top * (0.4 + 0.6 * mid);
            }
            // a pole wraps the body around itself: material beside the dent is pulled toward it
            if (narrow && r > 1e-6)
            {
                const double pull = 0.35 * comp * std::min(1.0, q);
                for (int k = 0; k < 3; ++k)
                    d[k] -= pull * perp[k] / r;
            }

            // accumulate (a node cannot be pushed further than ~1.4 m in total)
            double* t = &part.target[3 * i];
            for (int k = 0; k < 3; ++k)
                t[k] += d[k];
            const double tl = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
            if (tl > 1.4)
                for (int k = 0; k < 3; ++k)
                    t[k] *= 1.4 / tl;
        }
    }
    computeStats();
    m_animTime = 0.0;
    m_animating = true;
    return depth;
}

void CrashDeformer::computeStats()
{
    // largest cell strain of the final shape (rest + target)
    m_finalStrain = 0.0;
    for (const auto& part : m_parts)
    {
        const vtkIdType nc = static_cast<vtkIdType>(part.edgeOffset.size()) - 1;
        for (vtkIdType c = 0; c < nc; ++c)
        {
            double s2 = 0.0;
            const vtkIdType e0 = part.edgeOffset[c], e1 = part.edgeOffset[c + 1];
            for (vtkIdType e = e0; e < e1; ++e)
            {
                double a[3], b[3];
                part.rest->GetPoint(part.edgeA[e], a);
                part.rest->GetPoint(part.edgeB[e], b);
                const double* ta = &part.target[3 * part.edgeA[e]];
                const double* tb = &part.target[3 * part.edgeB[e]];
                double d2 = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    const double d = (a[k] + ta[k]) - (b[k] + tb[k]);
                    d2 += d * d;
                }
                const double eps = std::sqrt(d2) / part.edgeLen[e] - 1.0;
                s2 += eps * eps;
            }
            if (e1 > e0)
                m_finalStrain = std::max(m_finalStrain, 100.0 * std::sqrt(s2 / double(e1 - e0)));
        }
    }
    m_maxDisp = 0.0;
    for (double& f : m_face) f = 0.0;
    for (const auto& part : m_parts)
    {
        const vtkIdType np = part.rest->GetNumberOfPoints();
        for (vtkIdType i = 0; i < np; ++i)
        {
            const double* t = &part.target[3 * i];
            m_maxDisp = std::max(m_maxDisp, std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]));
            double p[3];
            part.rest->GetPoint(i, p);
            // inward displacement of the nodes on each outer face
            if (p[0] < 0.15) m_face[0] = std::max(m_face[0], t[0]);
            if (p[0] > 3.85) m_face[1] = std::max(m_face[1], -t[0]);
            if (p[1] < 0.08) m_face[2] = std::max(m_face[2], t[1]);
            if (p[1] > 0.92) m_face[3] = std::max(m_face[3], -t[1]);
        }
    }
}

bool CrashDeformer::update(double dt)
{
    if (!m_animating)
        return false;
    m_animTime += dt;
    // the body crushes within ~0.1 s, overshoots by 7 % of the last dent and springs back
    const double over = 0.07 * std::exp(-m_animTime / 0.15);
    const double k = std::min(1.0, dt * 22.0);
    double err = 0.0;
    for (auto& part : m_parts)
    {
        const size_t n = part.disp.size();
        for (size_t i = 0; i < n; ++i)
        {
            const double goal = part.target[i] + over * part.last[i];
            const double d = goal - part.disp[i];
            part.disp[i] += k * d;
            err = std::max(err, std::abs(d));
        }
    }
    if (err < 2e-4 && m_animTime > 0.6)
    {
        for (auto& part : m_parts)
            part.disp = part.target;
        m_animating = false;
    }
    apply();
    return true;
}

void CrashDeformer::settle()
{
    for (auto& part : m_parts)
        part.disp = part.target;
    m_animating = false;
    apply();
}

void CrashDeformer::apply()
{
    m_maxStrain = 0.0;
    for (auto& part : m_parts)
    {
        vtkPoints* pts = part.grid->GetPoints();
        const vtkIdType np = part.rest->GetNumberOfPoints();
        for (vtkIdType i = 0; i < np; ++i)
        {
            double r[3];
            part.rest->GetPoint(i, r);
            const double* d = &part.disp[3 * i];
            pts->SetPoint(i, r[0] + d[0], r[1] + d[1], r[2] + d[2]);
        }
        pts->Modified();
        updateArrays(part);
        part.grid->Modified();
        for (vtkIdType cId = 0; cId < part.strain->GetNumberOfTuples(); ++cId)
            m_maxStrain = std::max(m_maxStrain, part.strain->GetValue(cId));
    }
}

void CrashDeformer::updateArrays(Part& part)
{
    vtkPoints* pts = part.grid->GetPoints();
    const vtkIdType nc = part.grid->GetNumberOfCells();
    vtkNew<vtkIdList> ids;
    for (vtkIdType c = 0; c < nc; ++c)
    {
        // mean nodal displacement
        part.grid->GetCellPoints(c, ids);
        double sum = 0.0;
        for (vtkIdType k = 0; k < ids->GetNumberOfIds(); ++k)
        {
            const double* d = &part.disp[3 * ids->GetId(k)];
            sum += std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        }
        part.damage->SetValue(c, sum / std::max<vtkIdType>(1, ids->GetNumberOfIds()));

        // effective strain: RMS of the edge strains (compression and stretching both count)
        double s2 = 0.0;
        const vtkIdType e0 = part.edgeOffset[c], e1 = part.edgeOffset[c + 1];
        for (vtkIdType e = e0; e < e1; ++e)
        {
            double a[3], b[3];
            pts->GetPoint(part.edgeA[e], a);
            pts->GetPoint(part.edgeB[e], b);
            const double len = std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                                         (a[2] - b[2]) * (a[2] - b[2]));
            const double eps = len / part.edgeLen[e] - 1.0;
            s2 += eps * eps;
        }
        const double strain = e1 > e0 ? std::sqrt(s2 / double(e1 - e0)) : 0.0;
        part.strain->SetValue(c, 100.0 * strain);
    }
    part.damage->Modified();
    part.strain->Modified();
}

double CrashDeformer::damagedFraction(double threshold) const
{
    vtkNew<vtkIdList> ids;
    vtkIdType total = 0, damaged = 0;
    for (const auto& part : m_parts)
    {
        const vtkIdType nc = part.grid->GetNumberOfCells();
        for (vtkIdType c = 0; c < nc; ++c)
        {
            part.grid->GetCellPoints(c, ids);
            double sum = 0.0;
            for (vtkIdType k = 0; k < ids->GetNumberOfIds(); ++k)
            {
                const double* d = &part.target[3 * ids->GetId(k)];
                sum += std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            }
            if (sum / std::max<vtkIdType>(1, ids->GetNumberOfIds()) > threshold)
                ++damaged;
            ++total;
        }
    }
    return total ? static_cast<double>(damaged) / total : 0.0;
}

double CrashDeformer::maxDisplacementIn(size_t partIndex, bool (*region)(const double p[3])) const
{
    if (partIndex >= m_parts.size())
        return 0.0;
    const Part& part = m_parts[partIndex];
    double best = 0.0;
    for (vtkIdType i = 0; i < part.rest->GetNumberOfPoints(); ++i)
    {
        double p[3];
        part.rest->GetPoint(i, p);
        if (!region(p))
            continue;
        const double* d = &part.target[3 * i];
        best = std::max(best, std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }
    return best;
}

void CrashDeformer::reset()
{
    for (auto& part : m_parts)
    {
        std::fill(part.disp.begin(), part.disp.end(), 0.0);
        std::fill(part.target.begin(), part.target.end(), 0.0);
        std::fill(part.last.begin(), part.last.end(), 0.0);
    }
    m_lastDepth = 0.0;
    m_maxDisp = 0.0;
    m_maxStrain = 0.0;
    m_finalStrain = 0.0;
    for (double& f : m_face) f = 0.0;
    m_animating = false;
    apply();
}
