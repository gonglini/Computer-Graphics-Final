#include <functional>
#include "SceneBuilder.h"
#include "RoadModel.h"

#include <algorithm>
#include <cmath>

#include <vtkAppendFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCellType.h>
#include <vtkConeSource.h>
#include <vtkCylinderSource.h>
#include <vtkDoubleArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace SceneBuilder
{

void setCellScalar(vtkUnstructuredGrid* grid, double value)
{
    const vtkIdType n = grid->GetNumberOfCells();
    vtkNew<vtkDoubleArray> s;
    s->SetName("ElementScalar");
    s->SetNumberOfComponents(1);
    s->SetNumberOfTuples(n);
    for (vtkIdType i = 0; i < n; ++i)
        s->SetValue(i, value);
    grid->GetCellData()->SetScalars(s);
}

Grid morphedHex(int nx, int ny, int nz, const ShapeFn& shape, const MaterialFn& material)
{
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();

    auto pid = [&](int i, int j, int k) {
        return static_cast<vtkIdType>((k * (ny + 1) + j) * (nx + 1) + i);
    };

    for (int k = 0; k <= nz; ++k)
        for (int j = 0; j <= ny; ++j)
            for (int i = 0; i <= nx; ++i)
            {
                double p[3];
                shape(static_cast<double>(i) / nx, static_cast<double>(j) / ny,
                      static_cast<double>(k) / nz, p);
                points->InsertNextPoint(p);
            }

    Grid grid = Grid::New();
    grid->SetPoints(points);
    grid->Allocate(static_cast<vtkIdType>(nx) * ny * nz);

    vtkNew<vtkDoubleArray> s;
    s->SetName("ElementScalar");
    s->SetNumberOfComponents(1);

    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i)
            {
                const vtkIdType ids[8] = {
                    pid(i, j, k),         pid(i + 1, j, k),
                    pid(i + 1, j + 1, k), pid(i, j + 1, k),
                    pid(i, j, k + 1),     pid(i + 1, j, k + 1),
                    pid(i + 1, j + 1, k + 1), pid(i, j + 1, k + 1)
                };
                grid->InsertNextCell(VTK_HEXAHEDRON, 8, ids);
                s->InsertNextValue(material(i, j, k, (i + 0.5) / nx, (j + 0.5) / ny, (k + 0.5) / nz));
            }

    grid->GetCellData()->SetScalars(s);
    return grid;
}

Grid hexBlock(const std::array<double, 3>& mn, const std::array<double, 3>& mx,
              int nx, int ny, int nz, const MaterialFn& material)
{
    return morphedHex(nx, ny, nz,
        [&](double u, double v, double w, double out[3]) {
            out[0] = mn[0] + (mx[0] - mn[0]) * u;
            out[1] = mn[1] + (mx[1] - mn[1]) * v;
            out[2] = mn[2] + (mx[2] - mn[2]) * w;
        },
        material);
}

Grid hexBlock(const std::array<double, 3>& mn, const std::array<double, 3>& mx,
              int nx, int ny, int nz, int material)
{
    return hexBlock(mn, mx, nx, ny, nz,
                    [material](int, int, int, double, double, double) { return material; });
}

Grid fromPolyData(vtkPolyData* pd, int material)
{
    vtkNew<vtkAppendFilter> appendFilter;
    appendFilter->AddInputData(pd);
    appendFilter->Update();

    Grid grid = Grid::New();
    grid->ShallowCopy(appendFilter->GetOutput());
    setCellScalar(grid, material);
    return grid;
}

Grid cylinderGroup(const std::vector<std::array<double, 3>>& centers, double radius, double height,
                   int material, char axis, int resolution)
{
    vtkNew<vtkAppendPolyData> append;
    for (const auto& c : centers)
    {
        vtkNew<vtkCylinderSource> cylinder;
        cylinder->SetRadius(radius);
        cylinder->SetHeight(height);
        cylinder->SetResolution(resolution);
        cylinder->CappingOn();

        // vtkCylinderSource: centered at origin, axis = Y
        vtkNew<vtkTransform> t;
        t->Translate(c[0], c[1], c[2]);
        if (axis == 'Z' || axis == 'z')
            t->RotateX(90.0);
        else if (axis == 'X' || axis == 'x')
            t->RotateZ(-90.0);

        vtkNew<vtkTransformPolyDataFilter> tf;
        tf->SetInputConnection(cylinder->GetOutputPort());
        tf->SetTransform(t);
        tf->Update();
        append->AddInputData(tf->GetOutput());
    }
    append->Update();
    return fromPolyData(append->GetOutput(), material);
}

Grid coneGroup(const std::vector<std::array<double, 3>>& centers, double radius, double height,
               int material, int resolution)
{
    vtkNew<vtkAppendPolyData> append;
    for (const auto& c : centers)
    {
        vtkNew<vtkConeSource> cone;
        cone->SetRadius(radius);
        cone->SetHeight(height);
        cone->SetResolution(resolution);
        cone->SetDirection(0.0, 0.0, 1.0);
        cone->SetCenter(c[0], c[1], c[2]);
        cone->CappingOn();
        cone->Update();

        auto pd = vtkSmartPointer<vtkPolyData>::New();
        pd->DeepCopy(cone->GetOutput());
        append->AddInputData(pd);
    }
    append->Update();
    return fromPolyData(append->GetOutput(), material);
}

Grid roadStrip(const RoadModel& road, double segmentLength, int lateralDivisions, int material)
{
    return ribbon(road, 0.0, road.length(), -road.halfWidth - 0.4, road.halfWidth + 0.4, road.groundZ,
                  segmentLength, material, lateralDivisions);
}

Grid ribbon(const RoadModel& road, double s0, double s1, double lat0, double lat1, double z,
            double segmentLength, int material, int lateralDivisions)
{
    const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / segmentLength)));
    const int m = std::max(1, lateralDivisions);
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    for (int i = 0; i <= n; ++i)
    {
        const double s = s0 + (s1 - s0) * i / n;
        for (int j = 0; j <= m; ++j)
        {
            double x, y;
            road.point(s, lat0 + (lat1 - lat0) * j / m, x, y);
            points->InsertNextPoint(x, y, z + road.elevation(s));
        }
    }
    Grid grid = Grid::New();
    grid->SetPoints(points);
    grid->Allocate(static_cast<vtkIdType>(n) * m);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
        {
            const vtkIdType a = i * (m + 1) + j;
            const vtkIdType b = (i + 1) * (m + 1) + j;
            const vtkIdType ids[4] = { a, a + 1, b + 1, b };
            grid->InsertNextCell(VTK_QUAD, 4, ids);
        }
    setCellScalar(grid, material);
    return grid;
}

Grid wallAlong(const RoadModel& road, double s0, double s1, double lat, double width, double z0, double z1,
               double segmentLength, int material)
{
    const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / segmentLength)));
    return morphedHex(n, 1, 1,
        [&](double u, double v, double w, double out[3]) {
            const double s = s0 + (s1 - s0) * u;
            road.point(s, lat + (v - 0.5) * width, out[0], out[1]);
            out[2] = z0 + (z1 - z0) * w + road.elevation(s);
        },
        [material](int, int, int, double, double, double) { return material; });
}

Grid orientedBlock(double cx, double cy, double yawDeg, double lenX, double lenY, double z0, double z1,
                   int nx, int ny, int nz, const MaterialFn& material)
{
    const double c = std::cos(yawDeg * 3.14159265358979323846 / 180.0);
    const double sn = std::sin(yawDeg * 3.14159265358979323846 / 180.0);
    return morphedHex(nx, ny, nz,
        [&](double u, double v, double w, double out[3]) {
            const double lx = (u - 0.5) * lenX, ly = (v - 0.5) * lenY;
            out[0] = cx + c * lx - sn * ly;
            out[1] = cy + sn * lx + c * ly;
            out[2] = z0 + (z1 - z0) * w;
        },
        material);
}

Grid gableRoof(double cx, double cy, double yawDeg, double lenX, double lenY, double z0, double height,
               int material)
{
    // prism: ridge along the local x axis, 0.4 m overhang
    const double c = std::cos(yawDeg * 3.14159265358979323846 / 180.0);
    const double sn = std::sin(yawDeg * 3.14159265358979323846 / 180.0);
    const double L = lenX + 0.6, W = lenY + 0.8;
    return morphedHex(2, 2, 1,
        [&](double u, double v, double w, double out[3]) {
            const double lx = (u - 0.5) * L;
            const double off = (v - 0.5) * W;             // across the ridge
            const double ridge = 1.0 - std::abs(2.0 * v - 1.0);
            const double ly = off;
            out[0] = cx + c * lx - sn * ly;
            out[1] = cy + sn * lx + c * ly;
            // bottom face slightly sloped too, so no cell is flat (thin roof slab)
            out[2] = z0 + height * ridge + 0.18 * w - 0.12 * (1.0 - ridge);
        },
        [material](int, int, int, double, double, double) { return material; });
}

Grid blobGroup(const std::vector<std::array<double, 6>>& blobs, int material, int resolution)
{
    // ellipsoids (cx, cy, cz, rx, ry, rz) as closed triangle meshes, with a small deterministic wobble
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    vtkNew<vtkCellArray> polys;
    const int nu = std::max(6, resolution), nv = std::max(4, resolution / 2);
    unsigned seed = 7u;
    for (const auto& b : blobs)
    {
        const vtkIdType base = points->GetNumberOfPoints();
        seed = seed * 1664525u + 1013904223u;
        const double ph = (seed >> 8) / 16777216.0 * 6.283;
        for (int j = 0; j <= nv; ++j)
        {
            const double th = 3.14159265358979323846 * j / nv;   // 0 top .. pi bottom
            for (int i = 0; i < nu; ++i)
            {
                const double phi = 6.28318530717958647692 * i / nu;
                const double wob = 1.0 + 0.10 * std::sin(3.0 * phi + ph) * std::sin(2.0 * th + ph);
                const double flat = j > nv * 0.7 ? 0.75 : 1.0;   // flatter underside
                points->InsertNextPoint(b[0] + b[3] * wob * std::sin(th) * std::cos(phi),
                                        b[1] + b[4] * wob * std::sin(th) * std::sin(phi),
                                        b[2] + b[5] * flat * std::cos(th));
            }
        }
        for (int j = 0; j < nv; ++j)
            for (int i = 0; i < nu; ++i)
            {
                const vtkIdType a = base + j * nu + i, bb = base + j * nu + (i + 1) % nu;
                const vtkIdType c = base + (j + 1) * nu + (i + 1) % nu, d = base + (j + 1) * nu + i;
                const vtkIdType q[4] = { a, d, c, bb };
                polys->InsertNextCell(4, q);
            }
    }
    vtkNew<vtkPolyData> pd;
    pd->SetPoints(points);
    pd->SetPolys(polys);
    return fromPolyData(pd, material);
}

Grid rect(double x0, double x1, double y0, double y1, double z, int material)
{
    vtkNew<vtkPoints> points;
    points->InsertNextPoint(x0, y0, z);
    points->InsertNextPoint(x1, y0, z);
    points->InsertNextPoint(x1, y1, z);
    points->InsertNextPoint(x0, y1, z);
    Grid grid = Grid::New();
    grid->SetPoints(points);
    const vtkIdType ids[4] = { 0, 1, 2, 3 };
    grid->InsertNextCell(VTK_QUAD, 4, ids);
    setCellScalar(grid, material);
    return grid;
}

Grid placed(vtkUnstructuredGrid* grid, double x, double y, double angleDeg)
{
    vtkNew<vtkTransform> t;
    t->PostMultiply();
    t->RotateZ(angleDeg);
    t->Translate(x, y, 0.0);

    Grid out = Grid::New();
    out->DeepCopy(grid);
    vtkNew<vtkPoints> moved;
    t->TransformPoints(grid->GetPoints(), moved);
    out->SetPoints(moved);
    return out;
}

Grid laneMarkings(const RoadModel& road, int white, int yellow, const std::vector<double>& gaps, double gapHalf)
{
    auto inGap = [&](double s) {
        for (double g : gaps)
            if (std::abs(road.diff(s, g)) < gapHalf)
                return true;
        return false;
    };

    const double z = road.groundZ + 0.006;
    const double w = 0.07;  // half line width

    auto makeGrid = [&](int material, const std::function<void(const std::function<void(double, double, double)>&)>& fill) {
        vtkNew<vtkPoints> points;
        points->SetDataTypeToDouble();
        Grid grid = Grid::New();
        grid->SetPoints(points);
        // one quad between s0 and s1 at lateral offset lat
        fill([&](double s0, double s1, double lat) {
            double x0a, y0a, x0b, y0b, x1a, y1a, x1b, y1b;
            road.point(s0, lat - w, x0a, y0a);
            road.point(s0, lat + w, x0b, y0b);
            road.point(s1, lat + w, x1b, y1b);
            road.point(s1, lat - w, x1a, y1a);
            const double z0 = z + road.elevation(s0), z1 = z + road.elevation(s1);
            const vtkIdType a = points->InsertNextPoint(x0a, y0a, z0);
            const vtkIdType b = points->InsertNextPoint(x0b, y0b, z0);
            const vtkIdType c = points->InsertNextPoint(x1b, y1b, z1);
            const vtkIdType d = points->InsertNextPoint(x1a, y1a, z1);
            const vtkIdType ids[4] = { a, b, c, d };
            grid->InsertNextCell(VTK_QUAD, 4, ids);
        });
        setCellScalar(grid, material);
        return grid;
    };

    const double L = road.length();
    // white: dashed lane dividers (3 m dash, 4 m gap) + solid edge lines
    Grid whiteGrid = makeGrid(white, [&](const auto& strip) {
        for (double s = 0.0; s + 3.0 < L; s += 7.0)
            if (!inGap(s) && !inGap(s + 3.0))
                for (int k = 1; k < road.lanes; ++k)
                    for (double side : { -1.0, 1.0 })
                        strip(s, s + 3.0, side * k * road.laneWidth);
        for (double s = 0.0; s < L - 1e-6; s += 2.0)
            if (!inGap(s) && !inGap(s + 2.0))
                for (double side : { -1.0, 1.0 })
                    strip(s, std::min(s + 2.0, L), side * (road.halfWidth + 0.15));
    });

    // yellow: solid double center line (median)
    Grid yellowGrid = makeGrid(yellow, [&](const auto& strip) {
        for (double s = 0.0; s < L - 1e-6; s += 2.0)
            if (!inGap(s) && !inGap(s + 2.0))
                for (double side : { -1.0, 1.0 })
                    strip(s, std::min(s + 2.0, L), side * 0.13);
    });

    return merge({ whiteGrid, yellowGrid });
}

Grid merge(const std::vector<Grid>& grids)
{
    vtkNew<vtkAppendFilter> append;
    for (const auto& g : grids)
        append->AddInputData(g);
    append->Update();

    Grid out = Grid::New();
    out->ShallowCopy(append->GetOutput());
    return out;
}

} // namespace SceneBuilder
