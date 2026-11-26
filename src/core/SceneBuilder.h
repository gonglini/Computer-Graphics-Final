#pragma once

#include <array>
#include <functional>
#include <vector>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

class vtkPolyData;
struct RoadModel;

// Mesh factory. All parts are vtkUnstructuredGrid with a cell array "ElementScalar"
// (a Palette index per cell), like the PID meshes of the original drive-vehicle example.
namespace SceneBuilder
{
using Grid = vtkSmartPointer<vtkUnstructuredGrid>;

// (u, v, w) in [0,1]^3 -> position
using ShapeFn = std::function<void(double u, double v, double w, double out[3])>;
// cell (i, j, k) and its parametric center -> palette index
using MaterialFn = std::function<int(int i, int j, int k, double uc, double vc, double wc)>;

// Structured hexahedral block whose nodes are mapped through 'shape'.
// This is how the car bodies are built: a deformable solid mesh with a car silhouette.
Grid morphedHex(int nx, int ny, int nz, const ShapeFn& shape, const MaterialFn& material);

// Axis-aligned box split into nx*ny*nz hexahedra.
Grid hexBlock(const std::array<double, 3>& minCorner, const std::array<double, 3>& maxCorner,
              int nx, int ny, int nz, int material);

// Box with a material chosen per cell (facades, stripes ...).
Grid hexBlock(const std::array<double, 3>& minCorner, const std::array<double, 3>& maxCorner,
              int nx, int ny, int nz, const MaterialFn& material);

// Group of cylinders. axis = 'X', 'Y' or 'Z'.
Grid cylinderGroup(const std::vector<std::array<double, 3>>& centers, double radius, double height,
                   int material, char axis = 'Y', int resolution = 20);

// Group of cones pointing +Z.
Grid coneGroup(const std::vector<std::array<double, 3>>& centers, double radius, double height,
               int material, int resolution = 12);

// Road surface following RoadModel's centerline (the whole loop).
Grid roadStrip(const RoadModel& road, double segmentLength, int lateralDivisions, int material);

// Lane markings of the divided road: yellow double center line, dashed white lane dividers,
// solid white edge lines (thin quads just above the road).
// Gaps are left where the station s is within gapHalf of a gap (intersections).
Grid laneMarkings(const RoadModel& road, int white, int yellow,
                  const std::vector<double>& gaps = {}, double gapHalf = 0.0);

// Band that follows the road from station s0 to s1 (s0 < s1) between two lateral offsets.
Grid ribbon(const RoadModel& road, double s0, double s1, double lat0, double lat1, double z,
            double segmentLength, int material, int lateralDivisions = 1);

// Solid wall that follows the road (guard rails, median barrier): centered at 'lat', 'width' thick.
Grid wallAlong(const RoadModel& road, double s0, double s1, double lat, double width, double z0, double z1,
               double segmentLength, int material);

// Box centered at (cx, cy) rotated by yaw about z.
Grid orientedBlock(double cx, double cy, double yawDeg, double lenX, double lenY, double z0, double z1,
                   int nx, int ny, int nz, const MaterialFn& material);

// Gable roof (ridge along the box's local x axis) sitting on a box of lenX x lenY at height z0.
Grid gableRoof(double cx, double cy, double yawDeg, double lenX, double lenY, double z0, double height,
               int material);

// Wobbly ellipsoids (tree crowns): each entry = cx, cy, cz, rx, ry, rz.
Grid blobGroup(const std::vector<std::array<double, 6>>& blobs, int material, int resolution = 10);

// Flat axis-aligned rectangle at height z.
Grid rect(double x0, double x1, double y0, double y1, double z, int material);

// Copy of a grid moved by T(x, y) * Rz(angleDeg).
Grid placed(vtkUnstructuredGrid* grid, double x, double y, double angleDeg);

Grid fromPolyData(vtkPolyData* pd, int material);
Grid merge(const std::vector<Grid>& grids);
void setCellScalar(vtkUnstructuredGrid* grid, double value);
}
