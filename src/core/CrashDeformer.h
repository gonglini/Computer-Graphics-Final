#pragma once

#include <vector>

#include <vtkSmartPointer.h>

class vtkUnstructuredGrid;
class vtkPoints;
class vtkDoubleArray;

// Crash deformation of the vehicle's hexahedral meshes, accumulated over several impacts.
//
// A "crush zone" model (not FEM) shaped like real crash-test results:
//  - every impact adds a dent whose depth comes from the energy the body absorbs:
//    E = 1/2 k x^2 (k ~ 1 MN/m for a 1.5 t car, about 0.6 m at 56 km/h like a frontal NCAP test).
//    A dent on an already crushed spot needs more energy: the total crush there is
//    x = sqrt(x0^2 + 2E/k) (crushed metal is stiffer), so two hits at 30 km/h equal one at 42 km/h
//  - the material in front of the safety cage is compressed (more at the bumper, less toward the
//    firewall) and folds like an accordion (progressive buckling of the crash rails)
//  - compressed sheet metal bulges outward, the hood tents upward, small random dents
//  - a narrow object (pole, tree) makes a deep U-shaped dent that wraps around it
//  - side impacts dent the door / cargo box at the point of contact (front or rear half)
// The mesh moves toward the accumulated shape in real time (with a short dynamic overshoot and
// elastic spring-back), so the body crumples while the car is still moving.
// Per cell two arrays are stored:
//  "Deformation"    mean nodal displacement [m]
//  "PlasticStrain"  effective strain from the change of the cell edge lengths [%], shown with a
//                   banded color map like a crash-analysis post-processor (LS-PrePost / ParaView)
class CrashDeformer
{
public:
    static constexpr const char* kArrayName = "Deformation";
    static constexpr const char* kStrainArray = "PlasticStrain";

    void addPart(vtkUnstructuredGrid* grid);

    // One impact, in vehicle local coordinates; normal points INTO the vehicle.
    // energyJ: energy absorbed by this vehicle's body, impactRadius: lateral size of the dent
    // (narrow pole ~0.3 m, wall ~1.4 m). Returns the depth added at the impact point [m].
    double addImpact(const double impactLocal[3], const double normalLocal[3], double energyJ,
                     double impactRadius, double massKg);

    // animate the mesh toward the accumulated deformation; returns true if the mesh changed
    bool update(double dt);
    // jump to the final shape (headless runs)
    void settle();
    bool animating() const { return m_animating; }

    void reset();

    double crushDepth() const { return m_lastDepth; }        // total crush at the last impact point
    double maxDisplacement() const { return m_maxDisp; }      // of the final shape
    double maxStrainPct() const { return m_maxStrain; }       // of the shape shown now
    double finalStrainPct() const { return m_finalStrain; }   // of the final shape
    // how far each face is pushed in (final shape): 0 front (x = 0), 1 rear (x = 4), 2 side y = 0 (left),
    // 3 side y = 1 (right) -> the collision footprint shrinks by this much
    double faceCrush(int face) const { return m_face[face]; }

    // statistics of the final deformation
    // fraction of all cells whose mean displacement exceeds 'threshold'
    double damagedFraction(double threshold) const;
    // largest nodal displacement of part 'part' among nodes whose rest position satisfies 'region'
    double maxDisplacementIn(size_t part, bool (*region)(const double p[3])) const;

private:
    struct Part
    {
        vtkSmartPointer<vtkUnstructuredGrid> grid;
        vtkSmartPointer<vtkPoints> rest;
        std::vector<double> disp;     // 3 * nPoints: shown now
        std::vector<double> target;   // 3 * nPoints: accumulated final deformation
        std::vector<double> last;     // 3 * nPoints: the last dent (for the dynamic overshoot)
        vtkSmartPointer<vtkDoubleArray> damage;
        vtkSmartPointer<vtkDoubleArray> strain;
        // cell edges: [offset[c], offset[c+1]) into edgeA / edgeB / edgeLen
        std::vector<vtkIdType> edgeOffset, edgeA, edgeB;
        std::vector<double> edgeLen;
    };
    void apply();                 // disp -> points + cell arrays
    void updateArrays(Part& part);
    void computeStats();          // final-shape statistics

    std::vector<Part> m_parts;
    double m_lastDepth = 0.0;
    double m_maxDisp = 0.0;
    double m_maxStrain = 0.0;
    double m_finalStrain = 0.0;
    double m_face[4] = { 0, 0, 0, 0 };
    double m_animTime = 0.0;
    bool m_animating = false;
};
