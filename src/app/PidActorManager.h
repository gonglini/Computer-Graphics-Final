#pragma once

#include <map>
#include <vector>

#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkUnstructuredGrid;
class vtkActor;
class vtkDataSetMapper;
class vtkScalarsToColors;
class vtkLinearTransform;

// From the original drive-vehicle example: one actor (+ highlight actor) per PID.
// Extended with: shared user transforms (moving vehicle), damage color view, PBR material.
class PidActorManager
{
public:
    struct PidEntry
    {
        int pid = -1;

        vtkSmartPointer<vtkUnstructuredGrid> grid;

        vtkSmartPointer<vtkDataSetMapper> mapper;
        vtkSmartPointer<vtkActor> actor;

        vtkSmartPointer<vtkDataSetMapper> highlightMapper;
        vtkSmartPointer<vtkActor> highlightActor;

        vtkSmartPointer<vtkScalarsToColors> defaultLut;

        bool visible = true;
    };

public:
    // color table used for "ElementScalar" (call before buildFromPidGrids)
    void setLookupTable(vtkScalarsToColors* lut, double minValue, double maxValue);

    void buildFromPidGrids(
        vtkRenderer* renderer,
        const std::map<int, vtkSmartPointer<vtkUnstructuredGrid>>& pidGrids);

    void setVisibility(int pid, bool visible);
    bool isVisible(int pid) const;

    void setSurfaceMode();
    void setWireframeMode();

    void setSelectedPid(int pid);
    void clearSelection();

    int getSelectedPid() const;

    std::vector<int> getPidList() const;

    int findPidByActor(vtkActor* actor) const;

    vtkActor* getActorByPid(int pid) const;
    vtkActor* getHighlightActorByPid(int pid) const;

    // --- added for lidar-drive ---
    // actor + highlight actor follow this transform (vehicle pose)
    void setUserTransform(int pid, vtkLinearTransform* transform);

    // color by the cell array "Deformation" instead of the PID color
    void setDamageView(int pid, bool on, vtkScalarsToColors* lut, double maxValue);

    void setPbr(int pid, bool on, double metallic, double roughness);

    // show the finite-element mesh lines (cell edges) on surfaces
    void setEdgesVisible(bool on);

private:
    std::map<int, PidEntry> m_pidEntries;
    int m_selectedPid = -1;
    vtkSmartPointer<vtkScalarsToColors> m_lut;
    double m_lutRange[2] = { 10.0, 25.0 };
    bool m_edges = false;
    bool m_wireframe = false;
};
