#include "PidActorManager.h"

#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkDataSetMapper.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkFeatureEdges.h>
#include <vtkNew.h>
#include <vtkLinearTransform.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkScalarsToColors.h>
#include <vtkUnstructuredGrid.h>

#include "core/CrashDeformer.h"
#include "core/World.h"

void PidActorManager::setLookupTable(vtkScalarsToColors* lut, double minValue, double maxValue)
{
    m_lut = lut;
    m_lutRange[0] = minValue;
    m_lutRange[1] = maxValue;
}

void PidActorManager::setEdgesVisible(bool on)
{
    m_edges = on;
    for (auto& [pid, entry] : m_pidEntries)
        if (entry.actor && !m_wireframe)
            entry.actor->GetProperty()->SetEdgeVisibility(on);
}

void PidActorManager::buildFromPidGrids(
    vtkRenderer* renderer,
    const std::map<int, vtkSmartPointer<vtkUnstructuredGrid>>& pidGrids)
{
    m_pidEntries.clear();
    m_selectedPid = -1;

    for (const auto& [pid, grid] : pidGrids)
    {
        PidEntry entry;
        entry.pid = pid;
        entry.grid = grid;
        entry.visible = true;

        // --- default actor ---
        entry.mapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.actor = vtkSmartPointer<vtkActor>::New();

        entry.mapper->SetInputData(grid);

        if (grid->GetCellData() && grid->GetCellData()->GetScalars())
        {
            entry.mapper->SetScalarModeToUseCellData();
            entry.mapper->ScalarVisibilityOn();
            if (m_lut)
                entry.mapper->SetLookupTable(m_lut);
            entry.mapper->SetScalarRange(m_lutRange[0], m_lutRange[1]);
        }
        else
        {
            entry.mapper->ScalarVisibilityOff();
        }
        entry.defaultLut = entry.mapper->GetLookupTable();

        entry.actor->SetMapper(entry.mapper);
        entry.actor->GetProperty()->SetRepresentationToSurface();
        entry.actor->GetProperty()->SetEdgeVisibility(m_edges);
        entry.actor->GetProperty()->SetEdgeColor(0.08, 0.08, 0.08);

        renderer->AddActor(entry.actor);

        // --- highlight actor (selection overlay, ParaView style) ---
        entry.highlightMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.highlightActor  = vtkSmartPointer<vtkActor>::New();

        // Vehicle parts: full wireframe like the original example. Scene parts (road, buildings,
        // trees ...) are huge meshes spread over the whole map: their full wireframe turned into a
        // yellow grid over everything, so only the outline (boundary + sharp feature edges) is drawn.
        const bool outlineOnly = !World::isVehiclePid(pid);
        if (outlineOnly)
        {
            vtkNew<vtkDataSetSurfaceFilter> surf;
            surf->SetInputData(grid);
            vtkNew<vtkFeatureEdges> edges;
            edges->SetInputConnection(surf->GetOutputPort());
            edges->BoundaryEdgesOn();
            edges->FeatureEdgesOn();
            edges->SetFeatureAngle(40.0);
            edges->ManifoldEdgesOff();
            edges->NonManifoldEdgesOff();
            edges->ColoringOff();
            entry.highlightMapper->SetInputConnection(edges->GetOutputPort());
        }
        else
        {
            entry.highlightMapper->SetInputData(grid);
        }
        entry.highlightMapper->ScalarVisibilityOff();

        entry.highlightActor->SetMapper(entry.highlightMapper);
        entry.highlightActor->SetVisibility(0);
        entry.highlightActor->PickableOff();

        entry.highlightActor->GetProperty()->SetRepresentationToWireframe();
        entry.highlightActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
        entry.highlightActor->GetProperty()->SetLineWidth(outlineOnly ? 2.0 : 3.0);
        entry.highlightActor->GetProperty()->SetAmbient(1.0);
        entry.highlightActor->GetProperty()->SetDiffuse(0.0);
        entry.highlightActor->GetProperty()->SetEdgeVisibility(0);

        renderer->AddActor(entry.highlightActor);

        m_pidEntries[pid] = entry;
    }
}

void PidActorManager::setVisibility(int pid, bool visible)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;

    it->second.visible = visible;
    if (it->second.actor)
        it->second.actor->SetVisibility(visible ? 1 : 0);

    if (it->second.highlightActor)
    {
        const bool highlightOn = visible && (pid == m_selectedPid);
        it->second.highlightActor->SetVisibility(highlightOn ? 1 : 0);
    }
}

bool PidActorManager::isVisible(int pid) const
{
    auto it = m_pidEntries.find(pid);
    return it != m_pidEntries.end() && it->second.visible;
}

void PidActorManager::setSurfaceMode()
{
    m_wireframe = false;
    for (auto& [pid, entry] : m_pidEntries)
    {
        if (!entry.actor)
            continue;
        entry.actor->GetProperty()->SetRepresentationToSurface();
        entry.actor->GetProperty()->SetEdgeVisibility(m_edges);
    }
}

void PidActorManager::setWireframeMode()
{
    m_wireframe = true;
    for (auto& [pid, entry] : m_pidEntries)
    {
        if (!entry.actor)
            continue;
        entry.actor->GetProperty()->SetRepresentationToWireframe();
        entry.actor->GetProperty()->SetEdgeVisibility(0);
    }
}

void PidActorManager::setSelectedPid(int pid)
{
    if (m_selectedPid != -1)
    {
        auto oldIt = m_pidEntries.find(m_selectedPid);
        if (oldIt != m_pidEntries.end() && oldIt->second.highlightActor)
            oldIt->second.highlightActor->SetVisibility(0);
    }

    m_selectedPid = pid;

    auto it = m_pidEntries.find(m_selectedPid);
    if (it != m_pidEntries.end() && it->second.highlightActor)
        it->second.highlightActor->SetVisibility(it->second.visible ? 1 : 0);
}

void PidActorManager::clearSelection()
{
    if (m_selectedPid != -1)
    {
        auto it = m_pidEntries.find(m_selectedPid);
        if (it != m_pidEntries.end() && it->second.highlightActor)
            it->second.highlightActor->SetVisibility(0);
    }
    m_selectedPid = -1;
}

std::vector<int> PidActorManager::getPidList() const
{
    std::vector<int> pids;
    pids.reserve(m_pidEntries.size());
    for (const auto& [pid, entry] : m_pidEntries)
        pids.push_back(pid);
    return pids;
}

int PidActorManager::getSelectedPid() const
{
    return m_selectedPid;
}

int PidActorManager::findPidByActor(vtkActor* actor) const
{
    for (const auto& [pid, entry] : m_pidEntries)
        if (entry.actor == actor)
            return pid;
    return -1;
}

vtkActor* PidActorManager::getActorByPid(int pid) const
{
    auto it = m_pidEntries.find(pid);
    return (it != m_pidEntries.end() && it->second.actor) ? it->second.actor.GetPointer() : nullptr;
}

vtkActor* PidActorManager::getHighlightActorByPid(int pid) const
{
    auto it = m_pidEntries.find(pid);
    return (it != m_pidEntries.end() && it->second.highlightActor)
        ? it->second.highlightActor.GetPointer() : nullptr;
}

void PidActorManager::setUserTransform(int pid, vtkLinearTransform* transform)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;
    it->second.actor->SetUserTransform(transform);
    it->second.highlightActor->SetUserTransform(transform);
}

void PidActorManager::setDamageView(int pid, bool on, vtkScalarsToColors* lut, double maxValue)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;

    vtkDataSetMapper* mapper = it->second.mapper;
    if (on)
    {
        mapper->SetScalarModeToUseCellFieldData();
        mapper->SelectColorArray(CrashDeformer::kStrainArray);
        mapper->SetLookupTable(lut);
        mapper->SetScalarRange(0.0, maxValue);
        mapper->UseLookupTableScalarRangeOff();
    }
    else
    {
        mapper->SetScalarModeToUseCellData();
        mapper->SetLookupTable(it->second.defaultLut);
        mapper->SetScalarRange(m_lutRange[0], m_lutRange[1]);
    }
}

void PidActorManager::setPbr(int pid, bool on, double metallic, double roughness)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;

    vtkProperty* p = it->second.actor->GetProperty();
    if (on)
    {
        p->SetInterpolationToPBR();
        // clear coat: the glossy lacquer layer of car paint (sharp sky reflections over the color)
        const bool paint = pid == 1 || pid == 2;
        p->SetCoatStrength(paint ? 1.0 : 0.0);
        p->SetCoatRoughness(0.04);
        p->SetCoatIOR(1.5);
        p->SetMetallic(metallic);
        p->SetRoughness(roughness);
    }
    else
    {
        p->SetInterpolationToGouraud();
    }
}
