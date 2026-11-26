#pragma once

#include <vector>

#include "LidarSensor.h"
#include "RoadModel.h"
#include "VehicleModel.h"


// Perception + planning + control, using LiDAR returns only (no ground-truth obstacles).
//
//  1. Perception : non-ground LiDAR hits -> road frame (lateral offset) -> free distance per lane
//  2. Planning   : keep lane / change lane / follow-stop / AEB
//  3. Control    : pure-pursuit steering + P speed control
class AutoPilot
{
public:
    enum class State { Cruise, LaneChange, Braking, Aeb, Blocked, Curve, RedLight };

    void reset();
    void setTargetLane(int lane) { m_targetLane = m_goalLane = lane; }
    // traffic light (from the map / V2X: the LiDAR cannot see colors): distance from the front bumper
    // to the stop line where we must stop (1e9 = none), and whether a red light is close ahead
    void setSignal(double stopDistance, bool redAhead) { m_stopDist = stopDistance; m_redAhead = redAhead; }
    VehicleModel::Command update(const VehicleModel& v, const std::vector<LidarSensor::Hit>& hits,
                                 const RoadModel& road, double dt);

    State state() const { return m_state; }
    const char* stateName() const;
    const char* stateDescriptionKo() const;   // one-line explanation for the UI
    int targetLane() const { return m_targetLane; }
    int currentLane() const { return m_currentLane; }
    double laneFreeDistance(int lane) const { return m_laneFree[lane]; }
    static int laneCount() { return kLanes; }
    double egoFreeDistance() const { return m_egoFree; }
    double desiredSpeed() const { return m_desiredSpeed; }

    // lookahead target (world) for visualization
    const double* lookaheadPoint() const { return m_lookahead; }

    double cruiseSpeed = 12.0;   // [m/s]
    double sensorRange = 45.0;

private:
    void perceive(const VehicleModel& v, const std::vector<LidarSensor::Hit>& hits, const RoadModel& road);

    State m_state = State::Cruise;
    int m_targetLane = 0;
    int m_currentLane = 0;
    static constexpr int kLanes = RoadModel::kLanes;   // ego-direction lanes (oncoming lanes are never used)
    double m_laneFree[kLanes] = {};
    bool m_laneSide[kLanes] = {};            // something beside the vehicle in that lane
    double m_egoFree = 1e9;
    double m_egoLat = 0.0;
    int m_goalLane = 0;      // lane we are heading for (may be two lanes away)
    unsigned m_tick = 0;
    double m_desiredSpeed = 0.0;
    double m_latInt = 0.0;
    double m_stopDist = 1e9;
    bool m_redAhead = false;
    double m_lookahead[3] = { 0, 0, 0 };
};
