#include "AutoPilot.h"

#include <algorithm>
#include <cmath>

#include "RoadModel.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}

void AutoPilot::reset()
{
    m_state = State::Cruise;
    m_latInt = 0.0;
    m_targetLane = 0;
    m_goalLane = 0;
    m_currentLane = 0;
    for (int k = 0; k < kLanes; ++k)
    {
        m_laneFree[k] = 1e9;
        m_laneSide[k] = false;
    }
    m_egoFree = 1e9;
    m_desiredSpeed = 0.0;
    m_tick = 0;
    m_stopDist = 1e9;
    m_redAhead = false;
}

const char* AutoPilot::stateName() const
{
    switch (m_state)
    {
    case State::Cruise:     return "CRUISE";
    case State::LaneChange: return "LANE CHANGE";
    case State::Braking:    return "BRAKING";
    case State::Aeb:        return "AEB  EMERGENCY BRAKE";
    case State::Curve:      return "CURVE";
    case State::Blocked:    return "STOPPED  ROAD BLOCKED";
    case State::RedLight:   return "RED LIGHT  STOP";
    }
    return "";
}

const char* AutoPilot::stateDescriptionKo() const
{
    switch (m_state)
    {
    case State::Cruise:     return u8"순항: 앞이 비어 있어 설정 속도로 차선을 따라 주행 중";
    case State::LaneChange: return u8"차선 변경: 앞 차선이 막혀서 비어 있는 옆 차선으로 이동 중";
    case State::Braking:    return u8"감속: 앞에 장애물/차량이 있어 안전거리에 맞춰 속도를 줄이는 중";
    case State::Aeb:        return u8"긴급제동(AEB): 충돌 직전 거리라 최대 제동 중";
    case State::Curve:      return u8"커브 감속: 앞 커브가 급해서 미끄러지지 않을 속도로 줄이는 중";
    case State::Blocked:    return u8"정지: 갈 수 있는 차선이 모두 막혀 멈춤";
    case State::RedLight:   return u8"신호 대기: 앞 신호가 빨간불(또는 멈출 수 있는 노란불)이라 정지선에 맞춰 멈추는 중";
    }
    return "";
}

void AutoPilot::perceive(const VehicleModel& v, const std::vector<LidarSensor::Hit>& hits,
                         const RoadModel& road)
{
    for (int k = 0; k < kLanes; ++k)
    {
        m_laneFree[k] = 1e9;
        m_laneSide[k] = false;
    }
    m_egoFree = 1e9;

    // the road frame: station s along the loop, lateral offset (curved road)
    const double frontLocal[3] = { 0.0, 0.5, 0.0 };
    double front[3];
    v.localToWorld(frontLocal, front);
    double sFront, latFront;
    if (!road.project(front[0], front[1], sFront, latFront))
        return;

    const double centerLocal[3] = { 2.0, 0.5, 0.0 };
    double center[3];
    v.localToWorld(centerLocal, center);
    double sEgo, egoLat;
    road.project(center[0], center[1], sEgo, egoLat);
    m_egoLat = egoLat;

    const double laneHalf = 0.63 + 0.35;  // half vehicle width (with mirrors) + margin
    const double egoHalf  = 0.63 + 0.25;
    const double horizon = std::min(sensorRange, 90.0);

    for (const auto& h : hits)
    {
        // ground, and overhead structures the truck fits under (signal arms, lamp heads, gantries)
        if (h.ground || h.p[2] < road.groundZ + 0.15 || h.p[2] > road.groundZ + 3.2)
            continue;
        // cheap pre-filter in the vehicle frame (forward = -x): ahead up to the horizon, a little
        // behind (vehicles beside us), not too far sideways (buildings, forest)
        double l[3];
        v.worldToLocal(h.p, l);
        if (-l[0] > horizon || l[0] > 10.5 || std::abs(l[1] - 0.5) > 25.0)
            continue;

        if (!h.onRoadFrame)
            continue;
        const double hs = h.roadS, lat = h.roadLat;
        if (std::abs(lat) > road.halfWidth + 0.2)
            continue;  // roadside poles, guard rails, buildings

        const double d = road.diff(hs, sFront);  // distance ahead along the road

        // beside the vehicle (from 2.5 m behind the rear to the front bumper)
        if (d >= -6.5 && d < -0.5)
        {
            for (int lane = 0; lane < kLanes; ++lane)
                if (std::abs(lat - road.laneOffset(lane)) < laneHalf)
                    m_laneSide[lane] = true;
            continue;
        }
        if (d < -0.5 || d > horizon)
            continue;

        for (int lane = 0; lane < kLanes; ++lane)
            if (std::abs(lat - road.laneOffset(lane)) < laneHalf)
                m_laneFree[lane] = std::min(m_laneFree[lane], d);

        if (std::abs(lat - egoLat) < egoHalf)
            m_egoFree = std::min(m_egoFree, d);
    }

    m_currentLane = road.laneOf(egoLat);
}

VehicleModel::Command AutoPilot::update(const VehicleModel& v, const std::vector<LidarSensor::Hit>& hits,
                                        const RoadModel& road, double dt)
{
    (void)dt;
    // perception at 50 Hz is plenty (the LiDAR delivers a full revolution every 100 ms)
    if (++m_tick % 2 == 1 || m_tick < 3)
        perceive(v, hits, road);

    const double speed = v.speed();
    const double egoLat = m_egoLat;

    // ------------------------------------------------------------
    // planning
    // ------------------------------------------------------------
    const double toTarget = std::abs(egoLat - road.laneOffset(m_targetLane));
    if (m_state == State::LaneChange && toTarget < 0.3)
        m_state = State::Cruise;
    if (m_goalLane < 0 || m_goalLane >= kLanes)
        m_goalLane = m_targetLane;

    // goal lane: the freest of my three lanes (never the oncoming lanes), chosen when my lane is
    // blocked ahead; ties go to the nearer lane
    const double trigger = std::max(20.0, 2.0 * speed + 8.0);
    if (m_laneFree[m_targetLane] < trigger && m_laneFree[m_goalLane] < trigger)
    {
        int best = m_targetLane;
        for (int lane = 0; lane < kLanes; ++lane)
        {
            const double gain = m_laneFree[lane] - m_laneFree[best];
            if (gain > 1.0 || (std::abs(gain) <= 1.0 &&
                               std::abs(lane - m_targetLane) < std::abs(best - m_targetLane)))
                best = lane;
        }
        if (m_laneFree[best] > m_laneFree[m_targetLane] + 8.0 && m_laneFree[best] > 15.0)
            m_goalLane = best;
    }

    // move one lane at a time toward the goal. A lane in between may be blocked farther ahead (we only
    // pass through it) but there must be room to get into it and nothing beside us. The next step may
    // start once we are mostly in the intermediate lane.
    if (m_goalLane != m_targetLane && (m_state != State::LaneChange || toTarget < 1.2))
    {
        const int step = m_targetLane + (m_goalLane > m_targetLane ? 1 : -1);
        // room needed in the lane we move into grows with speed: a slow car 10 m ahead in the middle
        // lane is fine at 20 km/h but not at 60 km/h (we would hit it while passing through)
        const double need = std::max(7.0, 4.0 + 1.3 * speed);
        if (!m_laneSide[step] && m_laneFree[step] > need)
        {
            m_targetLane = step;
            m_state = State::LaneChange;
        }
    }

    // obstacles ahead (passing through a lane: what matters is the goal lane and my own corridor)
    const double laneAhead = m_goalLane != m_targetLane ? m_laneFree[m_goalLane] : m_laneFree[m_targetLane];
    const double dPlan = std::min(laneAhead, m_egoFree);
    const double comfortDecel = 3.5;
    const double vObstacle = std::sqrt(2.0 * comfortDecel * std::max(0.0, dPlan - 4.0));

    // corners ahead: lateral acceleration <= 2.2 m/s^2, slowing down with 2.5 m/s^2 before the curve
    double sRear, latRear;
    road.project(v.x(), v.y(), sRear, latRear);
    double vCurve = 1e9;
    for (double d = 0.0; d <= 90.0; d += 5.0)
    {
        const double k = std::abs(road.curvature(sRear + d));
        if (k < 1e-6)
            continue;
        const double vk = std::sqrt(2.2 / k);
        vCurve = std::min(vCurve, std::sqrt(vk * vk + 2.0 * 2.5 * std::max(0.0, d - 6.0)));
    }

    // traffic light: stop 1 m before the stop line (comfortable 2.8 m/s^2)
    const double vSignal = m_stopDist < 1e8 ? std::sqrt(2.0 * 2.8 * std::max(0.0, m_stopDist - 1.0)) : 1e9;

    m_desiredSpeed = std::min({ cruiseSpeed, vObstacle, vCurve, vSignal });
    if (m_state == State::LaneChange)
        m_desiredSpeed = std::min(m_desiredSpeed, 0.85 * cruiseSpeed);

    VehicleModel::Command cmd;

    // longitudinal: P control
    const double err = m_desiredSpeed - speed;
    cmd.throttle = std::clamp(0.6 * err, 0.0, 1.0);
    cmd.brake    = std::clamp(-0.35 * err, 0.0, 1.0);

    if (m_state != State::LaneChange)
    {
        if (m_desiredSpeed < 0.3 && speed < 0.3)
            m_state = m_redAhead ? State::RedLight : State::Blocked;   // waiting at (or queued before) a red light
        else if (vSignal < cruiseSpeed - 0.5 && vSignal <= vObstacle && vSignal <= vCurve)
            m_state = State::RedLight;
        else if (vObstacle < cruiseSpeed - 0.5 && vObstacle <= vCurve)
            m_state = State::Braking;
        else if (vCurve < cruiseSpeed - 0.5)
            m_state = State::Curve;
        else
            m_state = State::Cruise;
    }

    // AEB: obstacle inside the ego corridor closer than the hard-braking distance
    const double aebDist = 2.5 + speed * speed / (2.0 * 8.0);
    if (speed > 0.5 && m_egoFree < aebDist)
    {
        m_state = State::Aeb;
        cmd.throttle = 0.0;
        cmd.brake = 1.0;
    }

    // ------------------------------------------------------------
    // lateral: pure pursuit toward the target lane center, lookahead along the road
    // ------------------------------------------------------------
    const double Ld = std::clamp(0.7 * speed + 5.0, 6.0, 26.0);
    double xt, yt;
    road.point(sRear + Ld, road.laneOffset(m_targetLane), xt, yt);
    m_lookahead[0] = xt;
    m_lookahead[1] = yt;
    m_lookahead[2] = road.groundZ + 0.05;

    // pure pursuit from the rear axle heading
    const double course = v.heading();
    const double alpha = wrapAngle(std::atan2(yt - v.y(), xt - v.x()) - course);
    const double Lact = std::hypot(xt - v.x(), yt - v.y());
    // integral on the cross-track error removes the steady understeer offset in long corners
    const double eLat = latRear - road.laneOffset(m_targetLane);   // + = right of the lane center
    if (speed > 3.0 && std::abs(eLat) < 1.2)
        m_latInt = std::clamp(m_latInt + eLat * dt, -3.0, 3.0);
    else
        m_latInt *= 0.95;
    cmd.steer = std::atan2(2.0 * v.wheelBase * std::sin(alpha), Lact) + 0.008 * m_latInt;

    return cmd;
}
