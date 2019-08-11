#include "Racer.h"

#include <cmath>
#include <algorithm>

#include <inc/natives.h>
#include "Util/StringFormat.h"
#include "Util/MathExt.h"
#include "Util/Color.h"
#include "Util/UIUtils.h"
#include "Memory/VehicleExtensions.hpp"
//#include "Settings.h"

const std::vector<Hash> headLightsOnWeathers = {
//    0x97AA0A79, // EXTRASUNNY
//    0x36A83D84, // CLEAR
//    0x30FDAF5C, // CLOUDS
//    0x10DCF4B5, // SMOG
    0xAE737644, // FOGGY
    0xBB898D2D, // OVERCAST
    0x54A69840, // RAIN
    0xB677829F, // THUNDER
    0x6DB1A50D, // CLEARING
//    0xA4CA1326, // NEUTRAL
    0xEFB6EFF6, // SNOW
    0x27EA2814, // BLIZZARD
    0x23FB812B, // SNOWLIGHT
    0xAAC9C895, // XMAS
    0xC91A3202, // HALLOWEEN
};

size_t getPointsBetween(const std::vector<Point> & coords, size_t a, size_t b) {
    if (b < a) {
        return (b + coords.size()) - a;
    }
    return b - a;
};

Racer::Racer(Vehicle vehicle, const std::string& cfgPath)
    : mVehicle(vehicle)
    , mCfg(RacerConfig::Parse(cfgPath))
    , mCfgName(cfgPath)
    , mBlip(vehicle, BlipSpriteStandard, fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                                             VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)), BlipColorYellow, true)
    , mActive(mCfg.DefaultActive)
    , mDebugView(mCfg.ShowDebug)
    , mDebugText(mCfg.ShowDebugText)
    , mDead(false)
    , mNotifyHandle(0)
    , mTrack(nullptr)   // Set to with SetTrack after creation!
    , mTrackIdx(0)      // Set to with SetTrack after creation!
    , mLapTimer(0)
    , mLapTime(0)
    , mCurrentLap(0)
    , mStatusTimer(GetRand(10000, 2000))
    , mAuxTimer(GetRand(2000, 10000))
    , mStuckTimer(2000)
    , mUnstuckTimer(1000)
    , mStuckCountThreshold(3)
    , mStuckCount(0)
    , mStuckCountTimer(30000)
    , mOutsideTimer(10000)
    , mCDistPrev(0)
    , mSteerPrev(0) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
}

Racer::~Racer() {
    try {
        if (Unloading())
            return;

        if (ENTITY::DOES_ENTITY_EXIST(mVehicle)) {
            gExt.SetThrottleP(mVehicle, 0.0f);
            gExt.SetBrakeP(mVehicle, 1.0f);
            gExt.SetSteeringAngle(mVehicle, 0.0f);
            gExt.SetHandbrake(mVehicle, false);
            ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, false, true);
            ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&mVehicle);
        }
    }
    catch (...) {
        // Discard exceptions
    }
}

void Racer::UpdateConfig(const std::string& path) {
    if (path.empty()) {
        mCfg = RacerConfig::Parse(mCfgName);
        return;
    }

    mCfgName = path;
    mCfg = RacerConfig::Parse(mCfgName);
}

void Racer::SetTrack(const Track& t) {
    mTrack = &t;
    findClosestNode(mTrackIdx);
    if (mTrackIdx == std::numeric_limits<size_t>::max()) {
        // TODO: NOTIFY
        mActive = false;
    }
}

bool Racer::IsDead() {
    return mDead;
}

void Racer::Fix() {
    if (mDead)
        return;

    float bodyHealth = VEHICLE::GET_VEHICLE_BODY_HEALTH(mVehicle);
    float engnHealth = VEHICLE::GET_VEHICLE_ENGINE_HEALTH(mVehicle);
    float fuelHealth = VEHICLE::GET_VEHICLE_PETROL_TANK_HEALTH(mVehicle);

    if (bodyHealth < 1000.0f || engnHealth < 1000.0f || fuelHealth < 1000.0f) {
        VEHICLE::SET_VEHICLE_FIXED(mVehicle);
        VEHICLE::SET_VEHICLE_DEFORMATION_FIXED(mVehicle);
        VEHICLE::SET_VEHICLE_BODY_HEALTH(mVehicle, 1000.0f);
        VEHICLE::SET_VEHICLE_ENGINE_HEALTH(mVehicle, 1000.0f);
        VEHICLE::SET_VEHICLE_PETROL_TANK_HEALTH(mVehicle, 1000.0f);
    }
}

bool Racer::GetActive() {
    return mActive;
}

void Racer::SetActive(bool value) {
    mActive = value;
}

void Racer::SetDebugView(bool value) {
    mDebugView = value;
}

bool Racer::GetDebugView() {
    return mDebugView;
}

void Racer::SetDebugText(bool value) {
    mDebugText = value;
}

Vehicle Racer::findClosestVehicle(const std::vector<Vehicle> &vehicles, Vector3 position, float radius) {
    float closest = 10000.0;
    auto closestIdx = vehicles.size();
    for (std::vector<int>::size_type i = 0; i < vehicles.size(); ++i) {
        if (!ENTITY::DOES_ENTITY_EXIST(vehicles[i])) continue;
        if (vehicles[i] == mVehicle) continue;
        
        Vector3 vehPos = ENTITY::GET_ENTITY_COORDS(vehicles[i], 1);
        float distance = Distance(position, vehPos);
        if (distance < radius) {
            if (distance < closest) {
                closest = Distance(position, vehPos);
                closestIdx = i;
            }
        }
    }
    if (closestIdx != vehicles.size()) {
        return vehicles[closestIdx];
    }
    return 0;
}

std::vector<Vector3> Racer::findOvertakingPoints(Vehicle npc) {
    std::vector<Vector3> overtakePoints(3);

    Vector3 aiDim = GetEntityDimensions(mVehicle);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 aiForward = Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle));
    float aiHeading = atan2(aiForward.y, aiForward.x);

    Vector3 npcDim = GetEntityDimensions(npc);
    Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, 1);
    Vector3 npcForward = Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(npc));
    Vector3 npcToAiVector = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(npc, aiPosition.x, aiPosition.y, aiPosition.z);
    float npcHeading = atan2(npcForward.y, npcForward.x);
    float npcSpeed = ENTITY::GET_ENTITY_SPEED(npc);

    Vector3 npcDirection = Normalize(npcPosition - aiPosition);
    Vector3 npcRelativePosition = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, npcPosition.x, npcPosition.y, npcPosition.z);
    float headingAiToNpc = GetAngleBetween(aiForward, npcDirection) * sgn(npcRelativePosition.x);
    float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

    // Add a +5% margin so same-speed cars evade each other.
    // TODO: Move 120 deg side-eye and 1.05 speedMult to settings
    // TODO: Alt: find out how to watch out for >90 deg
    if (abs(headingAiToNpc) < deg2rad(120.0f) && aiSpeed * 1.05f >= npcSpeed) {
        float diffHeading = atan2(sin(npcHeading - aiHeading), cos(npcHeading - aiHeading));

        // Make oval shape around entity to overtake, based on dimensions.
        // Add 0.5m margin
        float distMultX = npcDim.x * 0.5f + aiDim.x + 0.5f;
        float distMultY = npcDim.y * 0.5f + aiDim.x + 0.5f;

        // Translate polar coords to cartesian offset multipliers based on a unit circle
        float mulX = -cos(diffHeading + deg2rad(180.0f));
        float mulY = -sin(diffHeading + deg2rad(180.0f));

        // This thing is how far from perpendicular we are. Similar to mulX/mulY?
        // diffHeading is from -180 to 180. Make abs(it) pivot around 0, so -90 to 90.
        // then (90 - abs(that))/90 to get ratio. 0 for straight? 1 for perp?
        // could prolly be simpler
        float diffHeadingMultiplier = (deg2rad(90.0f) - abs(abs(diffHeading) - deg2rad(90.0f)))/(deg2rad(90.0f));

        float pMult = diffHeadingMultiplier;
        float nMult = (1.0f - diffHeadingMultiplier);

        float xOff = (npcToAiVector.x + aiDim.y * 1.5f * mulY) * pMult;
        float yOff = (npcToAiVector.y + aiDim.y * 1.5f * mulX) * nMult;

        float constrainX = (npcDim.x + aiDim.y) * pMult;
        float constrainY = (npcDim.y + aiDim.y) * nMult;

        float xOffset = std::clamp(xOff, -constrainX, constrainX);
        float yOffset = std::clamp(yOff, -constrainY, constrainY);

        Vector3 npcDirectionWorldCW = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, distMultX * sin(diffHeading + deg2rad(90.0f)) + xOffset, distMultY * cos(diffHeading + deg2rad(90.0f)) + yOffset, 0.0f);
        Vector3 npcDirectionWorldCCW = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, distMultX * sin(diffHeading - deg2rad(90.0f)) + xOffset, distMultY * cos(diffHeading - deg2rad(90.0f)) + yOffset, 0.0f);
        Vector3 npcDirectionWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, distMultX * sin(diffHeading + deg2rad(180.0f)) + xOffset, distMultY * cos(diffHeading + deg2rad(180.0f)) + yOffset, 0.0f);

        overtakePoints[0] = npcDirectionWorldCW;
        overtakePoints[1] = npcDirectionWorldCCW;
        overtakePoints[2] = npcDirectionWorld;

        if (mDebugView) {
            drawSphere(overtakePoints[0], 0.1250f, { 255, 0, 0, 255 });
            drawSphere(overtakePoints[1], 0.1250f, { 0, 0, 255, 255 });
            drawSphere(overtakePoints[2], 0.1250f, { 0, 255, 0, 255 });
        }

        return overtakePoints;
    }
    return {};
}

Vector3 Racer::chooseOvertakePoint(const std::vector<Point> &coords, const std::vector<Vector3> &overtakePoints, float aiLookahead, Vehicle npc, Vector3
                                   origSteerCoord, Racer::
                                   OvertakeSource& overtakeReason) {
    Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, 1);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);

    float aiTrackDist = 10000.0f;
    float npcTrackDist = 10000.0f;
    float overtakeTrackDist0 = 10000.0f;
    float overtakeTrackDist1 = 10000.0f;
    float overtakeTrackWidth0 = 0.0f;
    float overtakeTrackWidth1 = 0.0f;

    for (auto& point : coords) {
        Vector3 coord = point.v;
        float distanceAi = Distance(aiPosition, coord);
        float distanceOt0 = Distance(overtakePoints[0], coord);
        float distanceOt1 = Distance(overtakePoints[1], coord);
        float distanceNpc = Distance(npcPosition, coord);

        if (distanceAi < aiTrackDist) {
            aiTrackDist = distanceAi;
        }
        if (distanceOt0 < overtakeTrackDist0) {
            overtakeTrackDist0 = distanceOt0;
            overtakeTrackWidth0 = point.w;
        }
        if (distanceOt1 < overtakeTrackDist1) {
            overtakeTrackDist1 = distanceOt1;
            overtakeTrackWidth1 = point.w;
        }
        if (distanceNpc < npcTrackDist) {
            npcTrackDist = distanceNpc;
        }
    }

    // +5% self-speed over-estimation, sooner evasion/same-speed evasion
    Vector3 aiNextPVel = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle) * 1.05f;
    Vector3 aiNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 aiNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(aiNextVRot.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(aiNextVRot.z), 0.0f);
    Vector3 aiPredPos = (aiNextPVel + aiNextPRot) * 0.5f;
    Vector3 aiDim = GetEntityDimensions(mVehicle);
    Vector3 aiRot = ENTITY::GET_ENTITY_ROTATION(mVehicle, 0);
    Vector3 aiForward = ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle);

    // TODO: ai self size scale
    Vector3 aiPredOffLeftRear =   GetOffsetInWorldCoords(aiPredPos, aiRot, aiForward, Vector3_{ -aiDim.x * 0.5f - 0.5f, -aiDim.y * 0.5f - 0.5f, 0.0f });
    Vector3 aiPredOffLeftFront =  GetOffsetInWorldCoords(aiPredPos, aiRot, aiForward, Vector3_{ -aiDim.x * 0.5f - 0.5f,  aiDim.y * 0.5f + 0.5f, 0.0f });
    Vector3 aiPredOffRightRear =  GetOffsetInWorldCoords(aiPredPos, aiRot, aiForward, Vector3_{  aiDim.x * 0.5f + 0.5f, -aiDim.y * 0.5f - 0.5f, 0.0f });
    Vector3 aiPredOffRightFront = GetOffsetInWorldCoords(aiPredPos, aiRot, aiForward, Vector3_{  aiDim.x * 0.5f + 0.5f,  aiDim.y * 0.5f + 0.5f, 0.0f });

    Vector3 npcNextPVel = npcPosition + ENTITY::GET_ENTITY_VELOCITY(npc);
    Vector3 npcNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(npc);
    Vector3 npcNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, ENTITY::GET_ENTITY_SPEED(npc)*-sin(npcNextVRot.z), ENTITY::GET_ENTITY_SPEED(npc)*cos(npcNextVRot.z), 0.0f);
    Vector3 npcPredPos = (npcNextPVel + npcNextPRot) * 0.5f;
    Vector3 npcDim = GetEntityDimensions(npc);
    Vector3 npcRot = ENTITY::GET_ENTITY_ROTATION(npc, 0);
    Vector3 npcForward = ENTITY::GET_ENTITY_FORWARD_VECTOR(npc);

    // TODO: ai npc size scale
    Vector3 npcPredOffLeftRear =   GetOffsetInWorldCoords(npcPredPos, npcRot, npcForward, Vector3_{ -npcDim.x * 0.5f - 0.5f, -npcDim.y * 0.5f - 0.5f, 0.0f });
    Vector3 npcPredOffLeftFront =  GetOffsetInWorldCoords(npcPredPos, npcRot, npcForward, Vector3_{ -npcDim.x * 0.5f - 0.5f,  npcDim.y * 0.5f + 0.5f, 0.0f });
    Vector3 npcPredOffRightRear =  GetOffsetInWorldCoords(npcPredPos, npcRot, npcForward, Vector3_{  npcDim.x * 0.5f + 0.5f, -npcDim.y * 0.5f - 0.5f, 0.0f });
    Vector3 npcPredOffRightFront = GetOffsetInWorldCoords(npcPredPos, npcRot, npcForward, Vector3_{  npcDim.x * 0.5f + 0.5f,  npcDim.y * 0.5f + 0.5f, 0.0f });

    bool intersect2 =
        Distance(aiPredPos, npcPredPos) < std::max(aiDim.y + 0.5f, npcDim.y + 0.5f);

    bool intersect3 =
        Intersect(aiPosition, aiPredPos, npcPredOffLeftRear, npcPredOffRightRear) ||    // ai -> rear bumper
        Intersect(aiPosition, aiPredPos, npcPredOffLeftRear, npcPredOffLeftFront) ||    // ai -> left
        Intersect(aiPosition, aiPredPos, npcPredOffRightRear, npcPredOffRightFront) ||  // ai -> right
        Intersect(aiPosition, aiPredPos, npcPredOffLeftFront, npcPredOffRightFront) ||  // ai -> front bumper
        Intersect(aiPredOffLeftFront, aiPredOffRightFront, npcPredOffLeftFront, npcPredOffLeftRear) ||  // ai front -> npc left
        Intersect(aiPredOffLeftFront, aiPredOffRightFront, npcPredOffRightFront, npcPredOffRightFront); // ai front -> npc right


    if (intersect2 || intersect3) {
        // Check which overtake point has the least relative angle towards our vehicle
        float angleCW  = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[0] - aiPosition));
        float angleCCW = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[1] - aiPosition));

        float overtakePointAngleDist;
        float overtakePointAngleWidth;

        Vector3 overtakePointAngle;
        Vector3 overtakePointCenter;        // TODO: What is "center"?
        Vector3 overtakePoint;
        if (angleCW < angleCCW) {
            overtakePointAngle = overtakePoints[0];
            overtakePointAngleDist = overtakeTrackDist0;
            overtakePointAngleWidth = overtakeTrackWidth0;
        }
        else {
            overtakePointAngle = overtakePoints[1];
            overtakePointAngleDist = overtakeTrackDist1;
            overtakePointAngleWidth = overtakeTrackWidth1;
        }

        if (overtakeTrackDist0 < overtakeTrackDist1) {
            overtakePointCenter = overtakePoints[0];
        }
        else {
            overtakePointCenter = overtakePoints[1];
        }

        if (overtakePointAngle == overtakePointCenter) {
            // No conflict, commit!
            overtakeReason = OvertakeSource::Normal;
            overtakePoint = overtakePointAngle;
        }
        else {
            // Conflict! Figure out which is more harmful and choose the alternative!
            // Account for motion/reaction by extending rear
            Vector3 npcDim = GetEntityDimensions(npc);
            Vector3 offWLeftRear = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, -npcDim.x * 0.5f, -npcDim.y * 0.75f, 0.0f);
            Vector3 offWLeftFront = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, -npcDim.x * 0.5f, npcDim.y * 0.5f, 0.0f);
            Vector3 offWRightRear = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, npcDim.x * 0.5f, -npcDim.y * 0.75f, 0.0f);
            Vector3 offWRightFront = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, npcDim.x * 0.5f, npcDim.y * 0.5f, 0.0f);

            if (mDebugView) {
                drawLine(offWLeftRear, offWLeftFront, { 255, 0, 0, 255 });
                drawLine(offWLeftFront, offWRightFront, { 255, 0, 0, 255 });
                drawLine(offWRightFront, offWRightRear, { 255, 0, 0, 255 });
                drawLine(offWRightRear, offWLeftRear, { 255, 0, 0, 255 });
            }

            // Choose closest to track center when angle is less desirable
            // crosses the vehicle to overtake, or out of track
            if (Intersect(aiPosition, overtakePointAngle, offWLeftRear, offWRightFront) ||
                Intersect(aiPosition, overtakePointAngle, offWLeftFront, offWRightRear) ||
                overtakePointAngleDist > overtakePointAngleWidth + 0.75f * npcDim.x) {
                overtakePoint = overtakePointCenter;
                overtakeReason = OvertakeSource::Track;
            }
            else {
                overtakePoint = overtakePointAngle;
                overtakeReason = OvertakeSource::Angle;
            }
        }

        if (overtakeReason == OvertakeSource::Track) {
            float angleTrack = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePointCenter - aiPosition));
            float angleTrail = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[2] - aiPosition));
            if (angleTrack > angleTrail) {
                overtakePoint = overtakePoints[2];
                overtakeReason = OvertakeSource::Trail;
            }
        }

        // scale overtaking side distance with distance and speed
        float dDist = Distance(overtakePoint, aiPosition);
        if (dDist < aiLookahead) {
            float vAI = Length(ENTITY::GET_ENTITY_VELOCITY(mVehicle));
            float vNPC = Length(ENTITY::GET_ENTITY_VELOCITY(npc));
            float dV = abs(vAI - vNPC);

            // 28kph and up: full deviation (larger v, larger deviation)
            float dVRatio = std::clamp(map(dV, 0.0f, 8.0f, 0.0f, 1.0f), 0.0f, 1.0f);

            // dist car: full deviation. dist lookahead: no deviation
            float dDistRatio = std::clamp(map(dDist, aiDim.y, aiLookahead, 1.0f, 0.0f), 0.0f, 1.0f);

            // TODO: Consider max, average, mult, for these values
            // What I want: 
            // Max deviation:
            //      when going very fast relatively or
            //      when being side to side-ish
            // Min deviation:
            //      when speed matches
            //      when very far away
            // Mid deviation:
            //      low relative speed and low distance
            auto pt = map(std::max(dDistRatio, dVRatio), 0.0f, 1.0f, origSteerCoord, overtakePoint);
            return pt;
        }

    }
    overtakeReason = OvertakeSource::None;
    return {};
}

float Racer::avgCenterDiff(const std::vector<Point>& coords, uint32_t idx) {
    float diff = 0.0f;
    Vector3 a = coords[(idx + 0) % coords.size()].v;
    Vector3 b = coords[(idx + 1) % coords.size()].v;
    for (uint32_t i = 2; i < 10; ++i) {
        Vector3 c = coords[(idx + i) % coords.size()].v;

        Vector3 cwA = GetPerpendicular(a, b, coords[idx].w, true);
        Vector3 ccwA = GetPerpendicular(a, b, coords[idx].w, false);

        Vector3 cwB = GetPerpendicular(b, c, coords[idx].w, true);
        Vector3 ccwB = GetPerpendicular(b, c, coords[idx].w, false);

        float thisDiff = Distance(cwA, cwB) - Distance(ccwA, ccwB);
        diff += thisDiff;
    }
    diff /= 8.0f;
    return diff;
}

void Racer::getControls(const std::vector<Vehicle> &opponents, float limitRadians,
                        float actualAngle, Racer::InputInfo& inputs, DebugInfo& dbgInfo) {
    const std::vector<Point>& coords = mTrack->Points();
    std::vector<Vector3> overtakePoints;
    Vector3 aiDim = GetEntityDimensions(mVehicle);
    Vector3 aiNose2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, aiDim.y, 0.0f);
    float aiLookahead = ENTITY::GET_ENTITY_SPEED(mVehicle) * mCfg.LookaheadSteerSpeedMult * 1.5f;

    float searchDistance = aiLookahead > 30.0f ? aiLookahead : 30.0f;
    // Get NPC closest to front of vehicle
    Vehicle npc = findClosestVehicle(opponents, aiNose2, searchDistance);

    // Get a coordinate perpendicular to the NPC to overtake/avoid.
    if (npc)
        overtakePoints = findOvertakingPoints(npc);
    
    if (coords.size() < 2)
        return;

    if (!mActive)
        return;

    Vector3 aiVelocity = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 aiForward = aiPosition + ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle);
    float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

    float aiPitch = ENTITY::GET_ENTITY_PITCH(mVehicle);
    float pitchClp = abs(aiPitch);// std::clamp(aiPitch, mCfg.SteerLookAheadPitch, 0.0f);
    //showText(0.1f, 0.1f, 0.5f, fmt("Pitch: %.03f", pitchClp));
    float settingLAThrottle = mCfg.LookaheadThrottleSpeedMult;

    float settingLABrake = mCfg.LookaheadBrakeSpeedMult;
    float settingLASteer = mCfg.LookaheadSteerSpeedMult;

    settingLASteer = map(pitchClp, 0.0f, mCfg.SteerLookAheadPitch, settingLASteer, settingLABrake);

    float lookAheadThrottle = std::clamp(settingLAThrottle * aiSpeed, mCfg.LookaheadThrottleMinDistance, 9999.0f);
    float lookAheadBrake = std::clamp(settingLABrake * aiSpeed, mCfg.LookaheadBrakeMinDistance,          9999.0f);
    float lookAheadSteer = std::clamp(settingLASteer * aiSpeed, mCfg.LookaheadSteerMinDistance,          9999.0f);

    uint32_t throttleIdx;
    uint32_t brakeIdx;
    uint32_t steerIdx;

    Vector3 nextPositionThrottle = getCoord(coords, lookAheadThrottle, actualAngle, dbgInfo.laSrcThrottle, throttleIdx);
    Vector3 nextPositionBrake = getCoord(coords, lookAheadBrake, actualAngle, dbgInfo.laSrcBrake, brakeIdx);

    Vector3 nextPositionSteer = getCoord(coords, lookAheadSteer, actualAngle, dbgInfo.laSrcSteer, steerIdx);
    Vector3 origPosSteer = nextPositionSteer;
    {
        Vector3 steerA = coords[(steerIdx + 0) % coords.size()].v;
        Vector3 steerB = coords[(steerIdx + 1) % coords.size()].v;

        float steerDiff = avgCenterDiff(coords, steerIdx);
        float brakeDiff = avgCenterDiff(coords, brakeIdx);

        if (abs(brakeDiff) > abs(steerDiff))
            steerDiff = -brakeDiff;

        float cDist = lerp(
            mCDistPrev, 
            map(std::clamp(steerDiff, -2.0f, 2.0f), -2.0f, 2.0f, -coords[steerIdx].w, coords[steerIdx].w), 
            1.0f - pow(0.200f, GAMEPLAY::GET_FRAME_TIME()));
        mCDistPrev = cDist;

        nextPositionSteer = GetPerpendicular(steerA, steerB, cDist, false);
        //drawSphere(nextPositionSteer, 0.5f, { 0 ,0 ,0 , 255 });
    }

    if (overtakePoints.size() == 3) {
        Vector3 overtakePoint = chooseOvertakePoint(coords, overtakePoints, aiLookahead, npc, origPosSteer, dbgInfo.overtakeSource);
        if (Length(overtakePoint) > 0.0f) {
            nextPositionSteer = overtakePoint;
        }
    }

    Vector3 nextVectorThrottle = Normalize(nextPositionThrottle - aiPosition);
    Vector3 nextVectorSteer = Normalize(nextPositionSteer - aiPosition);
    Vector3 nextVectorBrake = Normalize(nextPositionBrake - aiPosition);

    Vector3 forwardVector = Normalize(aiForward - aiPosition);

    float aiHeading = atan2(forwardVector.y, forwardVector.x);
    float nextHeadingThrottle = atan2(nextVectorThrottle.y, nextVectorThrottle.x);
    float nextHeadingSteer = atan2(nextVectorSteer.y, nextVectorSteer.x);
    float nextHeadingBrake = atan2(nextVectorBrake.y, nextVectorBrake.x);

    float throttleBrakeHeading(atan2(nextPositionThrottle.y - nextPositionBrake.y, nextPositionThrottle.x - nextPositionBrake.x));
    float diffNodeHeading = atan2(sin(throttleBrakeHeading - aiHeading), cos(throttleBrakeHeading - aiHeading));

    float turnThrottle = atan2(sin(nextHeadingThrottle - aiHeading), cos(nextHeadingThrottle - aiHeading));
    float turnSteer = atan2(sin(nextHeadingSteer - aiHeading), cos(nextHeadingSteer - aiHeading));
    float turnBrake = atan2(sin(nextHeadingBrake - aiHeading), cos(nextHeadingBrake - aiHeading));

    float distanceThrottle = Distance(aiPosition, nextPositionThrottle);
    float distanceBrake = Distance(aiPosition, nextPositionBrake);

    float distPerpThrottle = abs((abs(turnThrottle) - 1.5708f) / 1.5708f);
    float distPerpBrake = abs((abs(turnBrake) - 1.5708f) / 1.5708f);

    float steerMult = mCfg.SteerMult;

    inputs.throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    inputs.throttle *= map(distPerpThrottle, 0.0f, 1.0f, 0.5f, 1.0f);

    // Decrease throttle when starting to spin out, increase countersteer
    Vector3 nextPositionVelocity = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(rotationVelocity.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(rotationVelocity.z), 0.0f);

    float angle = GetAngleBetween(ENTITY::GET_ENTITY_VELOCITY(mVehicle), turnWorld - aiPosition);
    float csMult = std::clamp(map(angle, deg2rad(mCfg.CountersteerIncreaseStartAngle), deg2rad(mCfg.CountersteerIncreaseEndAngle), 1.0f, 2.0f), 0.0f, 2.0f);
    float spinoutMult = std::clamp(map(angle, deg2rad(mCfg.ThrottleDecreaseStartAngle), deg2rad(mCfg.ThrottleDecreaseEndAngle), 1.0f, 0.0f), 0.0f, 1.0f);

    // TODO: Clean up? Check for use outside offroad slippery driving
    // Yank the handbrake when understeering?
    Vector3 velocityWorld = ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 travelWorld = velocityWorld + positionWorld;
    Vector3 travelRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, travelWorld.x, travelWorld.y, travelWorld.z);

    bool understeering = false;
    float steeringAngleRelX = ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(actualAngle);
    Vector3 turnRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, turnWorld.x, turnWorld.y, turnWorld.z);
    float turnRelativeNormX = (travelRelative.x + turnRelative.x) / 2.0f;
    if (steeringAngleRelX > turnRelativeNormX && turnRelativeNormX > travelRelative.x ||
        steeringAngleRelX < turnRelativeNormX && turnRelativeNormX < travelRelative.x) {
        understeering = true;
    }
    if (understeering && abs(turnSteer) > mCfg.UndersteerHandbrakeTrigger && aiSpeed > 5.0f) {
        inputs.handbrake = true;
        inputs.throttle = 0.0f;
    }

    // start oversteer detect
    float oversteerAngle = acos(aiVelocity.y / ENTITY::GET_ENTITY_SPEED(mVehicle))* 180.0f / 3.14159265f;
    if (isnan(oversteerAngle))
        oversteerAngle = 0.0;

    if (!understeering && oversteerAngle > mCfg.OversteerDetectionAngle && aiVelocity.y > 5.0f) {
        inputs.throttle *= spinoutMult;
        steerMult *= csMult;
        dbgInfo.oversteerCompensateThrottle = spinoutMult < 1.0f;
        dbgInfo.oversteerCompensateSteer = steerMult > 1.0f;
    }

    // Initial brake application
    float maxBrake = 0.0f;
    // maxBrake = map(aiSpeed, distanceThrottle * mCfg.BrakePointDistanceThrottleMult, distanceBrake * mCfg.BrakePointDistanceBrakeMult, -0.3f, 3.0f);
    // TODO: Expose 0.5f and 1.5f modifiers. Right more = more aggro/late brakey // 2.0 is also ok?
    maxBrake = map(aiSpeed, 0.5f * distanceBrake * mCfg.BrakePointDistanceBrakeMult, 2.0f * distanceBrake * mCfg.BrakePointDistanceBrakeMult, 0.0f, 1.0f);

    float brakeDiffThrottleBrake = 0.0f;
    //showText(0.1f, 0.00f, 0.5f, fmt("aiHeading: %.03f", aiHeading));
    //showText(0.1f, 0.05f, 0.5f, fmt("throttleBrakeHeading: %.03f", throttleBrakeHeading));
    //showText(0.1f, 0.10f, 0.5f, fmt("diffNodeHeading: %.03f", diffNodeHeading));
    if (Distance(aiPosition, nextPositionThrottle) < lookAheadThrottle * 1.5f && abs(diffNodeHeading) - abs(actualAngle) > deg2rad(mCfg.BrakePointHeadingMinAngle) && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        brakeDiffThrottleBrake = map(abs(diffNodeHeading) - abs(actualAngle) - deg2rad(mCfg.BrakePointHeadingMinAngle), 0.0f, deg2rad(mCfg.BrakePointHeadingMaxAngle - mCfg.BrakePointHeadingMinAngle), 0.0f, 1.0f);
        brakeDiffThrottleBrake *= std::clamp(map(aiSpeed, mCfg.BrakePointHeadingMinSpeed, mCfg.BrakePointHeadingMaxSpeed, 0.0f, 1.0f), 0.0f, 1.0f);
        if (brakeDiffThrottleBrake > maxBrake) {
            maxBrake = brakeDiffThrottleBrake;
            //dbgBrakeForHeading = true;
        }
    }

    if (abs(turnBrake) > limitRadians && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        float brakeTurn = map(distPerpBrake, 0.0f, 1.0f, 1.0f, 0.0f);
        if (brakeTurn > maxBrake) {
            maxBrake = brakeTurn;
            //dbgBrakeForAngle = true;
        }
    }

    // Track limits
    {

        Vector3 predictedPos = (nextPositionVelocity + turnWorld) * 0.5f;
        uint32_t idx = static_cast<uint32_t>(coords.size());
        Point trackClosestPred = getTrackCoordNearCoord(coords, predictedPos, idx);
        uint32_t x;
        auto trackClosestAi = getTrackCoordNearCoord(coords, aiPosition, x);
        if (Distance(aiPosition, trackClosestAi.v) < trackClosestAi.w * 2.0f) {

            // TODO: handle idx == coords.size();

            // This width offset is always closer to the track edge on the inside of the corner, than the center.
            float yeet = avgCenterDiff(coords, idx);
            Vector3 yeetV = GetPerpendicular(coords[idx].v, coords[(idx + 1) % coords.size()].v, yeet, false);

            // Are we on the inside of a corner?
            bool inside = Distance(predictedPos, yeetV) < Distance(predictedPos, trackClosestPred.v) &&
                Distance(trackClosestPred.v, yeetV) < Distance(trackClosestPred.v, predictedPos);

            dbgInfo.trackLimitsInside = inside;

            // Negative: Inside track. Positive: Outside track.
            float overshoot = Distance(predictedPos, trackClosestPred.v) - trackClosestPred.w;

            if (overshoot > mCfg.TrackLimitsAdjustMinOvershoot && !inside &&
                /*smallestDistanceAI < turnTrackClosest.w * 1.5f &&*/ aiSpeed > 5.0f) {
                dbgInfo.trackLimits = 1;
                inputs.throttle *= std::clamp(
                    map(overshoot,
                        mCfg.TrackLimitsAdjustMinOvershoot, mCfg.TrackLimitsAdjustMaxOvershoot,
                        mCfg.TrackLimitsThrottleMultMinOvershoot, mCfg.TrackLimitsThrottleMultMaxOvershoot)
                    , 0.0f, 1.0f
                );
                turnSteer *= map(overshoot,
                    mCfg.TrackLimitsAdjustMinOvershoot, mCfg.TrackLimitsAdjustMaxOvershoot,
                    mCfg.TrackLimitsSteerMultMinOvershoot, mCfg.TrackLimitsSteerMultMaxOvershoot);
                if (overshoot > mCfg.TrackLimitsAdjustMaxOvershoot) {
                    float overshootBrake = map(overshoot,
                        mCfg.TrackLimitsAdjustMaxOvershoot, mCfg.TrackLimitsAdjustMaxOvershoot * 2.0f,
                        0.0f, 1.0f);
                    if (overshootBrake > maxBrake) {
                        maxBrake = overshootBrake;
                        inputs.throttle = 0.0f;
                        dbgInfo.trackLimits = 2;
                    }
                }
            }
            if (overshoot > mCfg.TrackLimitsAdjustMinOvershoot && inside && aiSpeed > 5.0f) {
                dbgInfo.trackLimits = 1;
                turnSteer *=
                    std::clamp(
                        map(overshoot,
                            0.0f, mCfg.TrackLimitsAdjustMinOvershoot,
                            1.0f, 0.0f)
                        , 0.0f, 1.0f);
            }
        }
        if (dbgInfo.trackLimits) {
            // Ignore overtaking when (heading) out of track limits
            nextPositionSteer = origPosSteer;
        }
    }

    // TODO: Clean up, consider braking _earlier_ instead of _harder_.
    // Stop yeeting off cliffs
    float aiGndZ = 0.0f;
    float laGndZ = 0.0f;
    Vector3 laPhy = nextPositionBrake;// (nextPositionVelocity + turnWorld) * 0.5f;
    bool aiGnd = GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(aiPosition.x, aiPosition.y, aiPosition.z + aiDim.z, &aiGndZ, 0);
    bool laGnd = GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(laPhy.x, laPhy.y, laPhy.z + aiDim.z, &laGndZ, 0);

    if (aiGnd && laGnd) {
        float drop = aiGndZ - laGndZ;
        float dropDangerMult = map(drop, mCfg.ElevationMin, mCfg.ElevationMax, mCfg.ElevationDangerMin, mCfg.ElevationDangerMax);

        if (drop > mCfg.ElevationDropThreshold) {
            maxBrake *= dropDangerMult;

            showText(0.2f, 0.000f, 0.5f, fmt("~r~%.03f m drop", drop));
            showText(0.2f, 0.025f, 0.5f, fmt("~r~%.03f dropDangerMult", dropDangerMult));
            //showText(2.0f, 0.050f, 0.5f, fmt("~r~Accel: %.03f", throttle));
            showText(0.2f, 0.075f, 0.5f, fmt("~r~Brake: %.03f", maxBrake));
        }
    }

    if (dbgInfo.overtakeSource == OvertakeSource::Trail) {
        Vector3 relVelDiff = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true) - ENTITY::GET_ENTITY_SPEED_VECTOR(npc, true);
        Vector3 absVelDiff = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, false) - ENTITY::GET_ENTITY_SPEED_VECTOR(npc, false);
        // match speed?
        Vector3 npcDim = GetEntityDimensions(npc);
        Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, true);
        float dist = Distance(aiPosition, npcPosition);
        if (dist < aiDim.y + npcDim.y) {
            inputs.throttle = 0.0f;
        }
        if (dist < aiDim.y + npcDim.y * 0.5f) {
            inputs.brake = 1.0f;
        }
    }

    inputs.throttle = std::clamp(inputs.throttle, 0.0f, 1.0f);
    inputs.brake = std::clamp(maxBrake, 0.0f, 1.0f);
    inputs.steer = std::clamp(turnSteer * steerMult, -1.0f, 1.0f);
    
    if (inputs.brake > 0.7f)
        inputs.throttle = 0.0f;
    if (inputs.brake < 0.15f)
        inputs.brake = 0.0f;

    if (inputs.throttle > 0.95f)
        inputs.throttle = 1.0f;

    if (inputs.throttle > 0.95f && inputs.brake < 0.33f)
        inputs.brake = 0.0f;

    dbgInfo.nextPositionThrottle = nextPositionThrottle;
    dbgInfo.nextPositionBrake = nextPositionBrake;
    dbgInfo.nextPositionSteer = nextPositionSteer;
    dbgInfo.nextPositionVelocity = nextPositionVelocity;
    dbgInfo.nextPositionRotation = turnWorld;
    dbgInfo.oversteerAngle = oversteerAngle;
    // dbgInfo.oversteerCompensateThrottle above
    // dbgInfo.oversteerCompensateSteer above
    dbgInfo.understeering = understeering;
    //dbgInfo.trackLimits above
}

void Racer::UpdateControl(const std::vector<Vehicle> &opponents) {
    if (mStatusTimer.Expired()) {
        mStatusTimer.Reset(GetRand(10000, 2000));
        updateStatus();
    }

    if (mDead) {
        return;
    }

    if (mAuxTimer.Expired()) {
        mAuxTimer.Reset(GetRand(2000, 10000));
        updateAux();
    }

    //updateLapTimers(coords);
    updateStuck();

    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(mVehicle))
        VEHICLE::SET_VEHICLE_ENGINE_ON(mVehicle, true, true, true);

    float actualAngle = getSteeringAngle();
    float limitRadians = gExt.GetMaxSteeringAngle(mVehicle);
    float reduction = calculateReduction();

    DebugInfo dbgInfo{};
    InputInfo inputs = { 0.0f, 1.0f, 0.0f, false };
    getControls(opponents, limitRadians, actualAngle, inputs, dbgInfo);

    if (mStuckTimer.Expired()) {
        inputs.throttle = -0.4f;
        inputs.brake = 0.0f;
        inputs.steer = 0.0f;
        inputs.handbrake = false;
    }

    if (inputs.handbrake) {
        inputs.brake = 0.0f;
        reduction = 1.0f;
    }

    float desiredHeading = calculateDesiredHeading(limitRadians, inputs.steer, reduction);

    gExt.SetThrottle(mVehicle, inputs.throttle);
    gExt.SetThrottleP(mVehicle, inputs.throttle);

    gExt.SetBrakeP(mVehicle, inputs.brake);
    if (inputs.brake > 0.15f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, false);

    // clamp steering speed to 1 rot/s 
    // typically side-to-side in 0.2 seconds with 36 degrees steering angle
    float deltaT = GAMEPLAY::GET_FRAME_TIME();
    float maxDeltaSteer = 3.14f * 2.0f; // 2 turns/s
    float newSteer = desiredHeading;
    if (abs(mSteerPrev - desiredHeading)/deltaT > maxDeltaSteer) {
        newSteer = mSteerPrev - sgn(mSteerPrev - desiredHeading) * maxDeltaSteer * deltaT;
    }
    gExt.SetSteeringAngle(mVehicle, newSteer);
    gExt.SetHandbrake(mVehicle, inputs.handbrake);

    if (mDebugView)
        displayDebugInfo(inputs, dbgInfo);

    mSteerPrev = newSteer;
    updateLapTimers(); // prevIdx was updated here so need to be low
}

void Racer::updateStatus() {
    if (!VEHICLE::IS_VEHICLE_DRIVEABLE(mVehicle, 0) || ENTITY::IS_ENTITY_DEAD(mVehicle)) {
        mBlip.SetSprite(BlipSpriteDead);
        mBlip.SetColor(BlipColorRed);
        mBlip.SetName(mBlip.GetName());
    }
    else {
        mBlip.SetSprite(BlipSpriteStandard);
        mBlip.SetColor(BlipColorYellow);
        mBlip.SetName(mBlip.GetName());
    }
    if (!VEHICLE::IS_VEHICLE_DRIVEABLE(mVehicle, 0) || ENTITY::IS_ENTITY_DEAD(mVehicle)) {
        mDead = true;
    }
}

void Racer::updateLapTimers() {
    const std::vector<Point>& points = mTrack->Points();
    size_t currPointIdx = mTrackIdx;
    bool finishPassed = false;
    // Find coord closest to ourselves
    float smallestToAi = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    for (auto i = 0ull; i < points.size(); ++i) {
        float distanceAi = Distance(aiPosition, points[i].v);
        if (distanceAi < smallestToAi) {
            smallestToAi = distanceAi;
            currPointIdx = i;
        }
    }

    // TODO: Verify if it always successfully passes point zero / no overlap / doesn't jump
    if (getPointsBetween(points, mTrackIdx, currPointIdx) < 10) {
        if (currPointIdx == 0 && mTrackIdx == points.size() - 1)
            finishPassed = true;
        // We've progressed! Great!
        mTrackIdx = currPointIdx;
    }

    if (finishPassed) {
        mLapTime = mLapTimer.Elapsed();
        mLapTimer.Reset();
        uint64_t min = (mLapTime / (1000 * 60)) % 60;
        uint64_t sec = (mLapTime / 1000) % 60;
        uint64_t mil = (mLapTime % 1000);
        notify(fmt("Last lap: %d:%02d.%03d", min, sec, mil));
    }
}

void Racer::updateAux() {
    bool headlightsOn = false;
    headlightsOn |= std::find(headLightsOnWeathers.begin(), headLightsOnWeathers.end(), GAMEPLAY::GET_PREV_WEATHER_TYPE_HASH_NAME()) != headLightsOnWeathers.end();
    headlightsOn |= TIME::GET_CLOCK_HOURS() > 19 || TIME::GET_CLOCK_HOURS() < 6;
    VEHICLE::SET_VEHICLE_LIGHTS(mVehicle, headlightsOn ? 3 : 4);
    if (mCfg.AutoRepair) {
        Fix();
    }
}

Point Racer::findClosestNode(size_t& trackIdx) {
    Point closestPoint{};
    trackIdx = std::numeric_limits<size_t>::max();

    float smallestDistanceAI = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    for (size_t i = 0; i < mTrack->Points().size(); ++i) {
        Point point = mTrack->Points()[i];
        Vector3 coord = point.v;
        float distanceAI = Distance(aiPosition, coord);
        if (distanceAI < smallestDistanceAI) {
            smallestDistanceAI = distanceAI;
            closestPoint = point;
            trackIdx = i;
        }
    }
    return closestPoint;
}

void Racer::teleportToLastNode() {
    size_t trackSz = mTrack->Points().size();
    Point lastPoint = mTrack->Points()[mTrackIdx];
    Point nextPoint = mTrack->Points()[(mTrackIdx + 1) % trackSz];
    Vector3 vector = nextPoint.v - lastPoint.v;
    vector = Normalize(vector);
    float heading = rad2deg(atan2(vector.y, vector.x)) - 90.0f;
    ENTITY::SET_ENTITY_COORDS(mVehicle, lastPoint.v.x, lastPoint.v.y, lastPoint.v.z, 0, 0, 0, 0);
    ENTITY::SET_ENTITY_HEADING(mVehicle, heading);
    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
}

void Racer::updateStuck() {
    auto coords = mTrack->Points();
    if (coords.size() < 2 || !mActive || VEHICLE::GET_PED_IN_VEHICLE_SEAT(mVehicle, -1) == PLAYER::PLAYER_PED_ID()) {
        mStuckCount = 0;
        mStuckTimer.Reset();
        mUnstuckTimer.Reset();
        mStuckCountTimer.Reset();
        mOutsideTimer.Reset();
        return;
    }

    if (ENTITY::GET_ENTITY_SPEED(mVehicle) > 0.5f && !mStuckTimer.Expired()) {
        mStuckTimer.Reset();
    }

    if (!mStuckTimer.Expired()) {
        mUnstuckTimer.Reset();
    }

    if (mUnstuckTimer.Expired()) {
        notify(fmt("Unstuck attempt %d", mStuckCount + 1));
        if (!VEHICLE::IS_VEHICLE_ON_ALL_WHEELS(mVehicle)) {
            VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
        }
        mStuckTimer.Reset();
        mUnstuckTimer.Reset();
        mStuckCount++;
    }

    if (mStuckCountTimer.Expired()) {
        mStuckCount = 0;
        mStuckCountTimer.Reset();
    }

    if (mStuckCount == 0) {
        mStuckCountTimer.Reset();
    }

    if (mStuckCount > mStuckCountThreshold) {
        notify(fmt("Teleporting to track (after %d attempts)", mStuckCount));
        teleportToLastNode();
        mStuckCount = 0;
        mStuckCountTimer.Reset();
    }

    {
        float smallestDistanceAI = 10000.0f;
        Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
        Point aiTrackClosest(aiPosition , 5.0f);
        for (auto& point : coords) {
            Vector3 coord = point.v;
            float distanceAI = Distance(aiPosition, coord);
            if (distanceAI < smallestDistanceAI) {
                smallestDistanceAI = distanceAI;
                aiTrackClosest = point;
            }
        }
        if (smallestDistanceAI <= aiTrackClosest.w) {
            mOutsideTimer.Reset();
        }
        if (mOutsideTimer.Expired()) {
            notify(fmt("Teleporting to track (outside track %d ms)", mOutsideTimer.Elapsed()));
            teleportToLastNode();
            mOutsideTimer.Reset();
        }
    }

    //showText(0.5f, 0.000f, 0.5f, fmt("StuckTimer %s", mStuckTimer.Expired() ? "expired" : ""));
    //showText(0.5f, 0.025f, 0.5f, fmt("UnstuckTimer %s", mUnstuckTimer.Expired() ? "expired" : ""));
    //showText(0.5f, 0.050f, 0.5f, fmt("Stuck count %d", mStuckCount));
    //showText(0.5f, 0.075f, 0.5f, fmt("StuckT %d", mStuckTimer.Elapsed()));
    //showText(0.5f, 0.100f, 0.5f, fmt("UnstuckT %d", mUnstuckTimer.Elapsed()));
    //showText(0.5f, 0.125f, 0.5f, fmt("Outside for %d ms", mOutsideTimer.Elapsed()));
}

Vehicle Racer::GetVehicle() {
    return mVehicle;
}

Vector3 Racer::getCoord(const std::vector<Point> &coords,
                        float lookAheadDistance, float actualAngle, Racer::LookAheadSource& source) {
    uint32_t discard;
    return getCoord(coords, lookAheadDistance, actualAngle, source, discard);
}

Point Racer::getTrackCoordNearCoord(const std::vector<Point>& trackCoords, Vector3 findCoord, uint32_t& outIndex) {
    outIndex = static_cast<uint32_t>(trackCoords.size()); // return trackCoords.end() basically

    float smallestDistance = 10000.0f;
    Point closestPoint{};
    for (uint32_t i = 0; i < trackCoords.size(); ++i) {
        const Point& point = trackCoords[i];
        Vector3 trackCoord = point.v;
        float distance = Distance(findCoord, trackCoord);
        if (distance < smallestDistance) {
            smallestDistance = distance;
            closestPoint = point;
            outIndex = i;
        }
    }
    return closestPoint;
}

Vector3 Racer::getCoord(const std::vector<Point> &coords,
                        float lookAheadDistance, float actualAngle, Racer::LookAheadSource& source, uint32_t& index) {
    float smallestToLa = 9999.9f;
    size_t smallestToLaIdx = mTrackIdx;
    float smallestToAi = 9999.9f;
    int smallestToAiIdx = 0;

    Vector3 aiPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, 0.0f);

    float steeringAngleRelX = lookAheadDistance * -sin(actualAngle);
    float steeringAngleRelY = lookAheadDistance * cos(actualAngle);

    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);

    for (auto i = 0; i < coords.size(); ++i) {
        float distanceAi = Distance(aiPos, coords[i].v);
        float distanceLa = Distance(aiForward, coords[i].v);

        if (distanceAi < smallestToAi) {
            smallestToAi = distanceAi;
            smallestToAiIdx = i;
        }

        if (distanceLa < smallestToLa) {
            smallestToLa = distanceLa;
            smallestToLaIdx = i;
        }
    }

    size_t returnIndex = smallestToLaIdx;
    source = LookAheadSource::Normal;

    //// Only consider viable nodes, to not cut the track. 
    //// Significant overshoot still makes AI choose closest track node, 
    //// so prevent this from happening with walls or something. 
    //int nodeToConsiderMin = static_cast<int>(1.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));
    //int nodeToConsiderMax = static_cast<int>(2.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));

    //if ((smallestToLaIdx > smallestToAiIdx + nodeToConsiderMax || smallestToLaIdx < smallestToAiIdx - nodeToConsiderMax) &&
    //    smallestToAiIdx > nodeToConsiderMin && smallestToAiIdx < coords.size() - nodeToConsiderMin) {
    //    // Ensure track is followed continuously (no cutting off entire sections)
    //    returnIndex = (smallestToAiIdx + nodeToConsiderMin) % coords.size();
    //    source = LookAheadSource::Continuous;
    //}
    //else if (smallestToAiIdx >= smallestToLaIdx && smallestToAiIdx - smallestToLaIdx < coords.size() / 2) {
    //    // Ensure going forwards
    //    returnIndex = (smallestToAiIdx + (int)lookAheadDistance) % coords.size();
    //    source = LookAheadSource::Forward;
    //}

    // OK!!!! SO!!! ok uhhh
    // we need to make sure we traverse the track in a consistent and smooth way
    // this means a few things that went wrong before in the commented code above
    // 1. deal with crossings of the track, a figure eight, so to say
    // 2. deal with returning to track after leaving - previously it fishtailed because it focused too much on start
    // 3. deal with start/stop since i have no idea how to handle the index rollover situation



    // oh no, it wants to go backwards somehow
    if (getPointsBetween(coords, mTrackIdx, returnIndex) > static_cast<size_t>(lookAheadDistance * 1.50f)) {
        returnIndex = (mTrackIdx + static_cast<size_t>(lookAheadDistance)) % coords.size();
        //showText(0.0f, 0.3f, 0.25f, "Tryna cheat???");
    }

    // oh no it's tryna cheat
    //if (getPointsBetween(coords, mTrackIdx, returnIndex) > static_cast<size_t>(lookAheadDistance * 1.05f)) {
    //    returnIndex = (mTrackIdx + static_cast<size_t>(lookAheadDistance)) % coords.size();
    //}

    // oh no it's too close to going perpendicular
    if (getPointsBetween(coords, mTrackIdx, returnIndex) < static_cast<size_t>(lookAheadDistance * 0.50f)) {
        returnIndex = (mTrackIdx + static_cast<size_t>(lookAheadDistance)) % coords.size();
        //showText(0.0f, 0.3f, 0.25f, "2close??");
    }

    index = returnIndex;
    return coords[returnIndex].v;
}

float Racer::getSteeringAngle() {
    float largestAngle = 0.0f;
    auto angles = gExt.GetWheelSteeringAngles(mVehicle);

    for (auto angle : angles) {
        if (abs(angle) > abs(largestAngle)) {
            largestAngle = angle;
        }
    }
    return largestAngle;
}

float Racer::calculateReduction() {
    float mult = 1;
    Vector3 vel = ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 pos = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 motion = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, pos.x + vel.x, pos.y + vel.y,
                                                                       pos.z + vel.z);
    if (motion.y > 3) {
        mult = 0.15f + powf(0.9f, abs(motion.y) - 7.2f);
        if (mult != 0) { mult = floorf(mult * 1000) / 1000; }
        if (mult > 1) { mult = 1; }
    }
    mult = (1 + (mult - 1) * 1.0f);
    return mult;
}

float Racer::calculateDesiredHeading(float steeringMax, float desiredHeading,
                                     float reduction) {
    float correction = desiredHeading * reduction;

    if (abs(ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y) > 3.0f) {
        Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
        Vector3 travelWorld = positionWorld + ENTITY::GET_ENTITY_VELOCITY(mVehicle);

        Vector3 target = Normalize(
            ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, travelWorld.x, travelWorld.y, travelWorld.z));
        float travelDir = atan2(target.y, target.x) - M_PI / 2.0f;
        if (travelDir > M_PI / 2.0f) {
            travelDir -= M_PI;
        }
        if (travelDir < -M_PI / 2.0f) {
            travelDir += M_PI;
        }
        travelDir *= sgn(ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y);
        travelDir += desiredHeading * reduction;

        correction = atan2(sin(travelDir), cos(travelDir));
    }
    if (correction > steeringMax)
        correction = steeringMax;
    if (correction < -steeringMax)
        correction = -steeringMax;

    return correction;
}

void Racer::displayDebugInfo(const Racer::InputInfo& inputs, const Racer::DebugInfo& dbgInfo) {
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    float actualAngle = getSteeringAngle();

    drawLine(aiPosition, dbgInfo.nextPositionThrottle, solidGreen);
    drawSphere(dbgInfo.nextPositionThrottle, 0.25f, solidGreen);
    drawLine(aiPosition, dbgInfo.nextPositionSteer, solidBlue);
    drawSphere(dbgInfo.nextPositionSteer, 0.25f, solidBlue);
    drawLine(aiPosition, dbgInfo.nextPositionBrake, solidRed);
    drawSphere(dbgInfo.nextPositionBrake, 0.25f, solidRed);

    // draw chevron
    Vector3 p = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    Vector3 min, max;
    GAMEPLAY::GET_MODEL_DIMENSIONS(ENTITY::GET_ENTITY_MODEL(mVehicle), &min, &max);
    Vector3 up = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 1.0f);

    // ActualAngle - 90deg to rotate the chevron
    float debugAngle = actualAngle - deg2rad(90.0f);
    float steeringAngleRelX = -sin(debugAngle);
    float steeringAngleRelY = cos(debugAngle);

    Vector3 forward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);
    Vector3 dir = forward - p;
    Vector3 rot{};
    rot.y = 90.0f;

    Color c{
        std::clamp(static_cast<int>(map(inputs.brake, 0.0f, 1.0f, 0.0f, 255.0f)), 127, 255),
        std::clamp(static_cast<int>(map(inputs.brake, 0.0f, 1.0f, 255.0f, 0.0f)), 127, 255),
        0,
        255
    };

    if (mStuckTimer.Expired()) {
        drawSphere(up, 0.5f, solidRed);
    }
    else if (inputs.handbrake) {
        drawSphere(up, 0.5f, solidYellow);
    }
    else {
        drawChevron(up, dir, rot, 1.0f, inputs.throttle, c);
    }

    Vector3 predictedPos = (dbgInfo.nextPositionVelocity + dbgInfo.nextPositionRotation) * 0.5f;
    drawLine(aiPosition, predictedPos , solidWhite);
    drawSphere(predictedPos, 0.25f, solidWhite);

    // Debug text
    if (mDebugText) {
        Vector3 up2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 2.0f);
        showDebugInfo3D(up2, 10.0f, {
            fmt("%s(P) %s(ESC) %s(ABS)", 
                inputs.handbrake ? "~r~" : "~m~", 
                dbgInfo.oversteerCompensateThrottle || dbgInfo.oversteerCompensateSteer ? "~o~" : "~m~",
                dbgInfo.abs ? "~r~" : "~m~"),
            fmt("%sUnder ~m~| %sOver", 
                dbgInfo.understeering ? "~r~" : "~m~", 
                dbgInfo.oversteerAngle > mCfg.OversteerDetectionAngle ? "~r~" : "~m~"),
            fmt("%sTrack Limits %s", 
                dbgInfo.trackLimits == 2 ? "~r~" : dbgInfo.trackLimits == 1 ? "~o~" : "~m~", dbgInfo.trackLimitsInside ? "In" : "Out"),
            fmt("%sT: %03d%% ~m~| %sB: %03d%%", 
                inputs.throttle > 0.5 ? "~g~" : "~m~", 
                static_cast<int>(inputs.throttle * 100.0f), 
                inputs.brake > 0.5 ? "~r~" : "~m~", 
                static_cast<int>(inputs.brake * 100.0f)),
            });
    }

    // TODO: Consider dumping the following somewhere...
    //auto currentLapTime = mLapTimer.Elapsed();
    //std::string previousLapTimeFmt = fmt("%02d:%02d.%03d", mLapTime / 60000, (mLapTime / 1000) % 60, mLapTime % 1000);
    //std::string liveLapTimeFmt = fmt("%02d:%02d.%03d", currentLapTime / 60000, (currentLapTime / 1000) % 60, currentLapTime % 1000);
}

void Racer::notify(const std::string& msg) {
    notify(msg, false);
}

void Racer::notify(const std::string& msg, bool alwaysShow) {
    if (mDebugView || alwaysShow) {
        std::string name = getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle));
        std::string plate = VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle);

        showNotification(fmt("~b~%s (~r~%s~b~)\n~w~%s", 
            name.c_str(), plate.c_str(), msg.c_str()), 
            &mNotifyHandle);
    }
}
