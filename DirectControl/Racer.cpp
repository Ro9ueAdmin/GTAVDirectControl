#include "Racer.h"
#include <algorithm>
#include <inc/natives.h>
#include "Util/StringFormat.h"
#include "Util/MathExt.h"
#include "Util/Color.h"
#include "Util/UIUtils.h"
#include "Memory/VehicleExtensions.hpp"
#include "Settings.h"

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

Racer::Racer(Vehicle vehicle)
    : mVehicle(vehicle)
    , mBlip(vehicle, BlipSpriteStandard, fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
        VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)), BlipColorYellow, true)
    , mActive(gSettings.AIDefaultActive)
    , mDebugView(gSettings.AIShowDebug)
    , mPrevPointIdx(0)
    , mLapTimer(0)
    , mLapTime(0)
    , mStatusTimer(GetRand(10000, 2000))
    , mAuxTimer(GetRand(2000, 10000))
    , mStuckTimer(2000)
    , mUnstuckTimer(1000)
    , mStuckCountThreshold(3)
    , mStuckCount(0)
    , mStuckCountTimer(30000)
    , mOutsideTimer(10000) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
}

Racer::~Racer() {
    //TODO: Check if game is quitting
    if (ENTITY::DOES_ENTITY_EXIST(mVehicle)) {
        gExt.SetThrottleP(mVehicle, 0.0f);
        gExt.SetBrakeP(mVehicle, 1.0f);
        gExt.SetSteeringAngle(mVehicle, 0.0f);
        gExt.SetHandbrake(mVehicle, false);
        ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, false, true);
        ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&mVehicle);
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
    if (abs(headingAiToNpc) < deg2rad(90.0f) && aiSpeed * 1.05f >= npcSpeed) {
        float diffHeading = atan2(sin(npcHeading - aiHeading), cos(npcHeading - aiHeading));

        // Make oval shape around entity to overtake, based on dimensions.
        float distMultX = npcDim.x * 0.5f + aiDim.x;
        float distMultY = npcDim.y * 0.5f + aiDim.x;

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
            drawSphere(npcDirectionWorldCW, 0.1250f, { 255, 0, 0, 255 });
            drawSphere(npcDirectionWorldCCW, 0.1250f, { 0, 0, 255, 255 });
            drawSphere(npcDirectionWorld, 0.1250f, { 0, 255, 0, 255 });
        }

        return overtakePoints;
    }
    return {};
}

Vector3 Racer::chooseOvertakePoint(const std::vector<Point> &coords, const std::vector<Vector3> &overtakePoints, float aiLookahead, Vehicle npc, Racer::
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

    Vector3 aiNextPVel = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle) * 1.25f;
    Vector3 aiNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 aiNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(aiNextVRot.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(aiNextVRot.z), 0.0f);

    Vector3 npcNextPVel = npcPosition + ENTITY::GET_ENTITY_VELOCITY(npc);
    Vector3 npcNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(npc);
    Vector3 npcNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, ENTITY::GET_ENTITY_SPEED(npc)*-sin(npcNextVRot.z), ENTITY::GET_ENTITY_SPEED(npc)*cos(npcNextVRot.z), 0.0f);


    bool intersect = Intersect(aiPosition, (aiNextPVel + aiNextPRot) * 0.5f,overtakePoints[0], overtakePoints[1]);
    intersect |= Intersect(aiPosition, (aiNextPVel + aiNextPRot) * 0.5f, npcPosition, (npcNextPVel + npcNextPRot) * 0.5f);

    if (intersect) {
        float angleCW  = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[0] - aiPosition));
        float angleCCW = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[1] - aiPosition));

        float overtakePointAngleDist;
        float overtakePointAngleWidth;

        Vector3 overtakePointAngle;
        Vector3 overtakePointCenter;
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

        if (Distance(overtakePoint, aiPosition) < aiLookahead) {
            return overtakePoint;
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

void Racer::getControls(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents, float limitRadians,
                        float actualAngle, Racer::InputInfo& inputs, DebugInfo& dbgInfo) {

    std::vector<Vector3> overtakePoints;
    Vector3 aiDim = GetEntityDimensions(mVehicle);
    Vector3 aiNose2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, aiDim.y, 0.0f);
    float aiLookahead = ENTITY::GET_ENTITY_SPEED(mVehicle) * gSettings.AILookaheadSteerSpeedMult * 1.5f;

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
    float pitchClp = std::clamp(aiPitch, gSettings.AISteerLookAheadPitch, 0.0f);
    //showText(0.1f, 0.1f, 0.5f, fmt("Pitch: %.03f", pitchClp));
    float settingLAThrottle = gSettings.AILookaheadThrottleSpeedMult;

    float settingLABrake = gSettings.AILookaheadBrakeSpeedMult;
    float settingLASteer = gSettings.AILookaheadSteerSpeedMult;

    settingLASteer = map(pitchClp, 0.0f, gSettings.AISteerLookAheadPitch, settingLASteer, settingLABrake);

    float lookAheadSteer =  std::clamp(settingLASteer * aiSpeed, gSettings.AILookaheadSteerMinDistance, 9999.0f);

    float lookAheadThrottle = std::clamp(settingLAThrottle * aiSpeed, gSettings.AILookaheadThrottleMinDistance, 9999.0f);
    float lookAheadBrake = std::clamp(settingLABrake * aiSpeed, gSettings.AILookaheadBrakeMinDistance, 9999.0f);

    uint32_t throttleIdx;
    Vector3 nextPositionThrottle = getCoord(coords, lookAheadThrottle, actualAngle, dbgInfo.laSrcThrottle, throttleIdx);
    Vector3 nextPositionBrake = getCoord(coords, lookAheadBrake, actualAngle, dbgInfo.laSrcBrake);

    uint32_t steerIdx;
    Vector3 nextPositionSteer = getCoord(coords, lookAheadSteer, actualAngle, dbgInfo.laSrcSteer, steerIdx);
    Vector3 origPosSteer = nextPositionSteer;
    {
        Vector3 steerA = coords[(steerIdx + 0) % coords.size()].v;
        Vector3 steerB = coords[(steerIdx + 1) % coords.size()].v;

        float steerDiff = avgCenterDiff(coords, steerIdx);
        float accelDiff = avgCenterDiff(coords, throttleIdx);

        if (abs(accelDiff) > abs(steerDiff))
            steerDiff = -accelDiff;

        float cDist = lerp(mCDistPrev, map(std::clamp(steerDiff, -2.0f, 2.0f), -2.0f, 2.0f, -coords[steerIdx].w, coords[steerIdx].w), GAMEPLAY::GET_FRAME_TIME() / 0.200f);
        mCDistPrev = cDist;

        nextPositionSteer = GetPerpendicular(steerA, steerB, cDist, false);
        //drawSphere(nextPositionSteer, 0.5f, { 0 ,0 ,0 , 255 });
    }

    if (overtakePoints.size() == 3) {
        Vector3 overtakePoint = chooseOvertakePoint(coords, overtakePoints, aiLookahead, npc, dbgInfo.overtakeSource);
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

    float steerMult = gSettings.AISteerMult;

    inputs.throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    inputs.throttle *= map(distPerpThrottle, 0.0f, 1.0f, 0.5f, 1.0f);

    // Decrease throttle when starting to spin out, increase countersteer
    Vector3 nextPositionVelocity = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(rotationVelocity.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(rotationVelocity.z), 0.0f);

    float angle = GetAngleBetween(ENTITY::GET_ENTITY_VELOCITY(mVehicle), turnWorld - aiPosition);
    float csMult = std::clamp(map(angle, deg2rad(gSettings.AICountersteerIncreaseStartAngle), deg2rad(gSettings.AICountersteerIncreaseEndAngle), 1.0f, 2.0f), 0.0f, 2.0f);
    float spinoutMult = std::clamp(map(angle, deg2rad(gSettings.AIThrottleDecreaseStartAngle), deg2rad(gSettings.AIThrottleDecreaseEndAngle), 1.0f, 0.0f), 0.0f, 1.0f);

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
    if (understeering && abs(turnSteer) > gSettings.AIUndersteerHandbrakeTrigger && aiSpeed > 5.0f) {
        inputs.handbrake = true;
        inputs.throttle = 0.0f;
    }

    // start oversteer detect
    float oversteerAngle = acos(aiVelocity.y / ENTITY::GET_ENTITY_SPEED(mVehicle))* 180.0f / 3.14159265f;
    if (isnan(oversteerAngle))
        oversteerAngle = 0.0;

    if (!understeering && oversteerAngle > gSettings.AIOversteerDetectionAngle && aiVelocity.y > 5.0f) {
        inputs.throttle *= spinoutMult;
        steerMult *= csMult;
        dbgInfo.oversteerCompensateThrottle = spinoutMult < 1.0f;
        dbgInfo.oversteerCompensateSteer = steerMult > 1.0f;
    }

    // Initial brake application
    float maxBrake = 0.0f;
    // maxBrake = map(aiSpeed, distanceThrottle * gSettings.AIBrakePointDistanceThrottleMult, distanceBrake * gSettings.AIBrakePointDistanceBrakeMult, -0.3f, 3.0f);
    // TODO: Expose 0.5f and 1.5f modifiers. Right more = more aggro/late brakey // 2.0 is also ok?
    maxBrake = map(aiSpeed, 0.5f * distanceBrake * gSettings.AIBrakePointDistanceBrakeMult, 2.0f * distanceBrake * gSettings.AIBrakePointDistanceBrakeMult, 0.0f, 1.0f);

    float brakeDiffThrottleBrake = 0.0f;
    //showText(0.1f, 0.00f, 0.5f, fmt("aiHeading: %.03f", aiHeading));
    //showText(0.1f, 0.05f, 0.5f, fmt("throttleBrakeHeading: %.03f", throttleBrakeHeading));
    //showText(0.1f, 0.10f, 0.5f, fmt("diffNodeHeading: %.03f", diffNodeHeading));
    if (Distance(aiPosition, nextPositionThrottle) < lookAheadThrottle * 1.5f && abs(diffNodeHeading) - abs(actualAngle) > deg2rad(gSettings.AIBrakePointHeadingMinAngle) && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        brakeDiffThrottleBrake = map(abs(diffNodeHeading) - abs(actualAngle) - deg2rad(gSettings.AIBrakePointHeadingMinAngle), 0.0f, deg2rad(gSettings.AIBrakePointHeadingMaxAngle - gSettings.AIBrakePointHeadingMinAngle), 0.0f, 1.0f);
        brakeDiffThrottleBrake *= std::clamp(map(aiSpeed, gSettings.AIBrakePointHeadingMinSpeed, gSettings.AIBrakePointHeadingMaxSpeed, 0.0f, 1.0f), 0.0f, 1.0f);
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
        uint32_t idx = coords.size();
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

            if (overshoot > gSettings.AITrackLimitsAdjustMinOvershoot && !inside &&
                /*smallestDistanceAI < turnTrackClosest.w * 1.5f &&*/ aiSpeed > 5.0f) {
                dbgInfo.trackLimits = 1;
                inputs.throttle *= std::clamp(
                    map(overshoot,
                        gSettings.AITrackLimitsAdjustMinOvershoot, gSettings.AITrackLimitsAdjustMaxOvershoot,
                        gSettings.AITrackLimitsThrottleMultMinOvershoot, gSettings.AITrackLimitsThrottleMultMaxOvershoot)
                    , 0.0f, 1.0f
                );
                turnSteer *= map(overshoot,
                    gSettings.AITrackLimitsAdjustMinOvershoot, gSettings.AITrackLimitsAdjustMaxOvershoot,
                    gSettings.AITrackLimitsSteerMultMinOvershoot, gSettings.AITrackLimitsSteerMultMaxOvershoot);
                if (overshoot > gSettings.AITrackLimitsAdjustMaxOvershoot) {
                    float overshootBrake = map(overshoot,
                        gSettings.AITrackLimitsAdjustMaxOvershoot, gSettings.AITrackLimitsAdjustMaxOvershoot * 2.0f,
                        0.0f, 1.0f);
                    if (overshootBrake > maxBrake) {
                        maxBrake = overshootBrake;
                        inputs.throttle = 0.0f;
                        dbgInfo.trackLimits = 2;
                    }
                }
            }
            if (overshoot > gSettings.AITrackLimitsAdjustMinOvershoot && inside && aiSpeed > 5.0f) {
                dbgInfo.trackLimits = 1;
                turnSteer *=
                    std::clamp(
                        map(overshoot,
                            0.0f, gSettings.AITrackLimitsAdjustMinOvershoot,
                            1.0f, 0.0f)
                        , 0.0f, 1.0f);
            }
        }

    }

    // TODO: Clean up, consider braking _earlier_ instead of _harder_.
    // Stop yeeting off cliffs
    float aiGndZ = 0.0f;
    float laGndZ = 0.0f;
    Vector3 laPhy = nextPositionBrake;// (nextPositionVelocity + turnWorld) * 0.5f;
    bool aiGnd = GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(aiPosition.x, aiPosition.y, aiPosition.z, &aiGndZ, 0);
    bool laGnd = GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(laPhy.x, laPhy.y, laPhy.z, &laGndZ, 0);

    if (aiGnd && laGnd) {
        float drop = aiGndZ - laGndZ;
        float dropDangerMult = map(drop, gSettings.AIElevationMin, gSettings.AIElevationMax, gSettings.AIElevationDangerMin, gSettings.AIElevationDangerMax);

        if (drop > gSettings.AIElevationDropThreshold) {
            maxBrake *= dropDangerMult;

            //showText(0.0f, 0.000f, 0.5f, fmt("~r~%.03f m drop", drop));
            //showText(0.0f, 0.025f, 0.5f, fmt("~r~%.03f dropDangerMult", dropDangerMult));
            //showText(0.0f, 0.050f, 0.5f, fmt("~r~Accel: %.03f", throttle));
            //showText(0.0f, 0.075f, 0.5f, fmt("~r~Brake: %.03f", maxBrake));
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

void Racer::UpdateControl(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents) {
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

    updateLapTimers(coords);
    updateStuck(coords);

    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(mVehicle))
        VEHICLE::SET_VEHICLE_ENGINE_ON(mVehicle, true, true, true);

    float actualAngle = getSteeringAngle();
    float limitRadians = gExt.GetMaxSteeringAngle(mVehicle);
    float reduction = calculateReduction();

    DebugInfo dbgInfo{};
    InputInfo inputs = { 0.0f, 1.0f, 0.0f, false };
    getControls(coords, opponents, limitRadians, actualAngle, inputs, dbgInfo);

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

    float desiredHeading = calculateDesiredHeading(actualAngle, limitRadians, inputs.steer, reduction);

    float finalThrottle = lerp(gExt.GetThrottleP(mVehicle), inputs.throttle, GAMEPLAY::GET_FRAME_TIME() / 0.025f);
    gExt.SetThrottle(mVehicle, finalThrottle);
    gExt.SetThrottleP(mVehicle, finalThrottle);

    uint8_t numWheels = gExt.GetNumWheels(mVehicle);
    uint8_t numLockedUp = 0;
    auto curbrs = gExt.GetWheelBrakePressure(mVehicle);
    auto comprs = gExt.GetWheelCompressions(mVehicle);
    auto speeds = gExt.GetWheelRotationSpeeds(mVehicle);
    if (numWheels >= 4) {        
        for (uint8_t i = 0; i < 2; ++i) {
            if (speeds[i] == 0.0f && comprs[i] > 0.0f /*&& curbrs[i] > 0.0f*/) {
                ++numLockedUp;// = true;
            }
        }
    }

    float finalBrake = lerp(gExt.GetBrakeP(mVehicle), inputs.brake, GAMEPLAY::GET_FRAME_TIME() / 0.025f);
    if (numLockedUp == numWheels) {
        finalBrake = finalBrake * 0.25f;
        dbgInfo.abs = true;
    }
    gExt.SetBrakeP(mVehicle, finalBrake);
    if (inputs.brake > 0.15f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, false);

    gExt.SetSteeringAngle(mVehicle, lerp(actualAngle, desiredHeading, GAMEPLAY::GET_FRAME_TIME() / 0.050f));
    gExt.SetHandbrake(mVehicle, inputs.handbrake);

    if (mDebugView)
        displayDebugInfo(inputs, dbgInfo);
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

void Racer::updateLapTimers(const std::vector<Point>& points) {
    int currPointIdx = -1;

    float smallestToAi = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    for (auto i = 0ull; i < points.size(); ++i) {
        float distanceAi = Distance(aiPosition, points[i].v);
        if (distanceAi < smallestToAi) {
            smallestToAi = distanceAi;
            currPointIdx = i;
        }
    }

    if (currPointIdx < mPrevPointIdx && currPointIdx < 10 && mPrevPointIdx > points.size() - 11) {
        mLapTime = mLapTimer.Elapsed();
        mLapTimer.Reset();
    }
    mPrevPointIdx = currPointIdx;
}

void Racer::updateAux() {
    bool headlightsOn = false;
    headlightsOn |= std::find(headLightsOnWeathers.begin(), headLightsOnWeathers.end(), GAMEPLAY::GET_PREV_WEATHER_TYPE_HASH_NAME()) != headLightsOnWeathers.end();
    headlightsOn |= TIME::GET_CLOCK_HOURS() > 19 || TIME::GET_CLOCK_HOURS() < 6;
    VEHICLE::SET_VEHICLE_LIGHTS(mVehicle, headlightsOn ? 3 : 4);
}

void Racer::teleportToClosestNode(const std::vector<Point>& coords) {
    float smallestDistanceAI = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    Vector3 aiTrackClosest = aiPosition;
    for (auto& point : coords) {
        Vector3 coord = point.v;
        float distanceAI = Distance(aiPosition, coord);
        if (distanceAI < smallestDistanceAI) {
            smallestDistanceAI = distanceAI;
            aiTrackClosest = point.v;
        }
    }
    ENTITY::SET_ENTITY_COORDS(mVehicle, aiTrackClosest.x, aiTrackClosest.y, aiTrackClosest.z, 0, 0, 0, 0);
    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
}

void Racer::updateStuck(const std::vector<Point> &coords) {
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
        if (mDebugView) {
            showNotification(fmt("Attempting to unstuck %s (%s), attempt %d", 
                getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle), 
                mStuckCount + 1));
        }
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
        if (mDebugView) {
            showNotification(fmt("Teleporting %s (%s) to track (after %d attempts)",
                getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle),
                mStuckCount));
        }

        teleportToClosestNode(coords);
        mStuckCount = 0;
        mStuckCountTimer.Reset();
    }

    {
        float smallestDistanceAI = 10000.0f;
        Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
        Point aiTrackClosest = { aiPosition , 5.0f };
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
            if (mDebugView) {
                showNotification(fmt("Teleporting %s (%s) to track (outside track %d millis)", 
                    getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                    VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle),
                    mOutsideTimer.Elapsed()));
            }
            ENTITY::SET_ENTITY_COORDS(mVehicle, aiTrackClosest.v.x, aiTrackClosest.v.y, aiTrackClosest.v.z, 0, 0, 0, 0);
            VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
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
    outIndex = trackCoords.size(); // return trackCoords.end() basically

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
    int smallestToLaIdx = 0;
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

    int returnIndex = smallestToLaIdx;
    source = LookAheadSource::Normal;

    // Only consider viable nodes, to not cut the track. 
    // Significant overshoot still makes AI choose closest track node, 
    // so prevent this from happening with walls or something. 
    int nodeToConsiderMin = static_cast<int>(1.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));
    int nodeToConsiderMax = static_cast<int>(2.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));

    if ((smallestToLaIdx > smallestToAiIdx + nodeToConsiderMax || smallestToLaIdx < smallestToAiIdx - nodeToConsiderMax) &&
        smallestToAiIdx > nodeToConsiderMin && smallestToAiIdx < coords.size() - nodeToConsiderMin) {
        // Ensure track is followed continuously (no cutting off entire sections)
        returnIndex = (smallestToAiIdx + nodeToConsiderMin) % coords.size();
        source = LookAheadSource::Continuous;
    }
    else if (smallestToAiIdx >= smallestToLaIdx && smallestToAiIdx - smallestToLaIdx < coords.size() / 2) {
        // Ensure going forwards
        returnIndex = (smallestToAiIdx + (int)lookAheadDistance) % coords.size();
        source = LookAheadSource::Forward;
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
        float travelDir = atan2(target.y, target.x) - M_PI/2.0f + desiredHeading * reduction;

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
    if (gSettings.AIShowDebugText) {
        Vector3 up2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 2.0f);
        showDebugInfo3D(up2, 10.0f, {
            fmt("%s(P) %s(ESC) %s(ABS)", 
                inputs.handbrake ? "~r~" : "~m~", 
                dbgInfo.oversteerCompensateThrottle || dbgInfo.oversteerCompensateSteer ? "~o~" : "~m~",
                dbgInfo.abs ? "~r~" : "~m~"),
            fmt("%sUnder ~m~| %sOver", 
                dbgInfo.understeering ? "~r~" : "~m~", 
                dbgInfo.oversteerAngle > gSettings.AIOversteerDetectionAngle ? "~r~" : "~m~"),
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

