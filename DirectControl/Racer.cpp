#include "Racer.h"
#include <algorithm>
#include <inc/natives.h>
#include "Util/StringFormat.h"
#include "Util/MathExt.h"
#include "Util/Color.h"
#include "Util/UIUtils.h"
#include "Memory/VehicleExtensions.hpp"
#include "Settings.h"

std::vector<Hash> headLightsOnWeathers = {
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

Racer::Racer(Vehicle vehicle) : mVehicle(vehicle)
                              , mActive(gSettings.AIDefaultActive)
                              , mDebugView(gSettings.AIShowDebug)
                              , mPrevPointIdx(0)
                              , mLapTimer(0)
                              , mLapTime(0)
                              , mAuxPeriod(GetRand(2000, 10000))
                              , mAuxPrevTick(GetTickCount() + rand() % mAuxPeriod)
                              , mStuckTimeThreshold(2000)
                              , mStuckStarted(0)
                              , mIsStuck(false)
                              , mStuckCountThreshold(3)
                              , mStuckCountTime(30000)
                              , mStuckCountStarted(0)
                              , mStuckCount(0) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
    mBlip = std::make_unique<BlipX>(mVehicle);
    mBlip->SetSprite(BlipSpritePersonalVehicleCar);
    mBlip->SetName(fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                       VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip->SetColor(BlipColorYellow);
}

Racer::Racer(Racer &&other) noexcept : mVehicle(other.mVehicle)
                                     , mActive(other.mActive)
                                     , mDebugView(other.mDebugView)
                                     , mPrevPointIdx(other.mPrevPointIdx)
                                     , mLapTimer(other.mLapTimer)
                                     , mLapTime(other.mLapTime)
                                     , mAuxPeriod(other.mAuxPeriod)
                                     , mAuxPrevTick(other.mAuxPrevTick)
                                     , mStuckTimeThreshold(other.mStuckTimeThreshold)
                                     , mStuckStarted(other.mStuckStarted)
                                     , mIsStuck(other.mIsStuck)
                                     , mStuckCountThreshold(other.mStuckCountThreshold)
                                     , mStuckCountTime(other.mStuckCountTime)
                                     , mStuckCountStarted(other.mStuckCountStarted)
                                     , mStuckCount(other.mStuckCount) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
    mBlip = std::make_unique<BlipX>(mVehicle);
    mBlip->SetSprite(BlipSpritePersonalVehicleCar);
    mBlip->SetName(fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                       VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip->SetColor(BlipColorYellow);
    other.mVehicle = 0;
}

Racer & Racer::operator=(Racer &&other) noexcept {
    if (this != &other) {
        mVehicle = other.mVehicle;
        mBlip = std::make_unique<BlipX>(mVehicle);
        mBlip->SetSprite(BlipSpritePersonalVehicleCar);
        mBlip->SetName(fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)), VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
        mBlip->SetColor(BlipColorYellow);
        other.mVehicle = 0;
    }
    return *this;
}

Racer::~Racer() {
    if (ENTITY::DOES_ENTITY_EXIST(mVehicle)) {
        gExt.SetThrottleP(mVehicle, 0.0f);
        gExt.SetBrakeP(mVehicle, 1.0f);
        gExt.SetSteeringAngle(mVehicle, 0.0f);
        gExt.SetHandbrake(mVehicle, false);
        ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, false, true);
        ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&mVehicle);
    }
    if (mBlip) {
        mBlip->Delete();
        mBlip.reset(nullptr);
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
    std::vector<Vector3> overtakePoints(2);

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

        overtakePoints[0] = npcDirectionWorldCW;
        overtakePoints[1] = npcDirectionWorldCCW;

        if (mDebugView) {
            drawSphere(npcDirectionWorldCW, 0.1250f, { 255, 0, 0, 255 });
            drawSphere(npcDirectionWorldCCW, 0.1250f, { 0, 0, 255, 255 });
        }

        return overtakePoints;
    }
    return {};
}

Vector3 Racer::chooseOvertakePoint(const std::vector<Point> &coords, const std::vector<Vector3> &overtakePoints, float aiLookahead, Vehicle npc, std::string &overtakeReason) {
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
            overtakeReason = "Both: All clear!";
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
                overtakePointAngleDist > overtakePointAngleWidth) {
                overtakePoint = overtakePointCenter;
                overtakeReason = "Track center";
            }
            else {
                overtakePoint = overtakePointAngle;
                overtakeReason = "Angle";
            }
        }

        if (Distance(overtakePoint, aiPosition) < aiLookahead) {
            return overtakePoint;
        }

    }

    return {};
}

void Racer::getControls(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents, float limitRadians,
                        float actualAngle, bool &handbrake, float &throttle, float &brake, float &steer) {
    handbrake = false;
    throttle = 0.0f;
    brake = 1.0f;
    steer = 0.0f;

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

    bool dbgSpinThrottle = false;
    bool dbgSpinCountersteer = false;
    bool dbgBrakeForAngle = false;
    bool dbgBrakeForHeading = false;
    bool dbgTrackLimits = false;
    std::string dbgThrottleSrc;
    std::string dbgBrakeSrc;
    std::string dbgSteerSrc;
    std::string dbgOvertakeSrc = "N/A";

    Vector3 aiVelocity = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 aiForward = aiPosition + ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle);
    float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

    float lookAheadThrottle = std::clamp(gSettings.AILookaheadThrottleSpeedMult * aiSpeed, gSettings.AILookaheadThrottleMinDistance, 9999.0f);
    float lookAheadSteer =  std::clamp(gSettings.AILookaheadSteerSpeedMult * aiSpeed, gSettings.AILookaheadSteerMinDistance, 9999.0f);
    float lookAheadBrake = std::clamp(gSettings.AILookaheadBrakeSpeedMult * aiSpeed, gSettings.AILookaheadBrakeMinDistance, 9999.0f);

    Vector3 nextPositionThrottle = getCoord(coords, lookAheadThrottle, actualAngle, dbgThrottleSrc);
    Vector3 nextPositionSteer = getCoord(coords, lookAheadSteer, actualAngle, dbgSteerSrc);
    Vector3 nextPositionBrake = getCoord(coords, lookAheadBrake, actualAngle, dbgBrakeSrc);

    if (overtakePoints.size() == 2) {
        Vector3 overtakePoint = chooseOvertakePoint(coords, overtakePoints, aiLookahead, npc, dbgOvertakeSrc);
        if (Length(overtakePoint) > 0.0f)
            nextPositionSteer = overtakePoint;
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

    throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    throttle *= map(distPerpThrottle, 0.0f, 1.0f, 0.5f, 1.0f);

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
    if (understeering && abs(turnSteer * steerMult) > 1.0f && aiSpeed > 10.0f) {
        handbrake = true;
    }

    // start oversteer detect
    float angleOverSteer = acos(aiVelocity.y / ENTITY::GET_ENTITY_SPEED(mVehicle))* 180.0f / 3.14159265f;
    if (isnan(angleOverSteer))
        angleOverSteer = 0.0;

    if (!understeering && angleOverSteer > gSettings.AIOversteerDetectionAngle && aiVelocity.y > 5.0f) {
        throttle *= spinoutMult;
        steerMult *= csMult;
        dbgSpinThrottle = spinoutMult < 1.0f;
        dbgSpinCountersteer = steerMult > 1.0f;
    }

    float maxBrake = map(aiSpeed, distanceThrottle * gSettings.AIBrakePointDistanceThrottleMult, distanceBrake * gSettings.AIBrakePointDistanceBrakeMult, -0.3f, 3.0f);

    float brakeDiffThrottleBrake = 0.0f;
    if (Distance(aiPosition, nextPositionThrottle) < lookAheadThrottle * 1.5f && abs(diffNodeHeading) - abs(actualAngle) > deg2rad(gSettings.AIBrakePointHeadingMinAngle) && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        brakeDiffThrottleBrake = map(abs(diffNodeHeading) - abs(actualAngle) - deg2rad(gSettings.AIBrakePointHeadingMinAngle), 0.0f, deg2rad(gSettings.AIBrakePointHeadingMaxAngle - gSettings.AIBrakePointHeadingMinAngle), 0.0f, 1.0f);
        brakeDiffThrottleBrake *= std::clamp(map(aiSpeed, gSettings.AIBrakePointHeadingMinSpeed, gSettings.AIBrakePointHeadingMaxSpeed, 0.0f, 1.0f), 0.0f, 1.0f);
        if (brakeDiffThrottleBrake > maxBrake) {
            maxBrake = brakeDiffThrottleBrake;
            dbgBrakeForHeading = true;
        }
    }

    if (abs(turnBrake) > limitRadians && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        float brakeTurn = map(distPerpBrake, 0.0f, 1.0f, 1.0f, 0.0f);
        if (brakeTurn > maxBrake) {
            maxBrake = brakeTurn;
            dbgBrakeForAngle = true;
        }
    }

    // Track limits
    {
        float smallestDistanceVelocity = 10000.0f;
        float smallestDistanceRotation = 10000.0f;
        float smallestDistanceAI = 10000.0f;
        Point turnTrackClosest{};
        for (auto& point : coords) {
            Vector3 coord = point.v;
            float distanceVel = Distance(nextPositionVelocity, coord);
            float distanceRot = Distance(turnWorld, coord);
            float distanceAI = Distance(aiPosition, coord);
            if (distanceVel < smallestDistanceVelocity) {
                smallestDistanceVelocity = distanceVel;
            }
            if (distanceRot < smallestDistanceRotation) {
                smallestDistanceRotation = distanceRot;
                turnTrackClosest = point;
            }
            if (distanceAI < smallestDistanceAI) {
                smallestDistanceAI = distanceAI;
            }
        }

        float overshoot = (smallestDistanceVelocity + smallestDistanceRotation) / 2.0f - turnTrackClosest.w;

        if (overshoot > gSettings.AITrackLimitsAdjustMinOvershoot &&
            smallestDistanceAI < turnTrackClosest.w * 1.5f && aiSpeed > 5.0f) {
            dbgTrackLimits = true;
            throttle *= std::clamp(
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
                    throttle = 0.0f;
                }
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

    throttle = std::clamp(throttle, 0.0f, 1.0f);
    brake = std::clamp(maxBrake, 0.0f, 1.0f);
    steer = std::clamp(turnSteer * steerMult, -1.0f, 1.0f);
    
    if (brake > 0.7f)
        throttle = 0.0f;

    if (throttle > 0.95f)
        throttle = 1.0f;

    if (mDebugView) {
        Color red{ 255, 0, 0, 255 };
        Color green{ 0, 255, 0, 255 };
        Color blue{ 0, 0, 255, 255 };
        Color white{ 255, 255, 255, 255 };
        Color yellow{ 255, 255, 0, 255 };

        drawLine(aiPosition, nextPositionThrottle, green);
        drawSphere(nextPositionThrottle, 0.25f, green);
        drawLine(aiPosition, nextPositionSteer, blue);
        drawSphere(nextPositionSteer, 0.25f, blue);
        drawLine(aiPosition, nextPositionBrake, red);
        drawSphere(nextPositionBrake, 0.25f, red);

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
            std::clamp(static_cast<int>(map(brake, 0.0f, 1.0f, 0.0f, 255.0f)), 127, 255),
            std::clamp(static_cast<int>(map(brake, 0.0f, 1.0f, 255.0f, 0.0f)), 127, 255),
            0,
            255
        };

        if (mIsStuck) {
            drawSphere(up, 0.5f, red);
        }
        else if (handbrake) {
            drawSphere(up, 0.5f, yellow);
        }
        else {
            drawChevron(up, dir, rot, 1.0f, throttle, c);
        }

        drawLine(aiPosition, (nextPositionVelocity + turnWorld) * 0.5f, white);
        drawSphere((nextPositionVelocity + turnWorld) * 0.5f, 0.25f, white);


        int currPointIdx = -1;

        float smallestToAi = 10000.0f;
        for (auto i = 0; i < coords.size(); ++i) {
            float distanceAi = Distance(aiPosition, coords[i].v);
            if (distanceAi < smallestToAi) {
                smallestToAi = distanceAi;
                currPointIdx = i;
            }
        }


        if (currPointIdx < mPrevPointIdx && currPointIdx < 10 && mPrevPointIdx > coords.size() - 11) {
            mLapTime = mLapTimer.Elapsed();
            mLapTimer.Reset();
        }
        mPrevPointIdx = currPointIdx;
        int64_t currentLapTime = mLapTimer.Elapsed();
        std::string previousLapTimeFmt = fmt("%02d:%02d.%03d", mLapTime / 60000, (mLapTime / 1000) % 60, mLapTime % 1000);
        std::string liveLapTimeFmt = fmt("%02d:%02d.%03d", currentLapTime / 60000, (currentLapTime / 1000) % 60, currentLapTime % 1000);

        // Debug text
        if (gSettings.AIShowDebugText) {
            Vector3 up2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 2.0f);
            showDebugInfo3D(up2, 10.0f, {
                fmt("%sSpinThrottle--", dbgSpinThrottle ? "~r~" : "~w~"),
                fmt("%sSpinSteer++", dbgSpinCountersteer ? "~r~" : "~w~"),
                fmt("%sBrake4Angle", dbgBrakeForAngle ? "~r~" : "~w~"),
                fmt("%sBrake4Heading", dbgBrakeForHeading ? "~r~" : "~w~"),
                fmt("LAThrottle: %s", dbgThrottleSrc.c_str()),
                fmt("LABrake: %s", dbgBrakeSrc.c_str()),
                fmt("LASteer: %s", dbgSteerSrc.c_str()),
                fmt("%sTrackLimits", dbgTrackLimits ? "~r~" : "~w~"),
                fmt("OT: %s", dbgOvertakeSrc.c_str()),
                fmt("Lap: %s", previousLapTimeFmt.c_str()),
                fmt("Live: %s", liveLapTimeFmt.c_str()),
            });
        }
        else {
            Vector3 up2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 2.0f);
            showDebugInfo3D(up2, 10.0f, {
                fmt("Lap: %s", previousLapTimeFmt.c_str()),
                fmt("Live: %s", liveLapTimeFmt.c_str()),
                });
        }
    }
}

void Racer::UpdateControl(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents) {
    if (!VEHICLE::IS_VEHICLE_DRIVEABLE(mVehicle, 0) || ENTITY::IS_ENTITY_DEAD(mVehicle))
        return;

    if (GetTickCount() > mAuxPrevTick + mAuxPeriod) {
        mAuxPeriod = GetRand(2000, 10000);
        mAuxPrevTick = GetTickCount();
        updateAux();
    }

    updateStuck(coords);

    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(mVehicle))
        VEHICLE::SET_VEHICLE_ENGINE_ON(mVehicle, true, true, true);

    float actualAngle = getSteeringAngle();
    float limitRadians = gExt.GetMaxSteeringAngle(mVehicle);
    float reduction = calculateReduction();

    bool handbrake = false;
    float throttle = 0.0f;
    float brake = 0.0f;
    float steer = 0.0f;

    getControls(coords, opponents, limitRadians, actualAngle, handbrake, throttle, brake, steer);

    if (mIsStuck) {
        throttle = -0.4f;
        brake = 0.0f;
        steer = 0.0f;
        handbrake = false;
    }

    float desiredHeading = calculateDesiredHeading(actualAngle, limitRadians, steer, reduction);

    gExt.SetThrottleP(mVehicle, throttle);
    gExt.SetBrakeP(mVehicle, brake);
    if (brake > 0.15f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, false);

    gExt.SetSteeringAngle(mVehicle, lerp(actualAngle, desiredHeading, (1.0f / 0.05f) * GAMEPLAY::GET_FRAME_TIME()));
    gExt.SetHandbrake(mVehicle, handbrake);
}

void Racer::updateAux() {
    bool headlightsOn = false;
    headlightsOn |= std::find(headLightsOnWeathers.begin(), headLightsOnWeathers.end(), GAMEPLAY::GET_PREV_WEATHER_TYPE_HASH_NAME()) != headLightsOnWeathers.end();
    headlightsOn |= TIME::GET_CLOCK_HOURS() > 19 || TIME::GET_CLOCK_HOURS() < 6;
    VEHICLE::SET_VEHICLE_LIGHTS(mVehicle, headlightsOn ? 3 : 4);
}

void Racer::resetStuckState(bool resetStuckCount) {
    mIsStuck = false;
    mStuckStarted = 0;
    if (resetStuckCount) {
        mStuckCount = 0;
        mStuckCountStarted = 0;
    }
}

void Racer::updateStuck(const std::vector<Point> &coords) {
    if (coords.size() < 2 || !mActive || VEHICLE::GET_PED_IN_VEHICLE_SEAT(mVehicle, -1) == PLAYER::PLAYER_PED_ID()) {
        resetStuckState(true);
        return;
    }

    if (mStuckCountStarted != 0 && GetTickCount() > mStuckCountStarted + mStuckCountTime) {
        if (mStuckCount > mStuckCountThreshold) {
            if (mDebugView) {
                showNotification(fmt("Teleporting stuck %s (%s)", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                    VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
            }

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
            resetStuckState(true);
        }
        else {
            mStuckCount = 0;
            mStuckCountStarted = 0;
        }
    }

    if (!mIsStuck) {
        if (mStuckStarted == 0) {
            if (ENTITY::GET_ENTITY_SPEED(mVehicle) < 0.5f) {
                mStuckStarted = GetTickCount();
            }
        }
        if (GetTickCount() > mStuckStarted + mStuckTimeThreshold && GetTickCount() < mStuckStarted + 2 * mStuckTimeThreshold) {
            if (ENTITY::GET_ENTITY_SPEED(mVehicle) < 0.5f) {
                mIsStuck = true;
                if (!VEHICLE::IS_VEHICLE_ON_ALL_WHEELS(mVehicle)) {
                    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
                }
                if (mDebugView) {
                    showNotification(fmt("Attempting to unstuck %s (%s), attempt %d", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                        VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle), mStuckCount + 1));
                }
                if (mStuckCount == 0) {
                    mStuckCountStarted = GetTickCount();
                }
                mStuckCount++;
            }
            else {
                resetStuckState(false);
            }
        }
    }
    else {
        if (GetTickCount() > mStuckStarted + 2 * mStuckTimeThreshold) {
            resetStuckState(false);
        }
    }
    
}

Vehicle Racer::GetVehicle() {
    return mVehicle;
}

Vector3 Racer::getCoord(const std::vector<Point> &coords,
                        float lookAheadDistance, float actualAngle, std::string &source) {
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
    source = "normal";

    // Only consider viable nodes, to not cut the track. 
    // Significant overshoot still makes AI choose closest track node, 
    // so prevent this from happening with walls or something. 
    int nodeToConsiderMin = static_cast<int>(1.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));
    int nodeToConsiderMax = static_cast<int>(2.0f * lookAheadDistance / Distance(coords[smallestToAiIdx].v, coords[(smallestToAiIdx + 1) % coords.size()].v));

    if ((smallestToLaIdx > smallestToAiIdx + nodeToConsiderMax || smallestToLaIdx < smallestToAiIdx - nodeToConsiderMax) &&
        smallestToAiIdx > nodeToConsiderMin && smallestToAiIdx < coords.size() - nodeToConsiderMin) {
        // Ensure track is followed continuously (no cutting off entire sections)
        returnIndex = (smallestToAiIdx + nodeToConsiderMin) % coords.size();
        source = "continuous";
    }
    else if (smallestToAiIdx >= smallestToLaIdx && smallestToAiIdx - smallestToLaIdx < coords.size() / 2) {
        // Ensure going forwards
        returnIndex = (smallestToAiIdx + (int)lookAheadDistance) % coords.size();
        source = "forwards";
    }

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
        mult = (0.15f + (powf((1.0f / 1.13f), (abs(motion.y) - 7.2f))));
        if (mult != 0) { mult = floorf(mult * 1000) / 1000; }
        if (mult > 1) { mult = 1; }
    }
    mult = (1 + (mult - 1) * 1.0f);
    return mult;
}

float Racer::calculateDesiredHeading(float steeringAngle, float steeringMax, float desiredHeading,
                                     float reduction) {
    float correction = desiredHeading * reduction;

    if (abs(ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y) > 3.0f) {
        Vector3 velocityWorld = ENTITY::GET_ENTITY_VELOCITY(mVehicle);
        Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
        Vector3 travelWorld = velocityWorld + positionWorld;

        float steeringAngleRelX = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y * -sin(steeringAngle);
        float steeringAngleRelY = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y * cos(steeringAngle);
        Vector3 steeringWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(
            mVehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);

        Vector3 travelNorm = Normalize(travelWorld - positionWorld);
        Vector3 steerNorm = Normalize(steeringWorld - positionWorld);
        float travelDir = atan2(travelNorm.y, travelNorm.x) + desiredHeading * reduction;
        float steerDir = atan2(steerNorm.y, steerNorm.x);

        correction = 2.0f * atan2(sin(travelDir - steerDir), cos(travelDir - steerDir));
    }
    if (correction > steeringMax)
        correction = steeringMax;
    if (correction < -steeringMax)
        correction = -steeringMax;

    return correction;
}
