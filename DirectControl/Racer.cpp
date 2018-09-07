#include "Racer.h"
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

Racer::Racer(Vehicle vehicle) :
    mStuckThreshold(2000),
    mVehicle(vehicle),
    mActive(gSettings.AIDefaultActive),
    mDebugView(gSettings.AIShowDebug),
    mAuxPeriod(GetRand(2000, 10000)),
    mAuxPrevTick(GetTickCount() + rand() % mAuxPeriod),
    mIsStuck(false),
    mStuckStarted(0) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
    mBlip = std::make_unique<BlipX>(mVehicle);
    mBlip->SetSprite(BlipSpritePersonalVehicleCar);
    mBlip->SetName(fmt("AI %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                       VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip->SetColor(BlipColorYellow);
}

Racer::Racer(Racer &&other) noexcept :
    mStuckThreshold(other.mStuckThreshold),
    mVehicle(other.mVehicle),
    mActive(other.mActive),
    mDebugView(other.mDebugView),
    mAuxPeriod(other.mAuxPeriod),
    mAuxPrevTick(other.mAuxPrevTick),
    mIsStuck(other.mIsStuck),
    mStuckStarted(other.mStuckStarted) {
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
    //showText(0.0f, 0.000f, 0.5f, fmt("npc: %d %s", npc, getGxtName(ENTITY::GET_ENTITY_MODEL(npc)).c_str()));
    //showText(0.0f, 0.025f, 0.5f, fmt("npcD (%.03f, %.03f, %.03f)", npcDirection.x, npcDirection.y, npcDirection.z));
    //showText(0.0f, 0.050f, 0.5f, fmt("headingAiToNpc = %.03f deg", rad2deg(headingAiToNpc)));
    //showText(0.0f, 0.075f, 0.5f, fmt("%saiSpeed = %.03f, npcSpeed = %.03f", aiSpeed * 1.25f >= npcSpeed ? "~g~" : "~w~", aiSpeed, npcSpeed));

    if (abs(headingAiToNpc) < deg2rad(90.0f) && aiSpeed >= npcSpeed) {

        // shit now rotates around the heading difference, not the relative angle to the npc.
        // this keeps the "paralellity" regardless of distance! so smart ikt!
        float diffHeading = atan2(sin(npcHeading - aiHeading), cos(npcHeading - aiHeading));
        //float perpRatio = 1.0f - abs((abs(diffHeading) - 1.5708f) / 1.5708f);

        // Keep oval shape around entity to overtake, with the long direction being the direction you're heading
        float distMultX = npcDim.x * 0.5f + aiDim.x;
        float distMultY = npcDim.y * 0.5f + aiDim.x;

        // When the "additional offset" is added depends on the heading.
        // e.g. if parallel, just add in y. if perp, just add in x.
        // this makes the carrot travel paralelly with the AI
        float mulX = -(cos(diffHeading + (deg2rad(180.0f))));
        float mulY = -(sin(diffHeading + (deg2rad(180.0f))));

        // This thing is how far from perpendicular we are. Similar to mulX/mulY?
        // diffHeading is from -180 to 180. Make abs(it) pivot around 0, so -90 to 90.
        // then (90 - abs(that))/90 to get ratio. 0 for straight? 1 for perp?
        // could prolly be simpler
        float diffHeadingMultiplier = (deg2rad(90.0f) - abs(abs(diffHeading) - deg2rad(90.0f)))/(deg2rad(90.0f));

        float pMult = diffHeadingMultiplier;
        float nMult = (1.0f - diffHeadingMultiplier);

        float xOff = npcToAiVector.x+ (aiDim.y * 1.5f * mulY);
        float yOff = npcToAiVector.y+ (aiDim.y * 1.5f * mulX);

        float constrainX = npcDim.x + aiDim.y;
        float constrainY = npcDim.y + aiDim.y;

        constrainX *= pMult;
        constrainY *= nMult;

        float xOffset = constrain(xOff * pMult, -constrainX, constrainX);
        float yOffset = constrain(yOff * nMult, -constrainY, constrainY);

        Vector3 npcOffset = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, 0.0f, 0.0f, 0.0f);
        Vector3 npcDirectionWorldCW = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, distMultX * sin(diffHeading + deg2rad(90.0f)) + xOffset, distMultY * cos(diffHeading + deg2rad(90.0f)) + yOffset, 0.0f);
        Vector3 npcDirectionWorldCCW = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, distMultX * sin(diffHeading - deg2rad(90.0f)) + xOffset, distMultY * cos(diffHeading - deg2rad(90.0f)) + yOffset, 0.0f);


        overtakePoints[0] = npcDirectionWorldCW;
        overtakePoints[1] = npcDirectionWorldCCW;

        if (mDebugView) {
            drawSphere(npcPosition, 0.25f, { 0, 0, 0, 255 });
            drawLine(npcOffset, npcDirectionWorldCW, { 255, 0, 0, 255 });
            drawSphere(npcDirectionWorldCW, 0.1250f, { 255, 0, 0, 255 });
            drawLine(npcOffset, npcDirectionWorldCCW, { 0, 0, 255, 255 });
            drawSphere(npcDirectionWorldCCW, 0.1250f, { 0, 0, 255, 255 });
            //drawLine(aiPosition, npcSteerPos, { 255, 0, 255, 255 });
            //drawSphere(npcSteerPos, 0.1250f, { 255, 0, 255, 255 });
        }

        return overtakePoints;
    }
    return {};
}

bool ccw(Vector3 A, Vector3 B, Vector3 C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
}

// Return true if line segments AB and CD intersect
bool intersect(Vector3 A, Vector3 B, Vector3 C,Vector3 D) {
    return ccw(A, C, D) != ccw(B, C, D) && ccw(A, B, C) != ccw(A, B, D);
}

void Racer::getControls(const std::vector<Point> &coords, const std::vector<Vehicle> &opponents, float limitRadians,
                        float actualAngle, bool &handbrake, float &throttle, float &brake, float &steer) {
    handbrake = false;
    throttle = 0.0f;
    brake = 1.0f;
    steer = 0.0f;

    // 1. Calculate overtaking points.
    // 2. Choose whether to go left or right.
    // 3. Choose whether to follow ideal line or overtaking "line"
    std::vector<Vector3> overtakePoints;

    Vector3 aiDim = GetEntityDimensions(mVehicle);
    Vector3 aiNose2 = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, aiDim.y, 0.0f);
    float aiLookahead = ENTITY::GET_ENTITY_SPEED(mVehicle) * gSettings.AILookaheadSteerSpeedMult * 1.5f;

    float searchDistance = aiLookahead > 30.0f ? aiLookahead : 30.0f;
    // Get NPC closest to front of vehicle
    Vehicle npc = findClosestVehicle(opponents, aiNose2, searchDistance);

    // Get a coordinate perpendicular to the NPC to overtake/avoid.
    if (npc) {
        overtakePoints = findOvertakingPoints(npc);
    }
    

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

    Vector3 aiVelocity = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true);
    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0, 5.0f, 0.0f);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);

    float lookAheadThrottle = constrain(gSettings.AILookaheadThrottleSpeedMult * ENTITY::GET_ENTITY_SPEED(mVehicle), gSettings.AILookaheadThrottleMinDistance, 9999.0f);
    float lookAheadSteer = constrain(gSettings.AILookaheadSteerSpeedMult * ENTITY::GET_ENTITY_SPEED(mVehicle), gSettings.AILookaheadSteerMinDistance, 9999.0f);
    float lookAheadBrake = constrain(gSettings.AILookaheadBrakeSpeedMult * ENTITY::GET_ENTITY_SPEED(mVehicle), gSettings.AILookaheadBrakeMinDistance, 9999.0f);

    Vector3 nextPositionThrottle = getCoord(coords, lookAheadThrottle, actualAngle, dbgThrottleSrc);
    Vector3 nextPositionSteer = getCoord(coords, lookAheadSteer, actualAngle, dbgSteerSrc);

    std::string overtakeReason = "N/A";

    if (overtakePoints.size() == 2) {
        Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, 1);

        float aiTrackDist = 10000.0f;
        float npcTrackDist = 10000.0f;
        float overtakeTrackDist0 = 10000.0f;
        float overtakeTrackDist1 = 10000.0f;

        Point smallestToAi{};
        Point smallestToOt0{};
        Point smallestToOt1{};
        Point smallestToNpc{};

        for (auto& point : coords) {
            Vector3 coord = point.v;
            float distanceAi = Distance(aiPosition, coord);
            float distanceOt0 = Distance(overtakePoints[0], coord);
            float distanceOt1 = Distance(overtakePoints[1], coord);
            float distanceNpc = Distance(npcPosition, coord);

            if (distanceAi < aiTrackDist) {
                aiTrackDist = distanceAi;
                smallestToAi = point;
            }
            if (distanceOt0 < overtakeTrackDist0) {
                overtakeTrackDist0 = distanceOt0;
                smallestToOt0 = point;
            }
            if (distanceOt1 < overtakeTrackDist1) {
                overtakeTrackDist1 = distanceOt1;
                smallestToOt1 = point;
            }
            if (distanceNpc < npcTrackDist) {
                npcTrackDist = distanceNpc;
                smallestToNpc = point;
            }
        }

        Vector3 aiNextPVel = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle) * 1.25f;
        Vector3 aiNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
        Vector3 aiNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(aiNextVRot.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(aiNextVRot.z), 0.0f);

        Vector3 npcNextPVel = npcPosition + ENTITY::GET_ENTITY_VELOCITY(npc);
        Vector3 npcNextVRot = ENTITY::GET_ENTITY_ROTATION_VELOCITY(npc);
        Vector3 npcNextPRot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, ENTITY::GET_ENTITY_SPEED(npc)*-sin(npcNextVRot.z), ENTITY::GET_ENTITY_SPEED(npc)*cos(npcNextVRot.z), 0.0f);


        bool gonIntersect = intersect(aiPosition, (aiNextPVel + aiNextPRot) * 0.5f,overtakePoints[0], overtakePoints[1]);
        gonIntersect |= intersect(aiPosition, (aiNextPVel + aiNextPRot) * 0.5f, npcPosition, (npcNextPVel + npcNextPRot) * 0.5f);

        if (gonIntersect) {
            drawSphere(ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, 1.0f), 0.25f, { 255, 0, 255, 255 });

            float angleCW  = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[0] - aiPosition));
            float angleCCW = GetAngleBetween(Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle)), Normalize(overtakePoints[1] - aiPosition));

            if (mDebugView) {
                showText(0.0f, 0.0f, 0.5f, fmt("Angle CW: %.03f", rad2deg(angleCW)));
                showText(0.0f, 0.025f, 0.5f, fmt("AngleCCW: %.03f", rad2deg(angleCCW)));
            }

            Vector3 overtakePointAngle;
            Vector3 overtakePointCenter;
            Vector3 overtakePoint;
            if (angleCW < angleCCW) {
                overtakePointAngle = overtakePoints[0];
            }
            else {
                overtakePointAngle = overtakePoints[1];
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

                drawLine(offWLeftRear, offWLeftFront, { 255, 0, 0, 255 });
                drawLine(offWLeftFront, offWRightFront, { 255, 0, 0, 255 });
                drawLine(offWRightFront, offWRightRear, { 255, 0, 0, 255 });
                drawLine(offWRightRear, offWLeftRear, { 255, 0, 0, 255 });

                // Choose closest to track center when angle is less desirable (crosses the vehicle to overtake)
                if (intersect(aiPosition, overtakePointAngle, offWLeftRear, offWRightFront) ||
                    intersect(aiPosition, overtakePointAngle, offWLeftFront, offWRightRear)) {
                    overtakePoint = overtakePointCenter;
                    overtakeReason = "Track center";
                }
                else {
                    overtakePoint = overtakePointAngle;
                    overtakeReason = "Angle";

                }
            }

            if (Distance(overtakePoint, aiPosition) < aiLookahead) {
                nextPositionSteer = overtakePoint;
            }

        }
    }

    Vector3 nextPositionBrake = getCoord(coords, lookAheadBrake, actualAngle, dbgBrakeSrc);

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

    float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

    throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    throttle *= map(distPerpThrottle, 0.0f, 1.0f, 0.5f, 1.0f);

    // Decrease throttle when starting to spin out, increase countersteer
    Vector3 nextPositionVelocity = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(rotationVelocity.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(rotationVelocity.z), 0.0f);

    float angle = GetAngleBetween(ENTITY::GET_ENTITY_VELOCITY(mVehicle), turnWorld - aiPosition);
    float csMult = constrain(map(angle, deg2rad(gSettings.AICountersteerIncreaseStartAngle), deg2rad(gSettings.AICountersteerIncreaseEndAngle), 1.0f, 2.0f), 0.0f, 2.0f);
    float spinoutMult = constrain(map(angle, deg2rad(gSettings.AIThrottleDecreaseStartAngle), deg2rad(gSettings.AIThrottleDecreaseEndAngle), 1.0f, 0.0f), 0.0f, 1.0f);

    // start oversteer detect
    float angleOverSteer = acos(aiVelocity.y / ENTITY::GET_ENTITY_SPEED(mVehicle))* 180.0f / 3.14159265f;
    if (isnan(angleOverSteer))
        angleOverSteer = 0.0;

    if (angleOverSteer > gSettings.AIOversteerDetectionAngle && aiVelocity.y > 5.0f) {
        throttle *= spinoutMult;
        steerMult *= csMult;
        dbgSpinThrottle = spinoutMult < 1.0f;
        dbgSpinCountersteer = steerMult > 1.0f;
    }

    if (overtakePoints.size() == 0) {
        handbrake = abs(turnSteer) > limitRadians * 2.0f && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 12.0f;
    }

    float maxBrake = map(aiSpeed, distanceThrottle * gSettings.AIBrakePointDistanceThrottleMult, distanceBrake * gSettings.AIBrakePointDistanceBrakeMult, -0.3f, 3.0f);

    float brakeDiffThrottleBrake = 0.0f;
    if (Distance(aiPosition, nextPositionThrottle) < lookAheadThrottle * 1.5f && abs(diffNodeHeading) - abs(actualAngle) > deg2rad(gSettings.AIBrakePointHeadingMinAngle) && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        brakeDiffThrottleBrake = map(abs(diffNodeHeading) - abs(actualAngle) - deg2rad(gSettings.AIBrakePointHeadingMinAngle), 0.0f, deg2rad(gSettings.AIBrakePointHeadingMaxAngle - gSettings.AIBrakePointHeadingMinAngle), 0.0f, 1.0f);
        brakeDiffThrottleBrake *= constrain(map(aiSpeed, gSettings.AIBrakePointHeadingMinSpeed, gSettings.AIBrakePointHeadingMaxSpeed, 0.0f, 1.0f), 0.0f, 1.0f);
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


    {
        float smallestDistanceVelocity = 10000.0f;
        float smallestDistanceRotation = 10000.0f;
        float smallestDistanceAI = 10000.0f;
        Point aiTrackClosest{};
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
                aiTrackClosest = point;
            }

        }

        if ((smallestDistanceVelocity + smallestDistanceRotation) / 2.0f > turnTrackClosest.w && 
            smallestDistanceAI < turnTrackClosest.w * 1.5f && aiSpeed > 12.0f) {
            dbgTrackLimits = true;
            float overshoot = (smallestDistanceVelocity + smallestDistanceRotation) / 2.0f - turnTrackClosest.w;
            throttle *= constrain(map(overshoot, 0.0f, 1.0f, 1.0f, 0.0f), 0.0f, 1.0f);
            turnSteer *= map(overshoot, 0.0f, 1.0f, 1.0f, 2.0f);
        }
    }


    throttle = constrain(throttle, 0.0f, 1.0f);
    brake = constrain(maxBrake, 0.0f, 1.0f);
    steer = constrain(turnSteer * steerMult, -1.0f, 1.0f);
    
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
            constrain(static_cast<int>(map(brake, 0.0f, 1.0f, 127.0f, 255.0f)), 127, 255),
            constrain(static_cast<int>(map(brake, 0.0f, 1.0f, 255.0f, 127.0f)), 127, 255),
            0,
            255
        };

        if (mIsStuck) {
            drawSphere(up, 0.5f, red);
        }
        else {
            drawChevron(up, dir, rot, 1.0f, throttle, c);
        }

        // spinout ratio
        //drawLine(aiPosition, nextPositionVelocity, white);
        //drawSphere(nextPositionVelocity, 0.25f, white);
        //drawLine(aiPosition, turnWorld, yellow);
        //drawSphere(turnWorld, 0.25f, yellow);
        drawLine(aiPosition, (nextPositionVelocity + turnWorld) * 0.5f, white);
        drawSphere((nextPositionVelocity + turnWorld) * 0.5f, 0.25f, white);

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
                fmt("OT: %s", overtakeReason.c_str())
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
    if (brake > 0.0f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, false);

    gExt.SetSteeringAngle(mVehicle, lerp(actualAngle, desiredHeading, 20.0f * GAMEPLAY::GET_FRAME_TIME()));
    gExt.SetHandbrake(mVehicle, handbrake);
}

void Racer::updateAux() {
    bool headlightsOn = false;
    headlightsOn |= std::find(headLightsOnWeathers.begin(), headLightsOnWeathers.end(), GAMEPLAY::GET_PREV_WEATHER_TYPE_HASH_NAME()) != headLightsOnWeathers.end();
    headlightsOn |= TIME::GET_CLOCK_HOURS() > 19 || TIME::GET_CLOCK_HOURS() < 6;
    VEHICLE::SET_VEHICLE_LIGHTS(mVehicle, headlightsOn ? 3 : 4);
}

void Racer::updateStuck(const std::vector<Point> &coords) {
    if (coords.size() < 2 || !mActive || VEHICLE::GET_PED_IN_VEHICLE_SEAT(mVehicle, -1) == PLAYER::PLAYER_PED_ID()) {
        mIsStuck = false;
        mStuckStarted = 0;
        return;
    }

    if (!mIsStuck) {
        if (mStuckStarted == 0) {
            if (ENTITY::GET_ENTITY_SPEED(mVehicle) < 0.5f) {
                mStuckStarted = GetTickCount();
            }
        }
        if (GetTickCount() > mStuckStarted + mStuckThreshold && GetTickCount() < mStuckStarted + 2 * mStuckThreshold) {
            if (ENTITY::GET_ENTITY_SPEED(mVehicle) < 0.5f) {
                mIsStuck = true;
                if (!VEHICLE::IS_VEHICLE_ON_ALL_WHEELS(mVehicle)) {
                    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(mVehicle);
                }
                if (mDebugView) {
                    showNotification(fmt("Attempting to unstuck %s (%s)", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)),
                        VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
                }
            }
            else {
                mIsStuck = false;
                mStuckStarted = 0;
            }
        }
    }
    else {
        if (GetTickCount() > mStuckStarted + 2 * mStuckThreshold) {
            mIsStuck = false;
            mStuckStarted = 0;
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

    //if (returnIndex == coords.size() - 1 && (lookAheadDistance > smallestToLa || Distance(coords[0].v, coords[coords.size() - 1].v) > lookAheadDistance)) {
    //    returnIndex = 0;
    //    source = "start/stop";
    //}
    /*else */if ((smallestToLaIdx > smallestToAiIdx + nodeToConsiderMax || smallestToLaIdx < smallestToAiIdx - nodeToConsiderMax) && smallestToAiIdx > nodeToConsiderMin && smallestToAiIdx < coords.size() - nodeToConsiderMin) {
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
