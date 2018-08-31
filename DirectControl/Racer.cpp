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

void Racer::getControls(const std::vector<Vector3> &coords, const std::vector<Vehicle> &opponents, float limitRadians,
                        float actualAngle, bool &handbrake, float &throttle, float &brake, float &steer) {
    handbrake = false;
    throttle = 0.0f;
    brake = 1.0f;
    steer = 0.0f;
    
    Vector3 npcSteerPos{};
    {
        Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
        Vector3 aiForward = Normalize(ENTITY::GET_ENTITY_FORWARD_VECTOR(mVehicle));
        float aiHeading = atan2(aiForward.y, aiForward.x);
        Vector3 aiDimMin, aiDimMax;
        GAMEPLAY::GET_MODEL_DIMENSIONS(ENTITY::GET_ENTITY_MODEL(mVehicle), &aiDimMin, &aiDimMax);
        Vector3 aiDim = aiDimMax - aiDimMin;
        float aiLookahead = ENTITY::GET_ENTITY_SPEED(mVehicle) * gSettings.AILookaheadSteerSpeedMult;

        float closest = 10000.0;
        int closestIdx = opponents.size();
        for (int i = 0; i < opponents.size(); ++i) {
            Vehicle npc = opponents[i];
            Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, 1);
            if (!ENTITY::DOES_ENTITY_EXIST(npc)) continue;
            if (npc == mVehicle) continue;
            if (Distance(aiPosition, npcPosition) > closest) continue;
            closest = Distance(aiPosition, npcPosition);
            closestIdx = i;
        }

        if (closestIdx != opponents.size()) {
            Vehicle npc = opponents[closestIdx];
            Vector3 npcPosition = ENTITY::GET_ENTITY_COORDS(npc, 1);
            float npcDistance = Distance(aiPosition, npcPosition);
            if (npcDistance < (aiLookahead > 30.0f ? aiLookahead : 30.0f)) {
                Vector3 npcDimMin, npcDimMax;
                GAMEPLAY::GET_MODEL_DIMENSIONS(ENTITY::GET_ENTITY_MODEL(mVehicle), &npcDimMin, &npcDimMax);
                Vector3 npcDim = npcDimMax - npcDimMin;

                Vector3 npcDirection = Normalize(npcPosition - aiPosition);
                Vector3 npcRelativePosition = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(mVehicle, npcPosition.x, npcPosition.y, npcPosition.z);
                float npcHeading = GetAngleBetween(aiForward, npcDirection) * sgn(npcRelativePosition.x);

                bool inRange = false;

                float npcSpeed = ENTITY::GET_ENTITY_SPEED(npc);
                float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

                if (Distance(aiPosition, npcPosition) < aiSpeed && abs(npcHeading) < deg2rad(90.0f) && aiSpeed >= npcSpeed) {
                    Vector3 npcSteerOffset{};
                    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);

                    npcSteerOffset.x = npcDim.x * 2.0f * sgn(-npcRelativePosition.x - sin(rotationVelocity.z));
                    npcSteerOffset.y = constrain(-npcRelativePosition.y, -npcDim.y * 1.0f, npcDim.y * 3.0f) + aiDim.y;
                    npcSteerPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, npcSteerOffset.x, npcSteerOffset.y, 0.0f);
                    drawSphere(npcSteerPos, 0.50f, { 0, 255, 255, 255 });
                    inRange = true;
                }

                Vector3 up = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(npc, 0.0f, 0.0f, 2.0f);
                showDebugInfo3D(up, 10.0f, {
                    fmt("Rel. Angle: %.03f", rad2deg(npcHeading)),
                    fmt("%sDim(%.02f, %.02f, %.02f)", inRange ? "~g~" : "~r~", npcDim.x, npcDim.y, npcDim.z)
                    });
                closest = Distance(aiPosition, npcPosition);
            }
        }

    }

    if (coords.size() < 2)
        return;

    if (!mActive)
        return;

    bool dbgSpinThrottle = false;
    bool dbgSpinCountersteer = false;
    bool dbgBrakeForAngle = false;
    bool dbgBrakeForHeading = false;
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

    if (Length(npcSteerPos) > 0.0f) {
        float smallestToAiDist = 10000.0f;
        float smallestToNpcDist = 10000.0f;
        Vector3 smallestToAi{};
        Vector3 smallestToNpc{};
        for (auto i = 0; i < coords.size(); ++i) {
            float distanceAi = Distance(aiPosition, coords[i]);
            float distanceNpc = Distance(npcSteerPos, coords[i]);

            if (distanceAi < smallestToAiDist) {
                smallestToAiDist = distanceAi;
                smallestToAi = coords[i];
            }
            if (distanceNpc < smallestToNpcDist) {
                smallestToNpcDist = distanceNpc;
                smallestToNpc = coords[i];
            }

        }

        // dist < track width?
        if (Distance(aiPosition, smallestToAi) > Distance(npcSteerPos, smallestToNpc) || Distance(npcSteerPos, smallestToNpc) < 5.0f) {
            nextPositionSteer = npcSteerPos;
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

    if (Length(npcSteerPos) == 0.0f) {
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
        drawLine(aiPosition, nextPositionVelocity, white);
        drawSphere(nextPositionVelocity, 0.25f, white);
        drawLine(aiPosition, turnWorld, yellow);
        drawSphere(turnWorld, 0.25f, yellow);

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
            });
        }
    }
}

void Racer::UpdateControl(const std::vector<Vector3> &coords, const std::vector<Vehicle> &opponents) {
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

void Racer::updateStuck(const std::vector<Vector3> &coords) {
    if (coords.size() < 2 || !mActive) {
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

float Racer::getCornerRadius(const std::vector<Vector3> &coords, int focus) {
    int prev = focus - 1;
    if (prev < 0) prev = static_cast<int>(coords.size()) - 1;

    int next = (focus + 1) % coords.size();

    float angle = GetAngleBetween(coords[focus] - coords[next], coords[focus] - coords[prev]);

    float length = Distance(coords[prev], coords[focus]);
    float radius = (0.5f*length) / cos(angle*0.5f);

    if (radius <= 0.0f)
        radius = 10000.0f;
    else if (radius > 10000.0f)
        radius = 10000.0f;
    else if (std::isnan(radius))
        radius = 10000.0f;

    return radius;
}

Vector3 Racer::getCoord(const std::vector<Vector3>& coords,
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
        float distanceAi = Distance(aiPos, coords[i]);
        float distanceLa = Distance(aiForward, coords[i]);

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
    int nodeToConsiderMin = static_cast<int>(1.0f * lookAheadDistance / Distance(coords[smallestToAiIdx], coords[(smallestToAiIdx + 1) % coords.size()]));
    int nodeToConsiderMax = static_cast<int>(2.0f * lookAheadDistance / Distance(coords[smallestToAiIdx], coords[(smallestToAiIdx + 1) % coords.size()]));

    if ((smallestToLaIdx > smallestToAiIdx + nodeToConsiderMax || smallestToLaIdx < smallestToAiIdx - nodeToConsiderMax) && smallestToAiIdx > nodeToConsiderMin && smallestToAiIdx < coords.size() - nodeToConsiderMin) {
        // Ensure track is followed continuously (no cutting off entire sections)
        returnIndex = (smallestToAiIdx + nodeToConsiderMin) % coords.size();
        source = "continuous";
    }
    else 
    if (smallestToAiIdx >= smallestToLaIdx && smallestToAiIdx - smallestToLaIdx < coords.size() / 2) {
        // Ensure going forwards
        returnIndex = (smallestToAiIdx + (int)lookAheadDistance) % coords.size();
        source = "forwards";
    }

    return coords[returnIndex];
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
