#include "Racer.h"
#include <inc/natives.h>
#include "Util/StringFormat.h"
#include "Util/MathExt.h"
#include "Util/Color.h"
#include "Util/UIUtils.h"
#include "Memory/VehicleExtensions.hpp"

Racer::Racer(Vehicle vehicle) :
    mVehicle(vehicle),
    mActive(true),
    mDebugView(true) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
    mBlip = std::make_unique<BlipX>(mVehicle);
    mBlip->SetSprite(BlipSpritePersonalVehicleCar);
    mBlip->SetName(fmt("AI Racer %s", VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip->SetColor(BlipColorYellow);
}

Racer::Racer(Racer &&other) noexcept :
    mVehicle(other.mVehicle),
    mActive(other.mActive),
    mDebugView(other.mDebugView) {
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(mVehicle, true, false);
    mBlip = std::make_unique<BlipX>(mVehicle);
    mBlip->SetSprite(BlipSpritePersonalVehicleCar);
    mBlip->SetName(fmt("AI Racer %s", VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip->SetColor(BlipColorYellow);
    other.mVehicle = 0;
}

Racer & Racer::operator=(Racer &&other) noexcept {
    if (this != &other) {
        mVehicle = other.mVehicle;
        mBlip = std::make_unique<BlipX>(mVehicle);
        mBlip->SetSprite(BlipSpritePersonalVehicleCar);
        mBlip->SetName(fmt("AI Racer %s", VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
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
    mBlip->Delete();
    if (mBlip) {
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

void Racer::getControls(const std::vector<Vector3>& coords, float limitRadians, bool & handbrake, float & throttle, float & brake, float & steer) {
    handbrake = false;
    throttle = 0.0f;
    brake = 1.0f;
    steer = 0.0f;

    if (coords.size() < 2)
        return;

    if (!mActive)
        return;

    Vector3 aiVelocity = ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true);
    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0, 5.0f, 0.0f);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);

    float lookAheadThrottle = constrain(3.5f * ENTITY::GET_ENTITY_SPEED(mVehicle), 15.0f, 9999.0f);
    float lookAheadSteer = constrain(0.8f * ENTITY::GET_ENTITY_SPEED(mVehicle), 10.0f, 9999.0f);
    float lookAheadBrake = constrain(2.5f * ENTITY::GET_ENTITY_SPEED(mVehicle), 15.0f, 9999.0f);

    float cornerRadiusThrottle;
    float cornerRadiusSteer;
    float cornerRadiusBrake;

    Vector3 nextPositionThrottle = getCoord(coords, lookAheadThrottle, cornerRadiusThrottle);
    Vector3 nextPositionSteer = getCoord(coords, lookAheadSteer, cornerRadiusSteer);
    Vector3 nextPositionBrake = getCoord(coords, lookAheadBrake, cornerRadiusBrake);

    Vector3 nextVectorThrottle = Normalize(nextPositionThrottle - aiPosition);
    Vector3 nextVectorSteer = Normalize(nextPositionSteer - aiPosition);
    Vector3 nextVectorBrake = Normalize(nextPositionBrake - aiPosition);

    Vector3 forwardVector = Normalize(aiForward - aiPosition);

    float aiHeading = atan2(forwardVector.y, forwardVector.x);
    float nextHeadingThrottle = atan2(nextVectorThrottle.y, nextVectorThrottle.x);
    float nextHeadingSteer = atan2(nextVectorSteer.y, nextVectorSteer.x);
    float nextHeadingBrake = atan2(nextVectorBrake.y, nextVectorBrake.x);

    float turnThrottle = atan2(sin(nextHeadingThrottle - aiHeading), cos(nextHeadingThrottle - aiHeading));
    float turnSteer = atan2(sin(nextHeadingSteer - aiHeading), cos(nextHeadingSteer - aiHeading));
    float turnBrake = atan2(sin(nextHeadingBrake - aiHeading), cos(nextHeadingBrake - aiHeading));

    float distanceThrottle = Distance(aiPosition, nextPositionThrottle);
    float distanceSteer = Distance(aiPosition, nextPositionSteer);
    float distanceBrake = Distance(aiPosition, nextPositionBrake);

    float distPerpThrottle = abs((abs(turnThrottle) - 1.5708f) / 1.5708f);
    float distPerpSteer = abs((abs(turnSteer) - 1.5708f) / 1.5708f);
    float distPerpBrake = abs((abs(turnBrake) - 1.5708f) / 1.5708f);

    float steerMult = 1.33f;

    float aiSpeed = ENTITY::GET_ENTITY_SPEED(mVehicle);

    throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    throttle *= map(distPerpThrottle, 0.0f, 1.0f, 0.5f, 1.0f);

    // Decrease throttle when starting to spin out, increase countersteer
    Vector3 nextPositionVelocity = aiPosition + ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle)*-sin(rotationVelocity.z), ENTITY::GET_ENTITY_SPEED(mVehicle)*cos(rotationVelocity.z), 0.0f);

    float angle = GetAngleBetween(ENTITY::GET_ENTITY_VELOCITY(mVehicle), turnWorld - aiPosition);
    float csMult = constrain(map(angle, deg2rad(0.00f), deg2rad(90.0f), 1.0f, 2.0f), 0.0f, 2.0f);
    float spinoutMult = constrain(map(angle, deg2rad(45.00f), deg2rad(90.0f), 1.0f, 0.0f), 0.0f, 1.0f);
    //showText(0.1f, 0.10f, 0.5, fmt("SpinoutMult %.03f", spinoutMult));
    //showText(0.1f, 0.15f, 0.5, fmt("csMult %.03f", csMult));

    // start oversteer detect
    float oversteer = 0.0f;
    float angleOverSteer = acos(aiVelocity.y / ENTITY::GET_ENTITY_SPEED(mVehicle))* 180.0f / 3.14159265f;
    if (isnan(angleOverSteer))
        angleOverSteer = 0.0;

    if (angleOverSteer > 10.0f && aiVelocity.y > 1.0f) {
        oversteer = angleOverSteer / 90.0f;
    }
    //showText(0.1f, 0.20f, 0.5, fmt("%sOversteer %.03f", oversteer > 0.1f ? "~g~" : "", angleOverSteer));

    if (aiSpeed > 5.0f) {
        throttle *= spinoutMult;
        if (oversteer > 0.1f)
            steerMult *= csMult;
    }

    handbrake = abs(turnSteer) > limitRadians * 2.0f && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 12.0f;

    float maxBrake = map(aiSpeed, distanceThrottle * 0.50f, distanceBrake * 0.75f, -0.3f, 3.0f);

    if (abs(turnBrake) > limitRadians && ENTITY::GET_ENTITY_SPEED_VECTOR(mVehicle, true).y > 10.0f) {
        float brakeTurn = map(distPerpBrake, 0.0f, 1.0f, 1.0f, 0.0f);
        if (brakeTurn > maxBrake)
            maxBrake = brakeTurn;
    }

    // Slow down when next corner is a tight one
    // TODO: Consider scaling the corner radius to slow down for by speed
    // Current: Scales slow-down factor by speed regardless of upcoming corner radius
    float avgRadius1 = (cornerRadiusThrottle + cornerRadiusBrake) / 2.0f;
    float avgRadius2 = (cornerRadiusSteer + cornerRadiusBrake) / 2.0f;
    float avgRadius = avgRadius1 < avgRadius2 ? avgRadius1 : avgRadius2;

    if (avgRadius > 0.0f && avgRadius < 150.0f) {
        float brakeForRadius = map(avgRadius, 20.0f, 150.0f, 1.0f, 0.0f);

        brakeForRadius *= map(ENTITY::GET_ENTITY_SPEED(mVehicle), 30.0f, 50.0f, 0.0f, 1.0f);

        if (brakeForRadius > maxBrake) {
            maxBrake = brakeForRadius;
            //showText(0.45, 0.15, 1.0, "~r~!!!");
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
        drawLine(aiPosition, nextPositionSteer, blue);
        drawLine(aiPosition, nextPositionBrake, red);

        // draw chevron
        Vector3 p = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
        Vector3 min, max;
        GAMEPLAY::GET_MODEL_DIMENSIONS(ENTITY::GET_ENTITY_MODEL(mVehicle), &min, &max);
        Vector3 up = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 1.0f);

        float actualAngle = getSteeringAngle() - deg2rad(90.0f);
        float steeringAngleRelX = -sin(actualAngle);
        float steeringAngleRelY = cos(actualAngle);

        Vector3 forward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);
        Vector3 dir = forward - p;
        Vector3 rot{};
        rot.y = 90.0f;

        Color c{
            constrain(static_cast<int>(map(brake, 0.0f, 1.0f, 127.0f, 255.0f)), 0, 255),
            constrain(static_cast<int>(map(brake, 1.0f, 0.0f, 127.0f, 255.0f)), 0, 255),
            0,
            255
        };

        drawChevron(up, dir, rot, 1.0f, throttle, c);

        // spinout ratio
        drawLine(aiPosition, nextPositionVelocity, white);
        drawSphere(nextPositionVelocity, 0.25f, white);
        drawLine(aiPosition, turnWorld, yellow);
        drawSphere(turnWorld, 0.25f, yellow);
    }
}

void Racer::UpdateControl(const std::vector<Vector3> &coords) {
    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(mVehicle))
        VEHICLE::SET_VEHICLE_ENGINE_ON(mVehicle, true, true, true);

    float actualAngle = getSteeringAngle();
    float limitRadians = gExt.GetMaxSteeringAngle(mVehicle);
    float reduction = calculateReduction();

    bool handbrake = false;
    float throttle = 0.0f;
    float brake = 0.0f;
    float steer = 0.0f;

    getControls(coords, limitRadians, handbrake, throttle, brake, steer);

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
        radius = 1000.0f;
    else if (radius > 1000.0f)
        radius = 1000.0f;
    else if (std::isnan(radius))
        radius = 1000.0f;

    //showDebugInfo3D(coords[focus], {
    //    fmt("%.03f m length", length),
    //    fmt("%.03f degrees", rad2deg(angle)),
    //    fmt("%.03f m radius", radius)
    //});
    return radius;
}

Vector3 Racer::getCoord(const std::vector<Vector3>& coords, float lookAheadDistance, float &cornerRadius) {
    float smallestToLa = 9999.9f;
    int smallestToLaIdx = 0;
    float smallestToAi = 9999.9f;
    int smallestToAiIdx = 0;

    float actualAngle = getSteeringAngle();

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

    // Only consider viable nodes, to not cut the track. 
    // Significant overshoot still makes AI choose closest track node, 
    // so prevent this from happening with walls or something. 
    int nodesToConsider = static_cast<int>(1.25f * lookAheadDistance / Distance(coords[smallestToAiIdx], coords[(smallestToAiIdx+1)% coords.size()]));

    // Expected Look-ahead index
    int expectedLaIdx = (smallestToAiIdx + nodesToConsider) % coords.size();
    int expectedLaIdxB = (smallestToAiIdx + nodesToConsider);

    if (smallestToLaIdx < smallestToAiIdx && smallestToLaIdx < nodesToConsider && smallestToAiIdx > coords.size() - nodesToConsider) {
        // Ensure start/stop is continuous
        returnIndex = smallestToLaIdx;
    }
    else if (smallestToLaIdx > expectedLaIdxB) {
        // Ensure track is followed continuously (no cutting off entire sections)
        returnIndex = expectedLaIdx;
    }
    else if (smallestToAiIdx >= smallestToLaIdx) {
        // Ensure going forwards
        returnIndex = (smallestToAiIdx + 10) % coords.size();
    }

    int nodesForRadius = 6;
    float averageRadius = 0.0f;

    for (int i = 0; i < nodesForRadius; ++i) {
        averageRadius += getCornerRadius(coords, (returnIndex + i) % coords.size());
    }

    cornerRadius = averageRadius / static_cast<float>(nodesForRadius);
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
