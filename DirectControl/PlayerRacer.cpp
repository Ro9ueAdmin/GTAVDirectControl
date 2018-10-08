#include "PlayerRacer.h"
#include <inc/natives.h>
#include "Util/StringFormat.h"
#include "Util/MathExt.h"
#include "Util/Color.h"
#include "Util/UIUtils.h"
#include "Memory/VehicleExtensions.hpp"

PlayerRacer::PlayerRacer(Vehicle vehicle, int playerNumber) :
    Racer(vehicle),
    mXInput(playerNumber) {
    mBlip.SetName(fmt("Player %s %s", getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle)), VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle)));
    mBlip.SetColor(BlipColorWhite);
}

void PlayerRacer::UpdateControl() {
    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(mVehicle))
        VEHICLE::SET_VEHICLE_ENGINE_ON(mVehicle, true, true, true);

    float actualAngle = getSteeringAngle();
    float limitRadians = gExt.GetMaxSteeringAngle(mVehicle);
    float reduction = calculateReduction();

    bool handbrake = false;
    float throttle = 0.0f;
    float brake = 0.0f;
    float steer = 0.0f;

    getControls(handbrake, throttle, brake, steer);

    float desiredHeading = calculateDesiredHeading(actualAngle, limitRadians, steer, reduction);

    gExt.SetThrottleP(mVehicle, lerp(gExt.GetThrottleP(mVehicle), throttle, GAMEPLAY::GET_FRAME_TIME() / 0.050f));

    
    gExt.SetBrakeP(mVehicle, brake);
    if (brake > 0.0f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(mVehicle, false);

    gExt.SetSteeringAngle(mVehicle, lerp(actualAngle, desiredHeading, GAMEPLAY::GET_FRAME_TIME() / 0.050f));
    gExt.SetHandbrake(mVehicle, handbrake);

    if (mDebugView) {
        drawDebugLines(actualAngle, desiredHeading);
    }
}

void PlayerRacer::getControls(bool& handbrake, float& throttle, float& brake, float& steer) {
    mXInput.Update();
    if (mXInput.IsAvailable()) {
        handbrake = mXInput.IsButtonPressed(XInputController::RightShoulder);
        throttle = mXInput.GetAnalogValue(XInputController::RightTrigger);
        brake = mXInput.GetAnalogValue(XInputController::LeftTrigger);
        steer = mXInput.GetAnalogValue(XInputController::LeftThumbLeft);
        if (steer == 0.0f) steer = -mXInput.GetAnalogValue(XInputController::LeftThumbRight);
        bool reverseSwitch = mXInput.IsButtonPressed(XInputController::LeftShoulder);
        if (reverseSwitch)
            throttle = -mXInput.GetAnalogValue(XInputController::RightTrigger);

        float buzz = 0.0f;
        auto effects = gExt.GetWheelSkidSmokeEffect(mVehicle);
        for (auto effect : effects) {
            buzz += effect;
        }
        buzz = abs(buzz);
        buzz /= (float)effects.size();
        buzz *= 2000.0f;
        if (buzz > 65535.0f)
            buzz = 65535.0f;

        if (buzz > 10.0f) {
            mXInput.Vibrate((int)buzz, (int)buzz);
        }
        else {
            mXInput.Vibrate(0, 0);
        }
    }
    else {
        handbrake = GetAsyncKeyState('O') & 0x8000 ? 1.0f : 0.0f;
        throttle = GetAsyncKeyState('I') & 0x8000 ? 1.0f : 0.0f;
        float reverse = GetAsyncKeyState('U') & 0x8000 ? 1.0f : 0.0f;
        if (reverse > 0.5)
            throttle = -1.0f;

        brake = GetAsyncKeyState('K') & 0x8000 ? 1.0f : 0.0f;
        float left = GetAsyncKeyState('J') & 0x8000 ? 1.0f : 0.0f;
        float right = GetAsyncKeyState('L') & 0x8000 ? 1.0f : 0.0f;
        steer = left - right;
    }
}

void PlayerRacer::drawDebugLines(float steeringAngle, float nextAngle) {
    Vector3 velocityWorld = ENTITY::GET_ENTITY_VELOCITY(mVehicle);
    Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(mVehicle, 1);
    Vector3 travelWorld = velocityWorld + positionWorld;
    Vector3 travelRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(
        mVehicle, travelWorld.x, travelWorld.y, travelWorld.z);

    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(mVehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(
        mVehicle, ENTITY::GET_ENTITY_SPEED(mVehicle) * -sin(rotationVelocity.z),
        ENTITY::GET_ENTITY_SPEED(mVehicle) * cos(rotationVelocity.z), 0.0f);
    Vector3 turnRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(
        mVehicle, turnWorld.x, turnWorld.y, turnWorld.z);
    float turnRelativeNormX = (travelRelative.x + turnRelative.x) / 2.0f;
    float turnRelativeNormY = (travelRelative.y + turnRelative.y) / 2.0f;
    Vector3 turnWorldNorm = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(
        mVehicle, turnRelativeNormX, turnRelativeNormY, 0.0f);

    float steeringAngleRelX = ENTITY::GET_ENTITY_SPEED(mVehicle) * -sin(steeringAngle);
    float steeringAngleRelY = ENTITY::GET_ENTITY_SPEED(mVehicle) * cos(steeringAngle);
    Vector3 steeringWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(
        mVehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);

    float nextRelX = ENTITY::GET_ENTITY_SPEED(mVehicle) * -sin(nextAngle);
    float nextRelY = ENTITY::GET_ENTITY_SPEED(mVehicle) * cos(nextAngle);
    Vector3 nextWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, nextRelX, nextRelY, 0.0f);

    drawLine(positionWorld, travelWorld, { 0, 255, 0, 255 });
    drawLine(positionWorld, turnWorldNorm, { 255, 0, 0, 255 });
    drawLine(positionWorld, steeringWorld, { 255, 0, 255, 255 });
    drawLine(positionWorld, nextWorld, { 255, 255, 255, 255 });

    Vector3 p = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    Vector3 min, max;
    GAMEPLAY::GET_MODEL_DIMENSIONS(ENTITY::GET_ENTITY_MODEL(mVehicle), &min, &max);
    Vector3 up = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, 0.0f, 0.0f, ((max.z - min.z) / 2.0f) + 1.0f);

    float actualAngle = getSteeringAngle() - deg2rad(90.0f);
    Vector3 forward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(mVehicle, -sin(actualAngle), cos(actualAngle), 0.0f);
    Vector3 dir = forward - p;
    Vector3 rot{};
    rot.y = 90.0f;

    Color c{
        0,
        0,
        255,
        255
    };

    drawChevron(up, dir, rot, 0.5f, 1.0f, c);
}
