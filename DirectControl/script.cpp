#include "script.h"
#include "Memory/VehicleExtensions.hpp"
#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"
#include "XInputControl.h"

Player player;
Ped playerPed;
Vehicle vehicleToControl;
VehicleExtensions ext;
XInputController controller(2);
bool playerInput = true;

bool isVehicleAvailable(Vehicle vehicle, Ped playerPed) {
    return vehicle != 0 &&
        ENTITY::DOES_ENTITY_EXIST(vehicle) &&
        playerPed == VEHICLE::GET_PED_IN_VEHICLE_SEAT(vehicle, -1);
}

bool amIInCar(Vehicle vehicle, Ped playerPed) {
    return vehicle != 0 &&
        ENTITY::DOES_ENTITY_EXIST(vehicle) &&
        PED::GET_VEHICLE_PED_IS_IN(playerPed, false);
}

float CalculateReduction(Vehicle vehicle) {
    float mult = 1;
    Vector3 vel = ENTITY::GET_ENTITY_VELOCITY(vehicle);
    Vector3 pos = ENTITY::GET_ENTITY_COORDS(vehicle, 1);
    Vector3 motion = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(vehicle, pos.x + vel.x, pos.y + vel.y, pos.z + vel.z);
    if (motion.y > 3) {
        mult = (0.15f + (powf((1.0f / 1.13f), (abs(motion.y) - 7.2f))));
        if (mult != 0) { mult = floorf(mult * 1000) / 1000; }
        if (mult > 1) { mult = 1; }
    }
    mult = (1 + (mult - 1) * 1.0f);
    return mult;
}

void drawSteeringLines(Vehicle vehicle, float steeringAngle, float nextAngle) {
    Vector3 velocityWorld = ENTITY::GET_ENTITY_VELOCITY(vehicle);
    Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(vehicle, 1);
    Vector3 travelWorld = velocityWorld + positionWorld;
    Vector3 travelRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(vehicle, travelWorld.x, travelWorld.y, travelWorld.z);

    Vector3 rotationVelocity = ENTITY::GET_ENTITY_ROTATION_VELOCITY(vehicle);
    Vector3 turnWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(vehicle, ENTITY::GET_ENTITY_SPEED(vehicle)*-sin(rotationVelocity.z), ENTITY::GET_ENTITY_SPEED(vehicle)*cos(rotationVelocity.z), 0.0f);
    Vector3 turnRelative = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(vehicle, turnWorld.x, turnWorld.y, turnWorld.z);
    float turnRelativeNormX = (travelRelative.x + turnRelative.x) / 2.0f;
    float turnRelativeNormY = (travelRelative.y + turnRelative.y) / 2.0f;
    Vector3 turnWorldNorm = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(vehicle, turnRelativeNormX, turnRelativeNormY, 0.0f);

    float steeringAngleRelX = ENTITY::GET_ENTITY_SPEED(vehicle)*-sin(steeringAngle);
    float steeringAngleRelY = ENTITY::GET_ENTITY_SPEED(vehicle)*cos(steeringAngle);
    Vector3 steeringWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(vehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);

    float nextRelX = ENTITY::GET_ENTITY_SPEED(vehicle)*-sin(nextAngle);
    float nextRelY = ENTITY::GET_ENTITY_SPEED(vehicle)*cos(nextAngle);
    Vector3 nextWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(vehicle, nextRelX, nextRelY, 0.0f);

    GRAPHICS::DRAW_LINE(positionWorld.x, positionWorld.y, positionWorld.z, travelWorld.x, travelWorld.y, travelWorld.z, 0, 255, 0, 255);
    GRAPHICS::DRAW_LINE(positionWorld.x, positionWorld.y, positionWorld.z, turnWorldNorm.x, turnWorldNorm.y, turnWorldNorm.z, 255, 0, 0, 255);
    GRAPHICS::DRAW_LINE(positionWorld.x, positionWorld.y, positionWorld.z, steeringWorld.x, steeringWorld.y, steeringWorld.z, 255, 0, 255, 255);
    GRAPHICS::DRAW_LINE(positionWorld.x, positionWorld.y, positionWorld.z, nextWorld.x, nextWorld.y, nextWorld.z, 255, 255, 255, 255);

}

float CalculateDesiredHeading(Vehicle vehicle, float steeringAngle, float steeringMax, float desiredHeading, float reduction) {
    float correction = desiredHeading * reduction;

    if (abs(ENTITY::GET_ENTITY_SPEED_VECTOR(vehicle, true).y) > 3.0f) {
        Vector3 velocityWorld = ENTITY::GET_ENTITY_VELOCITY(vehicle);
        Vector3 positionWorld = ENTITY::GET_ENTITY_COORDS(vehicle, 1);
        Vector3 travelWorld = velocityWorld + positionWorld;

        float steeringAngleRelX = ENTITY::GET_ENTITY_SPEED_VECTOR(vehicle, true).y*-sin(steeringAngle);
        float steeringAngleRelY = ENTITY::GET_ENTITY_SPEED_VECTOR(vehicle, true).y*cos(steeringAngle);
        Vector3 steeringWorld = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(vehicle, steeringAngleRelX, steeringAngleRelY, 0.0f);

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

float getSteeringAngle(Vehicle v) {
    float largestAngle = 0.0f;
    auto angles = ext.GetWheelSteeringAngles(v);

    for (auto angle : angles) {
        if (abs(angle) > abs(largestAngle)) {
            largestAngle = angle;
        }
    }
    return largestAngle;
}

void GetControls(float limitRadians, bool &handbrake, float &throttle, float &brake, float &steer) {
    controller.Update();
    if (controller.IsAvailable()) {
        handbrake = controller.IsButtonPressed(XInputController::RightShoulder);
        throttle = controller.GetAnalogValue(XInputController::RightTrigger);
        brake = controller.GetAnalogValue(XInputController::LeftTrigger);
        steer = controller.GetAnalogValue(XInputController::LeftThumbLeft);
        if (steer == 0.0f) steer = -controller.GetAnalogValue(XInputController::LeftThumbRight);
        bool reverseSwitch = controller.IsButtonPressed(XInputController::LeftShoulder);
        if (reverseSwitch)
            throttle = -controller.GetAnalogValue(XInputController::RightTrigger);

        float buzz = 0.0f;
        auto effects = ext.GetWheelSkidSmokeEffect(vehicleToControl);
        for (auto effect : effects) {
            buzz += effect;
        }
        buzz = abs(buzz);
        buzz /= (float)effects.size();
        buzz *= 2000.0f;
        if (buzz > 65535.0f)
            buzz = 65535.0f;

        if (buzz > 10.0f) {
            controller.Vibrate((int)buzz, (int)buzz);
        }
        showText(0.5, 0.1, 1.0, std::to_string(buzz));
    }
    else {
        handbrake = GetAsyncKeyState('O') & 0x8000 ? 1.0f : 0.0f;
        throttle = GetAsyncKeyState('I') & 0x8000 ? 1.0f : 0.0f;
        float reverse = GetAsyncKeyState('U') & 0x8000 ? 1.0f : 0.0f;
        if (reverse > 0.5)
            throttle = -1.0f;

        brake = GetAsyncKeyState('K') & 0x8000 ? 1.0f : 0.0f;
        float left = GetAsyncKeyState('J') & 0x8000 ? limitRadians : 0.0f;
        float right = GetAsyncKeyState('L') & 0x8000 ? limitRadians : 0.0f;
        steer = left;
        if (right > 0.5f) steer = -right;
    }
}

float GET_DISTANCE_BETWEEN_COORDS(Vector3 a, Vector3 b) {
    return GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(a.x, a.y, a.z, b.x, b.y, b.z, false);
}

void GetControls(Vehicle ai, Ped pedToStalk, float limitRadians, bool &handbrake, float &throttle, float &brake, float &steer) {
    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ai, 0, 5.0f, 0.0f);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(ai, 1);

    Vector3 preyPosition = ENTITY::GET_ENTITY_COORDS(pedToStalk, 1);

    Vector3 preyVector = Normalize(preyPosition - aiPosition);
    Vector3 forwardVector = Normalize(aiForward - aiPosition);
    //GRAPHICS::DRAW_LINE(aiPosition.x, aiPosition.y, aiPosition.z, preyPosition.x, preyPosition.y, preyPosition.z, 255, 0, 0, 255);
    //GRAPHICS::DRAW_LINE(aiForward.x, aiForward.y, aiForward.z, preyPosition.x, preyPosition.y, preyPosition.z, 0, 255, 0, 255);

    float aiHeading = atan2(forwardVector.y, forwardVector.x);
    float preyHeading = atan2(preyVector.y, preyVector.x);

    float turn = atan2(sin(preyHeading - aiHeading), cos(preyHeading - aiHeading));
    float distance = GET_DISTANCE_BETWEEN_COORDS(aiPosition, preyPosition);

    steer = constrain(turn * 1.33f, -limitRadians, limitRadians);

    const float followDistance = 20.0f;

    if (distance > followDistance) {
        throttle = map(distance, followDistance, 35.0f, 0.0f, 1.0f);
        throttle = constrain(throttle, 0.0f, 1.0f);

        if (turn > 2.0944 || turn < -2.0944) 
            throttle = -throttle;
        
        handbrake = abs(turn) > limitRadians * 2.0f && ENTITY::GET_ENTITY_SPEED_VECTOR(ai, true).y > 12.0f;

        float distPerp = (abs(turn) - 1.5708f) / 1.5708f;
        throttle *= map(abs(distPerp), 0.0f, 1.0f, 0.5f, 1.0f);

        if (!handbrake && abs(turn) > limitRadians && ENTITY::GET_ENTITY_SPEED_VECTOR(ai, true).y > 12.0f) {
            brake = (abs(turn) - limitRadians) / 3.14f * throttle;
            throttle *= 0.33f;
        }
        else {
            brake = 0.0f;
        }
    }
    else {
        throttle = 0.0f;
        brake = map(distance, followDistance / 2.0f, followDistance, 1.0f, 0.0f);
        brake = constrain(brake, 0.0f, 1.0f);
    }

    showText(0.4, 0.05, 0.5, "T: " + std::to_string(throttle));
    showText(0.4, 0.10, 0.5, "B: " + std::to_string(brake));
    showText(0.4, 0.15, 0.5, "S: " + std::to_string(steer));
    showText(0.4, 0.20, 0.5, "H: " + std::to_string(handbrake));
    showText(0.4, 0.25, 0.5, "Rad: " + std::to_string(turn));
    showText(0.5, 0.25, 0.5, "Deg: " + std::to_string(rad2deg(turn)));
}

void UpdateControl() {
    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(vehicleToControl))
        VEHICLE::SET_VEHICLE_ENGINE_ON(vehicleToControl, true, true, true);

    float actualAngle = getSteeringAngle(vehicleToControl);
    float limitRadians = ext.GetMaxSteeringAngle(vehicleToControl);
    float reduction = CalculateReduction(vehicleToControl);

    bool handbrake;
    float throttle;
    float brake;
    float steer;

    if (playerInput)
        GetControls(limitRadians, handbrake, throttle, brake, steer);
    else
        GetControls(vehicleToControl, playerPed, limitRadians, handbrake, throttle, brake, steer);

    float desiredHeading = CalculateDesiredHeading(vehicleToControl, actualAngle, limitRadians, steer, reduction);

    ext.SetThrottleP(vehicleToControl, throttle);
    ext.SetBrakeP(vehicleToControl, brake);
    if (brake > 0.0f)
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(vehicleToControl, true);
    else
        VEHICLE::SET_VEHICLE_BRAKE_LIGHTS(vehicleToControl, false);
    
    ext.SetSteeringAngle(vehicleToControl, lerp(actualAngle, desiredHeading, 20.0f * GAMEPLAY::GET_FRAME_TIME()));
    ext.SetHandbrake(vehicleToControl, handbrake);

    showText(0.1, 0.05, 0.5, "T: " + std::to_string(throttle));
    showText(0.1, 0.10, 0.5, "B: " + std::to_string(brake));
    showText(0.1, 0.15, 0.5, "S: " + std::to_string(steer));
    showText(0.1, 0.20, 0.5, "H: " + std::to_string(handbrake));
    showText(0.1, 0.25, 0.5, "Wanted: " + std::to_string(desiredHeading));
    showText(0.1, 0.30, 0.5, "Actual: " + std::to_string(actualAngle));
    showText(0.1, 0.35, 0.5, "Reduct: " + std::to_string(reduction));

    drawSteeringLines(vehicleToControl, actualAngle, desiredHeading);
}

void update()
{
    player = PLAYER::PLAYER_ID();
    playerPed = PLAYER::PLAYER_PED_ID();
    Vehicle vehicle = PED::GET_VEHICLE_PED_IS_IN(playerPed, false);

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("cc"))) {
        if (ENTITY::DOES_ENTITY_EXIST(vehicleToControl)) {
            showNotification("Stop controlling vehicle " + std::to_string(vehicleToControl));
            ENTITY::SET_ENTITY_AS_MISSION_ENTITY(vehicleToControl, false, true);
            ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&vehicleToControl);
            vehicleToControl = 0;
        }
        else if (isVehicleAvailable(vehicle, playerPed) || amIInCar(vehicle, playerPed)) {
            vehicleToControl = vehicle;
            ENTITY::SET_ENTITY_AS_MISSION_ENTITY(vehicleToControl, true, false);
            showNotification("Controlling vehicle " + std::to_string(vehicle));
            PED::SET_PED_INTO_VEHICLE(playerPed, vehicleToControl, -2);
        }
    }
    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("pi")))
        playerInput = !playerInput;

    if (vehicle == vehicleToControl && playerPed == VEHICLE::GET_PED_IN_VEHICLE_SEAT(vehicle, -1)) {
        return;
    }

	if (ENTITY::DOES_ENTITY_EXIST(vehicleToControl)) {
        UpdateControl();
	}
    else {
        vehicleToControl = 0;
    }
}

void main() {
    logger.Write(INFO, "Direct vehicle control test script");
    logger.Write(INFO, "Use the \"cc\" cheat while in a car to take control");
    logger.Write(INFO, "IJKL as WASD, U is reverse, O is handbrake");
    logger.Write(INFO, "Use the \"cc\" cheat again to remove control");
    logger.Write(INFO, "Use the \"pi\" cheat to switch between player input and simple chase AI");

    ext.initOffsets();

    while (true) {
		update();
		WAIT(0);
	}
}

void ScriptMain() {
	srand(GetTickCount());
	main();
}
