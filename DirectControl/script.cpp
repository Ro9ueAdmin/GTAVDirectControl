#include "script.h"
#include "Memory/VehicleExtensions.hpp"
#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"
#include "XInputControl.h"
#include "Blip.h"

#include <thirdparty/json.hpp>
#include <fstream>
#include <iomanip>
#include "Util/StringFormat.h"

// for convenience
using json = nlohmann::json;

Player player;
Ped playerPed;
Vehicle vehicleToControl;
VehicleExtensions ext;
XInputController controller(2);
bool playerInput = true;
std::vector<std::unique_ptr<BlipX>> blips;
std::vector<Vector3> playerCoords;
bool recording = false;

void drawLine(Vector3 a, Vector3 b, Color c) {
    GRAPHICS::DRAW_LINE(a.x, a.y, a.z, b.x, b.y, b.z, c.R, c.G, c.B, c.A);
}

void drawSphere(Vector3 p, float scale, Color c) {
    GRAPHICS::DRAW_MARKER(eMarkerType::MarkerTypeDebugSphere, 
        p.x, p.y, p.z, 
        0.0f, 0.0f, 0.0f, 
        0.0f, 0.0f, 0.0f, 
        scale, scale, scale, 
        c.R, c.G, c.B, c.A, 
        false, false, 2, false, nullptr, nullptr, false);
}

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

Vector3 getCoord(Vehicle ai, const std::vector<Vector3> &coords, float lookAheadDistance, std::string what, int iNextRow, Color c) {
    float smallestToLa = 9999.9f;
    int smallestToLaIdx = 0;
    float smallestToAi = 9999.9f;
    int smallestToAiIdx = 0;

    int expectedLaIdx = 0;

    float actualAngle = getSteeringAngle(ai);

    Vector3 aiPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ai, 0.0f, 0.0f, 0.0f);

    float steeringAngleRelX = lookAheadDistance*-sin(actualAngle);
    float steeringAngleRelY = lookAheadDistance*cos(actualAngle);

    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ai, steeringAngleRelX, steeringAngleRelY, 0.0f);
    //Color cc = c;
    //cc.A = 127;
    //drawSphere(aiForward, 0.5f, cc);

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

    // Ensure start/stop is continuous
    if (smallestToLaIdx < smallestToAiIdx && smallestToLaIdx < coords.size() / 5 && smallestToAiIdx > coords.size() - coords.size() / 5) {
        showText(0.75f, 0.15f + 0.05f * iNextRow, 0.5f, fmt("%s: idxLoop = %d", what.c_str(), smallestToLaIdx));
        return coords[smallestToLaIdx];
    }

    // Ensure track is followed continuously (no cutting off entire sections)
    expectedLaIdx = (smallestToAiIdx + (int)((1.0f * lookAheadDistance) / Distance(coords[0], coords[1]))) % coords.size();
    int expectedLaIdxB = (smallestToAiIdx + (int)((1.0f * lookAheadDistance) / Distance(coords[0], coords[1])));
    drawSphere(coords[expectedLaIdx], 0.5f, c);

    if (smallestToLaIdx > expectedLaIdxB) {
        showText(0.75f, 0.15f + 0.05f * iNextRow, 0.5f, fmt("%s: idxCont = %d", what.c_str(), expectedLaIdx));
        return coords[expectedLaIdx];
    }

    // Ensure going forwards
    if (smallestToAiIdx >= smallestToLaIdx) {
        int nextIdx = (smallestToAiIdx + 10) % coords.size();
        showText(0.75f, 0.15f + 0.05f * iNextRow, 0.5f, fmt("%s: idxForw = %d", what.c_str(), nextIdx));
        return coords[nextIdx];
    }

    showText(0.75f, 0.15f + 0.05f * iNextRow, 0.5f, fmt("%s: idxNext = %d", what.c_str(), smallestToLaIdx));
    return coords[smallestToLaIdx];
}

void GetControlsFromAI(Vehicle ai, const std::vector<Vector3> &coords, float limitRadians, bool &handbrake, float &throttle, float &brake, float &steer) {
    handbrake = false;
    throttle = 0.0f;
    brake = 0.0f;
    steer = 0.0f;

    if (coords.size() < 2)
        return;

    Vector3 aiForward = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ai, 0, 5.0f, 0.0f);
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(ai, 1);

    float lookAheadThrottle = constrain(3.5f * ENTITY::GET_ENTITY_SPEED(ai), 15.0f, 9999.0f);
    float lookAheadSteer = constrain(0.8f * ENTITY::GET_ENTITY_SPEED(ai), 10.0f, 9999.0f);
    float lookAheadBrake = constrain(2.5f * ENTITY::GET_ENTITY_SPEED(ai), 15.0f, 9999.0f);

    Color red{ 255, 0, 0, 255 };
    Color green{ 0, 255, 0, 255 };
    Color blue{ 0, 0, 255, 255 };

    Vector3 nextPositionThrottle = getCoord(ai, coords, lookAheadThrottle, "T", 0, green);
    Vector3 nextPositionSteer = getCoord(ai, coords, lookAheadSteer, "S", 2, blue);
    Vector3 nextPositionBrake = getCoord(ai, coords, lookAheadBrake, "B", 1, red);

    drawLine(aiPosition, nextPositionThrottle, green);
    drawLine(aiPosition, nextPositionSteer,    blue);
    drawLine(aiPosition, nextPositionBrake,    red);

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

    float distanceThrottle = GET_DISTANCE_BETWEEN_COORDS(aiPosition, nextPositionThrottle);
    float distanceSteer = GET_DISTANCE_BETWEEN_COORDS(aiPosition, nextPositionSteer);
    float distanceBrake = GET_DISTANCE_BETWEEN_COORDS(aiPosition, nextPositionBrake);

    const float steerMult = 1.33;
    steer = constrain(turnSteer * steerMult, -limitRadians, limitRadians);

    // 145 degrees
    const float reverseAngle = 2.53073f;

    if (abs(turnThrottle) > reverseAngle) {
        if (turnThrottle > 0.0f) {
            steer = -constrain(3.1415f - turnThrottle, -limitRadians, limitRadians);
        }
        else {
            steer = -constrain(3.1415f + turnThrottle, -limitRadians, limitRadians);
        }
    }

    float aiSpeed = ENTITY::GET_ENTITY_SPEED(ai);

    throttle = map(aiSpeed, 0.0f, distanceThrottle, 2.0f, 0.0f);
    throttle = constrain(throttle, 0.0f, 1.0f);

    if (abs(turnThrottle) > reverseAngle) {
        throttle = -throttle;
    }

    float distPerpThrottle = (abs(turnThrottle) - 1.5708f) / 1.5708f;
    float distPerpSteer = (abs(turnSteer) - 1.5708f) / 1.5708f;
    float distPerpBrake = (abs(turnBrake) - 1.5708f) / 1.5708f;

    throttle *= map(abs(distPerpThrottle), 0.0f, 1.0f, 0.5f, 1.0f);

    handbrake = abs(turnSteer) > limitRadians * 2.0f && ENTITY::GET_ENTITY_SPEED_VECTOR(ai, true).y > 12.0f;

    float maxBrake = map(aiSpeed, distanceThrottle * 0.50f, distanceBrake * 0.75f, -0.3f, 3.0f);
    showText(0.4, 0.30, 0.5, "B: " + std::to_string(maxBrake));

    if (abs(turnBrake) > limitRadians && ENTITY::GET_ENTITY_SPEED_VECTOR(ai, true).y > 10.0f) {
        float brakeTurn = map(abs(distPerpBrake), 0.0f, 1.0f, 1.0f, 0.0f);
        if (brakeTurn > maxBrake) {
            maxBrake = brakeTurn;
            showText(0.75, 0.05, 0.4, "B: Angle");
        }
        else {
            showText(0.75, 0.05, 0.4, "B: Dist");
        }
    }
    
    brake = constrain(maxBrake, 0.0f, 1.0f);

    if (throttle < -0.3f)
        brake = 0.0f;

    if (brake > 0.7f)
        throttle = 0.0f;

    showText(0.4, 0.05, 0.5, "T: " + std::to_string(throttle));
    showText(0.4, 0.10, 0.5, "B: " + std::to_string(brake));
    showText(0.4, 0.15, 0.5, "S: " + std::to_string(steer));
    showText(0.4, 0.20, 0.5, "H: " + std::to_string(handbrake));
    showText(0.4, 0.25, 0.5, "Rad: " + std::to_string(turnThrottle));
    showText(0.5, 0.25, 0.5, "Deg: " + std::to_string(rad2deg(turnThrottle)));
}

void UpdateControl() {
    if (!VEHICLE::GET_IS_VEHICLE_ENGINE_RUNNING(vehicleToControl))
        VEHICLE::SET_VEHICLE_ENGINE_ON(vehicleToControl, true, true, true);

    float actualAngle = getSteeringAngle(vehicleToControl);
    float limitRadians = ext.GetMaxSteeringAngle(vehicleToControl);
    float reduction = CalculateReduction(vehicleToControl);

    bool handbrake = false;
    float throttle = 0.0f;
    float brake = 0.0f;
    float steer = 0.0f;

    if (playerInput) {
        GetControls(limitRadians, handbrake, throttle, brake, steer);
    }
    else {
        GetControlsFromAI(vehicleToControl, playerCoords, limitRadians, handbrake, throttle, brake, steer);
    }

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

    if (playerInput)
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
            auto blipIt = std::find_if(blips.begin(), blips.end(), [&](const auto& blip) {
                return blip.get()->GetEntity() == vehicleToControl;
            });
            if (blipIt != blips.end()) {
                (*blipIt).get()->Delete();
                blips.erase(blipIt);
            }
            else {
                showSubtitle("No blip for " + std::to_string(vehicleToControl));
            }
            ENTITY::SET_ENTITY_AS_MISSION_ENTITY(vehicleToControl, false, true);
            ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&vehicleToControl);
            vehicleToControl = 0;
        }
        else if (isVehicleAvailable(vehicle, playerPed) || amIInCar(vehicle, playerPed)) {
            vehicleToControl = vehicle;
            ENTITY::SET_ENTITY_AS_MISSION_ENTITY(vehicleToControl, true, false);
            showNotification("Controlling vehicle " + std::to_string(vehicle));
            PED::SET_PED_INTO_VEHICLE(playerPed, vehicleToControl, -2);
            blips.push_back(std::make_unique<BlipX>(vehicleToControl));
            auto blip = blips.back().get();
            blip->SetSprite(BlipSpritePersonalVehicleCar);
            blip->SetName("Remote Control Vehicle");
            blip->SetColor(eBlipColor::BlipColorYellow);
        }
    }
    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("pi"))) {
        playerInput = !playerInput;
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("startrecord"))) {
        playerCoords.clear();
        playerCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true));
        recording = true;
        showNotification("~g~Record started");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("stoprecord"))) {
        recording = false;
        showNotification("~r~Record stopped");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("clearrecord"))) {
        playerCoords.clear();
        playerCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true)); 
        recording = false;
        showNotification("~r~Record cleared");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("savetrack"))) {
        GAMEPLAY::DISPLAY_ONSCREEN_KEYBOARD(0, "FMMC_KEY_TIP8", "", "", "", "", "", 64);
        while (GAMEPLAY::UPDATE_ONSCREEN_KEYBOARD() == 0) WAIT(0);
        if (!GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT()) {
            showNotification("Cancelled save");
            return;
        }
        std::string saveFile = GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT();

        json j;
        for (int idx = 0; idx < playerCoords.size(); ++idx) {
            j[fmt("p%05d", idx)]["x"] = playerCoords[idx].x;
            j[fmt("p%05d", idx)]["y"] = playerCoords[idx].y;
            j[fmt("p%05d", idx)]["z"] = playerCoords[idx].z;
        }
        std::ofstream o("./DirectControl/" + saveFile + ".json");
        o << std::setw(4) << j << std::endl;

        showNotification("~g~Track saved");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("loadtrack"))) {
        GAMEPLAY::DISPLAY_ONSCREEN_KEYBOARD(0, "FMMC_KEY_TIP8", "", "", "", "", "", 64);
        while (GAMEPLAY::UPDATE_ONSCREEN_KEYBOARD() == 0) WAIT(0);
        if (!GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT()) {
            showNotification("Cancelled load");
            return;
        }
        std::string loadFile = GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT();

        json j;
        std::ifstream i("./DirectControl/" + loadFile + ".json");
        i >> j;

        playerCoords.clear();
        
        for (auto p : j) {
            Vector3 v;
            v.x = p["x"];
            v.y = p["y"];
            v.z = p["z"];

            playerCoords.push_back(v);
        }

        showNotification("~g~Track loaded");
    }

    if (recording) {
        Vector3 myCoords = ENTITY::GET_ENTITY_COORDS(playerPed, true);
        float mySpeed = ENTITY::GET_ENTITY_SPEED(playerPed);
        if (Distance(playerCoords.back(), myCoords) > 1.0f/*constrain(map(mySpeed, 1.0f, 20.0f, 1.0f, 2.0f), 0.5f, 20.0f)*/) {
            playerCoords.push_back(myCoords);
        }
    }

    for (int idx = 0; idx < playerCoords.size(); ++idx) {
        auto coord = playerCoords[idx];
        float screenX, screenY;
        bool visible = GRAPHICS::GET_SCREEN_COORD_FROM_WORLD_COORD(coord.x, coord.y, coord.z, &screenX, &screenY);
        if (visible && idx != playerCoords.size() - 1) {
            drawLine(coord, playerCoords[idx + 1], { 255, 255, 0, 255 });
        }
    }

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
