#include "script.h"

#include <fstream>
#include <iomanip>
#include <filesystem>
#include <thirdparty/json.hpp>

#include "Memory/VehicleExtensions.hpp"
#include "XInputControl.h"
#include "Blip.h"
#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"
#include "Util/StringFormat.h"
#include "Racer.h"
#include "PlayerRacer.h"

using json = nlohmann::json;

bool gRecording = false;

std::vector<Vector3> gTrackCoords;
std::vector<Racer> gRacers;
std::unique_ptr<PlayerRacer> gPlayerRacer(nullptr);

Vehicle spawnVehicle(Hash hash, Vector3 coords, float heading, DWORD timeout, bool managed) {
    if (!(STREAMING::IS_MODEL_IN_CDIMAGE(hash) && STREAMING::IS_MODEL_A_VEHICLE(hash))) {
        // Vehicle doesn't exist
        return 0;
    }
    STREAMING::REQUEST_MODEL(hash);
    DWORD startTime = GetTickCount();

    while (!STREAMING::HAS_MODEL_LOADED(hash)) {
        WAIT(0);
        if (GetTickCount() > startTime + timeout) {
            // Couldn't load model
            WAIT(0);
            STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(hash);
            return 0;
        }
    }

    Vehicle veh = VEHICLE::CREATE_VEHICLE(hash, coords.x, coords.y, coords.z, heading, 0, 1);
    Vehicle copy = veh;
    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(veh);

    if (!managed) {
        WAIT(0);
        STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(hash);
        ENTITY::SET_ENTITY_AS_MISSION_ENTITY(veh, false, true);
        ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&veh);
    }
    return copy;
}

void update(){
    Ped playerPed = PLAYER::PLAYER_PED_ID();
    Vehicle vehicle = PED::GET_VEHICLE_PED_IS_IN(playerPed, false);

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("cc"))) {
        if (gPlayerRacer) {
            showNotification("Stop controlling vehicle " + std::to_string(gPlayerRacer->GetVehicle()));
            gPlayerRacer.reset(nullptr);
        }
        else {
            if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
                showNotification("Controlling vehicle " + std::to_string(vehicle));
                gPlayerRacer = std::make_unique<PlayerRacer>(vehicle, 2);
            }
            else {
                showNotification("No vehicle to control");
            }
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("pp"))) {
        if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
            PED::SET_PED_INTO_VEHICLE(playerPed, vehicle, -2);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("dbgp"))) {
        if (gPlayerRacer) {
            showNotification("Toggling debug for " + std::to_string(gPlayerRacer->GetVehicle()));
            gPlayerRacer->SetDebugView(!gPlayerRacer->GetDebugView());
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("makeai0"))) {
        if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
            auto itFound = gRacers.end();
            for (auto it = gRacers.begin(); it != gRacers.end(); ++it) {
                if (it->GetVehicle() == vehicle) {
                    itFound = it;
                }
            }
            if (itFound != gRacers.end()) {
                gRacers.erase(itFound);
                showNotification(fmt("Remove AI from ~b~%s", VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(vehicle)));
            }
            else {
                showNotification("~r~No AI on this vehicle");
            }
        }
        else {
            showNotification("~r~Player not in vehicle");
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("makeai1"))) {
        if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
            bool found = false;
            for (auto &gRacer : gRacers) {
                if (gRacer.GetVehicle() == vehicle) {
                    found = true;
                }
            }
            if (!found) {
                gRacers.emplace_back(vehicle);
                showNotification(fmt("Adding AI to ~b~%s", VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(vehicle)));
            }
            else {
                showNotification("~r~AI already on this vehicle");
            }
        }
        else {
            showNotification("~r~Player not in vehicle");
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("addai"))) {
        showNotification("Enter number of AIs to add");
        GAMEPLAY::DISPLAY_ONSCREEN_KEYBOARD(0, "FMMC_KEY_TIP8", "", "", "", "", "", 64);
        while (GAMEPLAY::UPDATE_ONSCREEN_KEYBOARD() == 0) WAIT(0);
        if (!GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT()) {
            showNotification("Cancelled AI Spawn");
            return;
        }

        std::string numRacersString = GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT();
        if (numRacersString.empty())
            numRacersString = "1";
        int numRacers;
        try {
            numRacers = std::stoi(numRacersString);
        }
        catch (const std::invalid_argument&) {
            numRacers = 1;
        }
        showNotification("Enter AI model (empty = kuruma)");
        GAMEPLAY::DISPLAY_ONSCREEN_KEYBOARD(0, "FMMC_KEY_TIP8", "", "", "", "", "", 64);
        while (GAMEPLAY::UPDATE_ONSCREEN_KEYBOARD() == 0) WAIT(0);
        if (!GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT()) {
            showNotification("Cancelled AI Spawn");
            return;
        }

        std::string racerModelName = GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT();
        if (racerModelName.empty())
            racerModelName = "kuruma";
        
        Hash model = GAMEPLAY::GET_HASH_KEY((char*)racerModelName.c_str());
        if (!STREAMING::IS_MODEL_IN_CDIMAGE(model)) {
            showNotification("Model not valid: Cancelling AI Spawn");
            return;
        }

        for (auto i = 0; i < numRacers; ++i) {
            float offsetX = 0.0f;
            Vector3 modelDimMin, modelDimMax, myDimMin, myDimMax;
            GAMEPLAY::GET_MODEL_DIMENSIONS(model, &modelDimMin, &modelDimMax);

            if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
                Hash myModel = ENTITY::GET_ENTITY_MODEL(vehicle);
                GAMEPLAY::GET_MODEL_DIMENSIONS(myModel, &myDimMin, &myDimMax);
            }
            else {
                myDimMin = modelDimMin;
                myDimMax = modelDimMax;
            }

            // to the right
            // my width + ( margin + spawn width ) * iteration
            offsetX = ((myDimMax.x - myDimMin.x) / 2.0f) + (1.0f + ((modelDimMax.x - modelDimMin.x) / 2.0f)) * (float)(i + 1);

            Vector3 spawnPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(playerPed, offsetX, 0.0, 0);
            Vehicle spawnedVehicle = spawnVehicle(model, spawnPos, ENTITY::GET_ENTITY_HEADING(playerPed), 1000, true);
            gRacers.emplace_back(spawnedVehicle);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("startai"))) {
        for (auto& racer : gRacers) {
            racer.SetActive(true);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("stopai"))) {
        for (auto& racer : gRacers) {
            racer.SetActive(false);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("fixai"))) {
        for (auto& racer : gRacers) {
            Vehicle v = racer.GetVehicle();
            VEHICLE::SET_VEHICLE_FIXED(v);
            VEHICLE::SET_VEHICLE_DEFORMATION_FIXED(v);
            // TODO: Use vehicle broken flag?
            //auto address = getScriptHandleBaseAddress(vehicle); 
            //*(BYTE*)(address + 0xD8) &= ~7;
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("delai"))) {
        gRacers.clear();
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("dbgai0"))) {
        showNotification("Disable debug view for racers");
        for (auto& racer : gRacers) {
            racer.SetDebugView(false);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("dbgai1"))) {
        showNotification("Enable debug view for racers");
        for (auto& racer : gRacers) {
            racer.SetDebugView(true);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("startrecord"))) {
        gTrackCoords.clear();
        gTrackCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true));
        gRecording = true;
        showNotification("~g~Record started");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("stoprecord"))) {
        gRecording = false;
        showNotification("~r~Record stopped");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("cleartrack"))) {
        gTrackCoords.clear();
        gTrackCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true)); 
        gRecording = false;
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
        for (int idx = 0; idx < gTrackCoords.size(); ++idx) {
            j[fmt("p%05d", idx)]["x"] = gTrackCoords[idx].x;
            j[fmt("p%05d", idx)]["y"] = gTrackCoords[idx].y;
            j[fmt("p%05d", idx)]["z"] = gTrackCoords[idx].z;
        }
        std::ofstream o("./DirectControl/" + saveFile + ".json");
        o << std::setw(4) << j << std::endl;

        showNotification("~g~Track saved");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("loadtrack"))) {
        std::string path = "./DirectControl";
        for (auto & p : std::filesystem::directory_iterator(path)) {
            if (p.path().extension() == ".json")
                showNotification(fmt("Track: ~b~%s", p.path().stem().string().c_str()));
        }

        GAMEPLAY::DISPLAY_ONSCREEN_KEYBOARD(0, "FMMC_KEY_TIP8", "", "", "", "", "", 64);
        while (GAMEPLAY::UPDATE_ONSCREEN_KEYBOARD() == 0) WAIT(0);
        if (!GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT()) {
            showNotification("Cancelled load");
            return;
        }
        std::string loadFile = GAMEPLAY::GET_ONSCREEN_KEYBOARD_RESULT();

        json j;
        std::ifstream i("./DirectControl/" + loadFile + ".json");
        if (!i.is_open()) {
            showNotification("Couldn't find file");
            return;
        }
        i >> j;

        gTrackCoords.clear();
        
        for (auto p : j) {
            Vector3 v;
            v.x = p["x"];
            v.y = p["y"];
            v.z = p["z"];

            gTrackCoords.push_back(v);
        }

        showNotification("~g~Track loaded");
    }

    if (gRecording) {
        Vector3 myCoords = ENTITY::GET_ENTITY_COORDS(playerPed, true);
        float mySpeed = ENTITY::GET_ENTITY_SPEED(playerPed);
        if (Distance(gTrackCoords.back(), myCoords) > 1.0f/*constrain(map(mySpeed, 1.0f, 20.0f, 1.0f, 2.0f), 0.5f, 20.0f)*/) {
            gTrackCoords.push_back(myCoords);
        }
    }

    for (int idx = 0; idx < gTrackCoords.size(); ++idx) {
        auto coord = gTrackCoords[idx];
        float screenX, screenY;
        bool visible = GRAPHICS::GET_SCREEN_COORD_FROM_WORLD_COORD(coord.x, coord.y, coord.z, &screenX, &screenY);
        if (visible && idx != gTrackCoords.size() - 1) {
            drawLine(coord, gTrackCoords[idx + 1], { 255, 255, 0, 255 });
        }
    }

    if (gPlayerRacer != nullptr && VEHICLE::GET_PED_IN_VEHICLE_SEAT(gPlayerRacer->GetVehicle(), -1) != playerPed) {
        gPlayerRacer->UpdateControl();
    }

    for (auto& racer : gRacers) {
        racer.UpdateControl(gTrackCoords);
    }
}

void main() {
    logger.Write(INFO, "Direct vehicle control test script");
    logger.Write(INFO, "Direct Control Cheats:");
    logger.Write(INFO, "\"cc\" to take control of a non-controlled car");
    logger.Write(INFO, "\"cc\" to release control of a controlled car");
    logger.Write(INFO, "\"pp\" to switch to passenger seat ");
    logger.Write(INFO, "\"ddp\" to toggle debug lines");
    logger.Write(INFO, "Direct Control Controls:");
    logger.Write(INFO, "IJKL as WASD, U is reverse, O is handbrake");
    logger.Write(INFO, "Gamepad #2 for analog input. Hold LB to reverse with throttle.");
    logger.Write(INFO, "--------------------------------------------------------------------------------");
    logger.Write(INFO, "AI Cheats:");
    logger.Write(INFO, "\"addai\" to add ai cars. Follow onscreen instructions. Spawn to your right!");
    logger.Write(INFO, "\"delai\" to remove AI cars (release to game)");
    logger.Write(INFO, "\"startai\" to make AI follow current track (default is already started)");
    logger.Write(INFO, "\"stopai\" to make AI stop following track");
    logger.Write(INFO, "\"fixai\" to fix all AI cars");
    logger.Write(INFO, "\"ddai0\" to disable debug lines");
    logger.Write(INFO, "\"ddai1\" to enable debug lines");
    logger.Write(INFO, "--------------------------------------------------------------------------------");
    logger.Write(INFO, "Track Cheats:");
    logger.Write(INFO, "\"startrecord\" to start recording current location. 1m interval");
    logger.Write(INFO, "\"stoprecord\" to stop recording current location");
    logger.Write(INFO, "\"loadtrack\" to load track from file (json)");
    logger.Write(INFO, "\"savetrack\" to save current track to file (json)");
    logger.Write(INFO, "\"cleartrack\" to clear all points on current track");
    logger.Write(INFO, "");

    gExt.initOffsets();

    while (true) {
		update();
		WAIT(0);
	}
}

void ScriptMain() {
	srand(GetTickCount());
	main();
}
