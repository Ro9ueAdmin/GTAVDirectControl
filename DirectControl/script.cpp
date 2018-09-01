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
#include "Settings.h"

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

void UpdateAI(){
    Ped playerPed = PLAYER::PLAYER_PED_ID();

    std::vector<Vehicle> npcs(1024);
    int npcCount = worldGetAllVehicles(npcs.data(), 1024);

    if (gPlayerRacer != nullptr && VEHICLE::GET_PED_IN_VEHICLE_SEAT(gPlayerRacer->GetVehicle(), -1) != playerPed) {
        gPlayerRacer->UpdateControl();
    }

    for (auto& racer : gRacers) {
        racer.UpdateControl(gTrackCoords, npcs);
    }

    if (gRecording) {
        Vector3 myCoords = ENTITY::GET_ENTITY_COORDS(playerPed, true);
        float mySpeed = ENTITY::GET_ENTITY_SPEED(playerPed);
        if (Distance(gTrackCoords.back(), myCoords) > 1.0f/*constrain(map(mySpeed, 1.0f, 20.0f, 1.0f, 2.0f), 0.5f, 20.0f)*/) {
            gTrackCoords.push_back(myCoords);
        }
    }

    if (gSettings.TrackShowDebug || gRecording) {
        if (gTrackCoords.size()) {
            drawSphere(gTrackCoords[0], 0.125f, { 255, 255, 0, 255 });
        }

        for (int idx = 0; idx < gTrackCoords.size(); ++idx) {
            auto coord = gTrackCoords[idx];
            float screenX, screenY;
            bool visible = GRAPHICS::GET_SCREEN_COORD_FROM_WORLD_COORD(coord.x, coord.y, coord.z, &screenX, &screenY);
            if (visible && idx != gTrackCoords.size() - 1) {
                drawLine(coord, gTrackCoords[idx + 1], { 255, 255, 0, 255 });
            }
        }
    }
}

void UpdateCheats() {
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
            if (VEHICLE::GET_VEHICLE_MODEL_NUMBER_OF_SEATS(ENTITY::GET_ENTITY_MODEL(vehicle)) > 1) {
                if (VEHICLE::GET_PED_IN_VEHICLE_SEAT(vehicle, -1) == playerPed) {
                    showNotification("Player to Passenger seat");
                    PED::SET_PED_INTO_VEHICLE(playerPed, vehicle, -2);
                }
                else {                    
                    showNotification("Player to Driver seat");
                    PED::SET_PED_INTO_VEHICLE(playerPed, vehicle, -1);
                }
            }
            else {                
                showNotification("Vehicle has only one seat");
            }
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
            offsetX = ((myDimMax.x - myDimMin.x) / 2.0f) + (2.0f + ((modelDimMax.x - modelDimMin.x) / 2.0f)) * (float)(i + 1);

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

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("dbgaitxt"))) {
        showNotification("Toggle debug text for racers");
        gSettings.AIShowDebugText = !gSettings.AIShowDebugText;
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("dbgthis"))) {
        for (auto& racer : gRacers) {
            if (racer.GetVehicle() == vehicle) {
                showNotification("Toggle debug for current");
                racer.SetDebugView(!racer.GetDebugView());
                break;
            }
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
            j["Data"]["Route"]["Point"].push_back({
                { "X", gTrackCoords[idx].x },
                { "Y", gTrackCoords[idx].y },
                { "Z", gTrackCoords[idx].z },
                { "Wide", 5 },
                });
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

        for (auto& p : j["Data"]["Route"]["Point"]) {
            Vector3 v;

            if (p["X"].is_string()) {
                v.x = static_cast<float>(std::atof(p["X"].get<std::string>().c_str()));
                v.y = static_cast<float>(std::atof(p["Y"].get<std::string>().c_str()));
                v.z = static_cast<float>(std::atof(p["Z"].get<std::string>().c_str()));
            }
            else {
                v.x = p["X"];
                v.y = p["Y"];
                v.z = p["Z"];
            }

            gTrackCoords.push_back(v);
        }
        float avgDst = 0.0f;
        for (auto i = 0; i < gTrackCoords.size(); ++i) {
            avgDst += Distance(gTrackCoords[i], gTrackCoords[(i + 1) % gTrackCoords.size()]);
        }
        avgDst /= (float)gTrackCoords.size();
        showNotification(fmt("~g~Track loaded, %d nodes, %.03f average node distance", gTrackCoords.size(), avgDst));
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("drawtrack"))) {
        gSettings.TrackShowDebug = !gSettings.TrackShowDebug;
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("reloadsettings"))) {
        gSettings.ReadSettings("./DirectControl/settings.ini");
        showNotification("Reloaded settings");
    }
}

void main() {
    gLogger.Write(INFO, "Direct vehicle control test script");
    gLogger.Write(INFO, "Direct Control Cheats:");
    gLogger.Write(INFO, "\"cc\" to take control of a non-controlled car");
    gLogger.Write(INFO, "\"cc\" to release control of a controlled car");
    gLogger.Write(INFO, "\"pp\" to switch to passenger seat ");
    gLogger.Write(INFO, "\"dbgp\" to toggle debug lines");
    gLogger.Write(INFO, "Direct Control Controls:");
    gLogger.Write(INFO, "IJKL as WASD, U is reverse, O is handbrake");
    gLogger.Write(INFO, "Gamepad #2 for analog input. Hold LB to reverse with throttle.");
    gLogger.Write(INFO, "--------------------------------------------------------------------------------");
    gLogger.Write(INFO, "AI Cheats:");
    gLogger.Write(INFO, "\"addai\" to add ai cars. Follow onscreen instructions. Spawn to your right!");
    gLogger.Write(INFO, "\"delai\" to remove AI cars (release to game)");
    gLogger.Write(INFO, "\"startai\" to make AI follow current track (default is already started)");
    gLogger.Write(INFO, "\"stopai\" to make AI stop following track");
    gLogger.Write(INFO, "\"fixai\" to fix all AI cars");
    gLogger.Write(INFO, "\"dbgai0\" to disable debug lines");
    gLogger.Write(INFO, "\"dbgai1\" to enable debug lines");
    gLogger.Write(INFO, "--------------------------------------------------------------------------------");
    gLogger.Write(INFO, "Track Cheats:");
    gLogger.Write(INFO, "\"startrecord\" to start recording current location. 1m interval");
    gLogger.Write(INFO, "\"stoprecord\" to stop recording current location");
    gLogger.Write(INFO, "\"loadtrack\" to load track from file (json)");
    gLogger.Write(INFO, "\"savetrack\" to save current track to file (json)");
    gLogger.Write(INFO, "\"cleartrack\" to clear all points on current track");
    gLogger.Write(INFO, "\"drawtrack\" to toggle track drawing");
    gLogger.Write(INFO, "");

    gSettings.WriteDefaults("./DirectControl/defaults.ini");
    gSettings.ReadSettings("./DirectControl/settings.ini");

    gExt.initOffsets();

    while (true) {
        UpdateAI();
        WAIT(0);
    }
}

void ScriptMain() {
	srand(GetTickCount());
	main();
}

void CheatMain() {
    srand(GetTickCount());

    while(true) {
        UpdateCheats();
        WAIT(0);
    }
}
