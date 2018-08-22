#include "script.h"

#include <fstream>
#include <iomanip>
#include <thirdparty/json.hpp>

#include "Memory/VehicleExtensions.hpp"
#include "XInputControl.h"
#include "Blip.h"

#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"
#include "Util/StringFormat.h"
#include "Racer.h"

// for convenience
using json = nlohmann::json;

Player player;
Ped playerPed;

VehicleExtensions ext;

std::vector<Vector3> gTrackCoords;
bool recording = false;

std::vector<Racer> gRacers;
std::unique_ptr<PlayerRacer> gPlayerRacer(nullptr);

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

void update(){
    player = PLAYER::PLAYER_ID();
    playerPed = PLAYER::PLAYER_PED_ID();
    Vehicle vehicle = PED::GET_VEHICLE_PED_IS_IN(playerPed, false);

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("cc"))) {
        if (gPlayerRacer) {
            showNotification("Stop controlling vehicle " + std::to_string(gPlayerRacer->GetVehicle()));
            gPlayerRacer.reset(nullptr);
        }
        else {
            if (ENTITY::DOES_ENTITY_EXIST(vehicle)) {
                showNotification("Controlling vehicle " + std::to_string(vehicle));
                gPlayerRacer = std::make_unique<PlayerRacer>(vehicle, ext, 2);
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

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("ddp"))) {
        if (gPlayerRacer) {
            showNotification("Toggling debug for " + std::to_string(gPlayerRacer->GetVehicle()));
            gPlayerRacer->SetDebugView(!gPlayerRacer->GetDebugView());
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("ddai0"))) {
        showNotification("Disable debug view for racers");
        for (auto& racer : gRacers) {
            racer.SetDebugView(false);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("ddai1"))) {
        showNotification("Enable debug view for racers");
        for (auto& racer : gRacers) {
            racer.SetDebugView(true);
        }
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("startrecord"))) {
        gTrackCoords.clear();
        gTrackCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true));
        recording = true;
        showNotification("~g~Record started");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("stoprecord"))) {
        recording = false;
        showNotification("~r~Record stopped");
    }

    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(GAMEPLAY::GET_HASH_KEY("clearrecord"))) {
        gTrackCoords.clear();
        gTrackCoords.push_back(ENTITY::GET_ENTITY_COORDS(playerPed, true)); 
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

    if (recording) {
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
    logger.Write(INFO, "Use the \"cc\" cheat while in a car to take control");
    logger.Write(INFO, "IJKL as WASD, U is reverse, O is handbrake");
    logger.Write(INFO, "Use the \"cc\" cheat again to remove control");

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
