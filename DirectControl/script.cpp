#include "script.h"

#include <filesystem>
#include "json.hpp"

#include "Memory/VehicleExtensions.hpp"
#include "XInputControl.h"
#include "Blip.h"
#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"

#include "Racer.h"
#include "PlayerDirectInput.h"
#include "Settings.h"
#include "Track.h"
#include "Cheats.h"
#include "Session.h"
#include "Player.h"
#include "Util/StringFormat.h"

using json = nlohmann::json;

bool gRecording = false;

std::vector<std::unique_ptr<Racer>> gRacers;
std::unique_ptr<PlayerRacer> gPlayerRacer;
std::unique_ptr<PlayerDirectInput> gPlayerDirectInput(nullptr);
Vehicle gPrevVehicle = 0;

template <typename F, typename ... Args>
void CheckCheat(const std::string& cheat, F func, Args...args) {
    Hash hash = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(cheat.c_str()));
    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(hash)) {
        func(args...);
    }
}

void UpdatePlayer() {
    Ped playerPed = PLAYER::PLAYER_PED_ID();
    Vehicle vehicle = PED::GET_VEHICLE_PED_IS_IN(playerPed, false);

    if (vehicle != gPrevVehicle &&
        ENTITY::DOES_ENTITY_EXIST(vehicle) && 
        playerPed == VEHICLE::GET_PED_IN_VEHICLE_SEAT(vehicle, -1)) {
        gPlayerRacer.reset();
        gPlayerRacer = std::make_unique<PlayerRacer>(vehicle);
        gPlayerRacer->SetTrack(Session::Get().GetTrack());

        std::string name = getGxtName(ENTITY::GET_ENTITY_MODEL(vehicle));
        std::string plate = VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(vehicle);
        std::string msg = "Changed vehicle!";
        showNotification(fmt("Player ~b~%s (~r~%s~b~)\n~w~%s",
            name.c_str(), plate.c_str(), msg.c_str()),
            nullptr);
    }
    else if (vehicle != gPrevVehicle &&
        !ENTITY::DOES_ENTITY_EXIST(vehicle)) {
        gPlayerRacer.reset();
        std::string msg = "Exit vehicle!";
        showNotification(fmt("Player ~b~\n~w~%s",
            msg.c_str()),
            nullptr);
    }
    gPrevVehicle = vehicle;

    if (gPlayerDirectInput != nullptr && VEHICLE::GET_PED_IN_VEHICLE_SEAT(gPlayerDirectInput->GetVehicle(), -1) != playerPed) {
        gPlayerDirectInput->UpdateControl();
    }

    if (gPlayerRacer) {
        gPlayerRacer->UpdateControl();
    }

    if (gRecording) {
        Vector3 myCoords = ENTITY::GET_ENTITY_COORDS(playerPed, true);
        
        if (Distance(Recorder::Get().Points().back().v, myCoords) > 1.0f) {
            Recorder::Get().Append(Point(myCoords, 5.0f));
        }
    }

    if (PLAYER::IS_PLAYER_DEAD(PLAYER::GET_PLAYER_INDEX()) || 
        PLAYER::IS_PLAYER_BEING_ARRESTED(PLAYER::GET_PLAYER_INDEX(), false)) {
        Cheats::StartAi(false);
        if (gRecording) {
            Cheats::RecordTrack(playerPed, false);
        }
    }
}

void DrawDebugTrack(const std::vector<Point>& trackCoords) {
    if (!trackCoords.empty()) {
        drawSphere(trackCoords[0].v, 0.125f, { 255, 255, 0, 255 });
    }

    for (size_t idx = 0; idx < trackCoords.size(); ++idx) {
        auto coord = trackCoords[idx];
        float screenX, screenY;
        bool visibleC = GRAPHICS::GET_SCREEN_COORD_FROM_WORLD_COORD(coord.v.x, coord.v.y, coord.v.z, &screenX, &screenY);

        if (visibleC) {
            Vector3 a = trackCoords[idx].v;
            Vector3 b = trackCoords[(idx + 1) % trackCoords.size()].v;
            Vector3 c = trackCoords[(idx + 2) % trackCoords.size()].v;

            Vector3 cwA = GetPerpendicular(a, b, coord.w, true);
            Vector3 ccwA = GetPerpendicular(a, b, coord.w, false);

            Vector3 cwB = GetPerpendicular(b, c, coord.w, true);
            Vector3 ccwB = GetPerpendicular(b, c, coord.w, false);

            drawLine(a, b, { 255, 255, 0, 255 });
            drawLine(cwA, cwB, { 255, 255, 0, 255 });
            drawLine(ccwA, ccwB, { 255, 255, 0, 255 });
        }
    }
}

void UpdateAI(){
    std::vector<Vehicle> npcs(1024);
    int npcCount = worldGetAllVehicles(npcs.data(), 1024);
    npcs.resize(npcCount);

    auto& track = Session::Get().GetTrack();
    auto& trackCoords = track.Points();
    Track recordTrack{};
    if (gRecording)
        recordTrack = Recorder::Get().GetTrack();

    for (auto& racer : gRacers) {
        if (gRecording) {
            racer->SetTrack(recordTrack);
        }
        racer->UpdateControl(npcs);
    }

    if (gRecording) {
        DrawDebugTrack(recordTrack.Points());
    }
    else if (gSettings.TrackShowDebug) {
        DrawDebugTrack(trackCoords);
    }
}

void UpdateCheats() {
    Ped playerPed = PLAYER::PLAYER_PED_ID();
    Vehicle vehicle = PED::GET_VEHICLE_PED_IS_IN(playerPed, false);

    CheckCheat("cc", Cheats::ControlCar, vehicle);
    CheckCheat("pp", Cheats::Passenger, vehicle, playerPed);
    CheckCheat("ep", Cheats::EnterPassenger, playerPed);

    CheckCheat("dbgp", Cheats::DebugPlayer);

    CheckCheat("makeai0", Cheats::MakeAi, vehicle, false);
    CheckCheat("makeai1", Cheats::MakeAi, vehicle, true);
    // Spawned right relative to player car/ped
    CheckCheat("addai", Cheats::AddAi, vehicle, playerPed);
    CheckCheat("startai", Cheats::StartAi, true);
    CheckCheat("stopai", Cheats::StartAi, false);
    CheckCheat("fixai", Cheats::FixAi);
    CheckCheat("delai", Cheats::DelAi);
    
    CheckCheat("dbgai0", Cheats::DebugAiVis, false);
    CheckCheat("dbgai1", Cheats::DebugAiVis, true);
    CheckCheat("dbgtxt0", Cheats::DebugAiTxt, false);
    CheckCheat("dbgtxt1", Cheats::DebugAiTxt, true);
    CheckCheat("dbgthis", Cheats::DebugThis, vehicle);
    CheckCheat("loadconfig", Cheats::LoadConfig, vehicle);

    // playerPed used for coords
    CheckCheat("startrecord", Cheats::RecordTrack, playerPed, true);
    CheckCheat("stoprecord", Cheats::RecordTrack, playerPed, false);
    CheckCheat("cleartrack", Cheats::ClearTrack, playerPed);
    CheckCheat("savetrack", Cheats::SaveTrack);
    CheckCheat("loadtrack", Cheats::LoadTrack);
    CheckCheat("reversetrack", Cheats::ReverseTrack);
    CheckCheat("dbgtrack", Cheats::DebugTrack);

    CheckCheat("reloadsettings", Cheats::ReloadSettings);
}

void main() {
    gLogger.Write(INFO, "Direct vehicle control test script");
    gLogger.Write(INFO, "Direct Control Cheats: Refer to the GitHub page");
    gLogger.Write(INFO, "");

    gSettings.ReadSettings("./DirectControl/settings.ini");

    gExt.initOffsets();

    while (true) {
        UpdatePlayer();
        UpdateAI();
        WAIT(0);
    }
}

void ScriptMain() {
    srand(GetTickCount());
    main();
}

void ScriptExit() {
    gRacers.clear();
    gPlayerDirectInput.reset();
}

void CheatMain() {
    srand(GetTickCount());

    while(true) {
        UpdateCheats();
        WAIT(0);
    }
}
