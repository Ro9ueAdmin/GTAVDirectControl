#include "script.h"

#include <filesystem>
#include <thirdparty/json.hpp>

#include "Memory/VehicleExtensions.hpp"
#include "XInputControl.h"
#include "Blip.h"
#include "Util/Logger.hpp"
#include "Util/UIUtils.h"
#include "Util/MathExt.h"

#include "Racer.h"
#include "PlayerRacer.h"
#include "Settings.h"
#include "Track.h"
#include "Cheats.h"
#include "Session.h"

using json = nlohmann::json;

bool gRecording = false;

std::vector<std::unique_ptr<Racer>> gRacers;
std::unique_ptr<PlayerRacer> gPlayerRacer(nullptr);

template <typename F, typename ... Args>
void CheckCheat(const std::string& cheat, F func, Args...args) {
    Hash hash = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(cheat.c_str()));
    if (GAMEPLAY::_HAS_CHEAT_STRING_JUST_BEEN_ENTERED(hash)) {
        func(args...);
    }
}

void UpdatePlayer() {
    Ped playerPed = PLAYER::PLAYER_PED_ID();

    if (gPlayerRacer != nullptr && VEHICLE::GET_PED_IN_VEHICLE_SEAT(gPlayerRacer->GetVehicle(), -1) != playerPed) {
        gPlayerRacer->UpdateControl();
    }

    if (gRecording) {
        Vector3 myCoords = ENTITY::GET_ENTITY_COORDS(playerPed, true);
        
        if (Distance(Recorder::Get().Points().back().v, myCoords) > 1.0f) {
            Recorder::Get().Append(Point(myCoords, 5.0f));
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

    auto& trackCoords = Session::Get().GetTrack().Points();
    for (auto& racer : gRacers) {
        racer->UpdateControl(trackCoords, npcs);
    }    

    if (gRecording) {
        DrawDebugTrack(Recorder::Get().Points());
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
    CheckCheat("dbgaitxt", Cheats::DebugAiTxtToggle);
    CheckCheat("dbgthis", Cheats::DebugThis, vehicle);

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
    gPlayerRacer.reset();
}

void CheatMain() {
    srand(GetTickCount());

    while(true) {
        UpdateCheats();
        WAIT(0);
    }
}
