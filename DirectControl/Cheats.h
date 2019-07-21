#pragma once
#include "script.h"

namespace Cheats {
    void ControlCar(Vehicle vehicle);
    void Passenger(Vehicle vehicle, Ped playerPed);
    void DebugPlayer();

    void MakeAi(Vehicle vehicle, bool enableAi);
    void AddAi(Vehicle vehicle, Ped playerPed);
    void StartAi(bool start);
    void FixAi();
    void DelAi();
    void DebugAiVis(bool show);
    void DebugAiTxtToggle();
    void DebugThis(Vehicle vehicle);

    void RecordTrack(Ped playerPed, bool start);
    void ClearTrack(Ped playerPed);
    void SaveTrack();
    void LoadTrack();
    void ReverseTrack();
    void DebugTrack();

    void ReloadSettings();
}
