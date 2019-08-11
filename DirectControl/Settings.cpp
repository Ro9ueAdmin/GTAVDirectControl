#include "Settings.h"
#include <fstream>
#include <thirdparty/SimpleIni.h>
#include "Util/Logger.hpp"
#pragma warning( disable : 4244 )

Settings gSettings;

Settings::Settings() {
    SetOptimalDefaults();
}

void Settings::SetOptimalDefaults() {
    TrackShowDebug = true;
}

void Settings::ReadSettings(const char *file) {
    CSimpleIniA ini;
    ini.SetUnicode(true);
    SI_Error err = ini.LoadFile(file);

    if (err != SI_OK) {
        gLogger.Write(ERROR, "Error loading settings file, using defaults");
        SetOptimalDefaults();
        return;
    }

    const char* debugSection = "Debug";
    const char* paramSection = "AIParameters";
    const char* miscSection = "Misc";

    TrackShowDebug =                    ini.GetBoolValue(debugSection, "TrackShowDebug");
    
    gLogger.Write(INFO, "Load settings success");
}
