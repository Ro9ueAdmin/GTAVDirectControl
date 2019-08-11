#include "RacerConfig.h"
#include "Util/Logger.hpp"
#include <thirdparty/SimpleIni.h>
#include <filesystem>

/**
 * These defaults are suitable for Killatomate's AWD sports vehicles, such as the
 * Sultan or Kuruma. Similar vehicles should also perform well.
 */
RacerConfig::RacerConfig()
    : DefaultActive(true)
    , AutoRepair(true)
    , LookaheadThrottleSpeedMult(3.5f)
    , LookaheadThrottleMinDistance(25.0f)
    , LookaheadBrakeSpeedMult(2.5f)
    , LookaheadBrakeMinDistance(20.0f)
    , LookaheadSteerSpeedMult(1.0f)
    , LookaheadSteerMinDistance(15.0f)
    , SteerMult(2.0f)
    , CountersteerIncreaseStartAngle(5.0f)
    , CountersteerIncreaseEndAngle(45.0f)
    , ThrottleDecreaseStartAngle(30.0f)
    , ThrottleDecreaseEndAngle(90.0f)
    , OversteerDetectionAngle(3.0f)
    , SteerLookAheadPitch(-30.0f)
    , UndersteerHandbrakeTrigger(1.20f)
    , BrakePointDistanceThrottleMult(0.75f)
    , BrakePointDistanceBrakeMult(0.60f)
    , BrakePointHeadingMinAngle(30.0f)
    , BrakePointHeadingMaxAngle(52.5f)
    , BrakePointHeadingMinSpeed(10.0f)
    , BrakePointHeadingMaxSpeed(40.0f)
    , BrakePointRadiusMaxSpeed(100.0f)
    , BrakePointRadiusMaxRadius(1000.0f)
    , ElevationDropThreshold(0.5f)
    , ElevationMin(0.0f)
    , ElevationMax(1.75f)
    , ElevationDangerMin(1.00f)
    , ElevationDangerMax(1.35f)
    , TrackLimitsAdjustMinOvershoot(-1.0f)
    , TrackLimitsAdjustMaxOvershoot(-0.1f)
    , TrackLimitsThrottleMultMinOvershoot(1.0f)
    , TrackLimitsThrottleMultMaxOvershoot(0.0f)
    , TrackLimitsSteerMultMinOvershoot(1.0f)
    , TrackLimitsSteerMultMaxOvershoot(2.0f)
    , ShowDebug(false)
    , ShowDebugText(false) { }

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

std::string getFullPath(const std::string& name) {
    std::string pathPrefix = "./DirectControl/RacerConfigs"; // TODO: Move somewhere neat.
    std::string filename;
    for (auto& p : std::filesystem::directory_iterator(pathPrefix)) {
        if (p.path().extension() == ".ini" && toLower(p.path().stem().string()).find(toLower(name)) != std::string::npos)
            filename = p.path().string();
    }
    return filename;
}

RacerConfig RacerConfig::Parse(const std::string& name) {
    std::string fullPath = getFullPath(name);

    if (fullPath.empty()) {
        gLogger.Write(ERROR, "Error loading [%s], loading defaults", name.c_str());
        fullPath = getFullPath("Default");
        if (fullPath.empty()) {
            gLogger.Write(ERROR, "Error loading default config, loading hardcoded defaults", name.c_str());
            return RacerConfig();
        }
    }

    CSimpleIniA ini;
    ini.SetUnicode(true);
    SI_Error err = ini.LoadFile(fullPath.c_str());

    if (err != SI_OK) {
        gLogger.Write(ERROR, "Error loading [%s], using defaults", fullPath.c_str());
        return RacerConfig();
    }

    const char* sectionGeneral = "General";
    const char* sectionLookahead = "Lookahead";
    const char* sectionSteering = "Steering";
    const char* sectionBraking = "Braking";
    const char* sectionElevation = "Elevation";
    const char* sectionTrackLimits = "TrackLimits";
    const char* sectionDebug = "Debug";

    RacerConfig cfg{};

    // General section
    cfg.AutoRepair =    ini.GetBoolValue(sectionGeneral, "AutoRepair");
    cfg.DefaultActive = ini.GetBoolValue(sectionGeneral, "DefaultActive");

    // Lookahead
    cfg.LookaheadThrottleSpeedMult =      ini.GetDoubleValue(sectionLookahead, "LookaheadThrottleSpeedMult");
    cfg.LookaheadThrottleMinDistance =    ini.GetDoubleValue(sectionLookahead, "LookaheadThrottleMinDistance");
    cfg.LookaheadBrakeSpeedMult =         ini.GetDoubleValue(sectionLookahead, "LookaheadBrakeSpeedMult");
    cfg.LookaheadBrakeMinDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadBrakeMinDistance");
    cfg.LookaheadSteerSpeedMult =         ini.GetDoubleValue(sectionLookahead, "LookaheadSteerSpeedMult");
    cfg.LookaheadSteerMinDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadSteerMinDistance");

    // Steering
    cfg.SteerMult =                       ini.GetDoubleValue(sectionSteering, "SteerMult");
    cfg.CountersteerIncreaseStartAngle =  ini.GetDoubleValue(sectionSteering, "CountersteerIncreaseStartAngle");
    cfg.CountersteerIncreaseEndAngle =    ini.GetDoubleValue(sectionSteering, "CountersteerIncreaseEndAngle");
    cfg.ThrottleDecreaseStartAngle =      ini.GetDoubleValue(sectionSteering, "ThrottleDecreaseStartAngle");
    cfg.ThrottleDecreaseEndAngle =        ini.GetDoubleValue(sectionSteering, "ThrottleDecreaseEndAngle");
    cfg.OversteerDetectionAngle =         ini.GetDoubleValue(sectionSteering, "OversteerDetectionAngle");
    cfg.UndersteerHandbrakeTrigger =      ini.GetDoubleValue(sectionSteering, "UndersteerHandbrakeTrigger");

    // Braking
    cfg.BrakePointDistanceThrottleMult =  ini.GetDoubleValue(sectionBraking, "BrakePointDistanceThrottleMult");
    cfg.BrakePointDistanceBrakeMult =     ini.GetDoubleValue(sectionBraking, "BrakePointDistanceBrakeMult");
    cfg.BrakePointHeadingMinAngle =       ini.GetDoubleValue(sectionBraking, "BrakePointHeadingMinAngle");
    cfg.BrakePointHeadingMaxAngle =       ini.GetDoubleValue(sectionBraking, "BrakePointHeadingMaxAngle");
    cfg.BrakePointHeadingMinSpeed =       ini.GetDoubleValue(sectionBraking, "BrakePointHeadingMinSpeed");
    cfg.BrakePointHeadingMaxSpeed =       ini.GetDoubleValue(sectionBraking, "BrakePointHeadingMaxSpeed");
    cfg.BrakePointRadiusMaxSpeed =        ini.GetDoubleValue(sectionBraking, "BrakePointRadiusMaxSpeed");
    cfg.BrakePointRadiusMaxRadius =       ini.GetDoubleValue(sectionBraking, "BrakePointRadiusMaxRadius");

    // Elevation
    cfg.ElevationDropThreshold =          ini.GetDoubleValue(sectionElevation, "ElevationDropThreshold");
    cfg.ElevationMin =                    ini.GetDoubleValue(sectionElevation, "ElevationMin");
    cfg.ElevationMax =                    ini.GetDoubleValue(sectionElevation, "ElevationMax");
    cfg.ElevationDangerMin =              ini.GetDoubleValue(sectionElevation, "ElevationDangerMin");
    cfg.ElevationDangerMax =              ini.GetDoubleValue(sectionElevation, "ElevationDangerMax");
    cfg.SteerLookAheadPitch =             ini.GetDoubleValue(sectionElevation, "SteerLookAheadPitch");

    // Track Limits
    cfg.TrackLimitsAdjustMinOvershoot =   ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsAdjustMinOvershoot");
    cfg.TrackLimitsAdjustMaxOvershoot =   ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsAdjustMaxOvershoot");
    cfg.TrackLimitsThrottleMultMinOvershoot = ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsThrottleMultMinOvershoot");
    cfg.TrackLimitsThrottleMultMaxOvershoot = ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsThrottleMultMaxOvershoot");
    cfg.TrackLimitsSteerMultMinOvershoot = ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsSteerMultMinOvershoot");
    cfg.TrackLimitsSteerMultMaxOvershoot = ini.GetDoubleValue(sectionTrackLimits, "TrackLimitsSteerMultMaxOvershoot");

    // Debug section
    cfg.ShowDebug =     ini.GetBoolValue(sectionDebug, "ShowDebug");
    cfg.ShowDebugText = ini.GetBoolValue(sectionDebug, "ShowDebugText");

    return cfg;
}
