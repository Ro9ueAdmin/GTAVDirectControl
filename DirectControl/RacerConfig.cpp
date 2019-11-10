#include "RacerConfig.h"
#include "Util/Logger.hpp"
#include "SimpleIni.h"
#include <filesystem>
#include <fstream>

namespace {
    const char* sectionGeneral = "General";
    const char* sectionLookahead = "Lookahead";
    const char* sectionSteering = "Steering";
    const char* sectionBraking = "Braking";
    const char* sectionElevation = "Elevation";
    const char* sectionTrackLimits = "TrackLimits";
    const char* sectionDebug = "Debug";
    const std::string pathPrefix = "./DirectControl/RacerConfigs";
}

/**
 * These defaults are suitable for Killatomate's AWD sports vehicles, such as the
 * Sultan or Kuruma. Similar vehicles should also perform well.
 */
RacerConfig::RacerConfig()
    : DefaultActive(true)
    , AutoRepair(true)
    , LookaheadThrottleSpeedMult(3.5f)
    , LookaheadThrottleMinDistance(25.0f)
    , LookaheadThrottleMaxDistance(150.0f)
    , LookaheadBrakeSpeedMult(2.5f)
    , LookaheadBrakeMinDistance(20.0f)
    , LookaheadBrakeMaxDistance(75.0f)
    , LookaheadSteerSpeedMult(1.0f)
    , LookaheadSteerMinDistance(15.0f)
    , LookaheadSteerMaxDistance(35.0f)
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
    , BrakePointSpeedMultMin(0.375f)
    , BrakePointSpeedMultMax(3.500f)
    , RadiusActivationMult(0.85f)
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
    std::string filename;
    for (auto& p : std::filesystem::directory_iterator(pathPrefix)) {
        if (p.path().extension() == ".ini" && toLower(p.path().stem().string()) == toLower(name))
            filename = p.path().string();
    }
    return filename;
}

RacerConfig RacerConfig::Parse(const std::string& name) {
    std::string fullPath = getFullPath(name);

    if (fullPath.empty()) {
        gLogger.Write(ERROR, "[RacerConfig] Error loading [%s], loading defaults", name.c_str());
        fullPath = getFullPath("Default");
        if (fullPath.empty()) {
            gLogger.Write(ERROR, "              Error loading default config, loading hardcoded defaults", name.c_str());
            RacerConfig defaultConfig{};
            defaultConfig.Save("Default");
            return RacerConfig();
        }
    }

    CSimpleIniA ini;
    ini.SetUnicode(true);
    SI_Error err = ini.LoadFile(fullPath.c_str());

    if (err != SI_OK) {
        gLogger.Write(ERROR, "[RacerConfig] Error loading [%s] (SimpleIni err [%d]), using defaults", fullPath.c_str(), err);
        return RacerConfig();
    }

    RacerConfig cfg{};

    // General section
    cfg.AutoRepair =    ini.GetBoolValue(sectionGeneral, "AutoRepair");
    cfg.DefaultActive = ini.GetBoolValue(sectionGeneral, "DefaultActive");

    // Lookahead
    cfg.LookaheadThrottleSpeedMult =      ini.GetDoubleValue(sectionLookahead, "LookaheadThrottleSpeedMult");
    cfg.LookaheadThrottleMinDistance =    ini.GetDoubleValue(sectionLookahead, "LookaheadThrottleMinDistance");
    cfg.LookaheadThrottleMaxDistance =    ini.GetDoubleValue(sectionLookahead, "LookaheadThrottleMaxDistance");

    cfg.LookaheadBrakeSpeedMult =         ini.GetDoubleValue(sectionLookahead, "LookaheadBrakeSpeedMult");
    cfg.LookaheadBrakeMinDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadBrakeMinDistance");
    cfg.LookaheadBrakeMaxDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadBrakeMaxDistance");

    cfg.LookaheadSteerSpeedMult =         ini.GetDoubleValue(sectionLookahead, "LookaheadSteerSpeedMult");
    cfg.LookaheadSteerMinDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadSteerMinDistance");
    cfg.LookaheadSteerMaxDistance =       ini.GetDoubleValue(sectionLookahead, "LookaheadSteerMaxDistance");

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

    cfg.BrakePointSpeedMultMin = ini.GetDoubleValue(sectionBraking, "BrakePointSpeedMultMin");
    cfg.BrakePointSpeedMultMax = ini.GetDoubleValue(sectionBraking, "BrakePointSpeedMultMax");

    cfg.RadiusActivationMult = ini.GetDoubleValue(sectionBraking, "RadiusActivationMult");
    cfg.LateralOffsetMin = ini.GetDoubleValue(sectionBraking, "LateralOffsetMin");
    cfg.LateralOffsetMax = ini.GetDoubleValue(sectionBraking, "LateralOffsetMax");
    cfg.LateralScaleSpeed = ini.GetDoubleValue(sectionBraking, "LateralScaleSpeed");

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

void RacerConfig::Save(const std::string& name) {
    std::string file = pathPrefix + "/" + name + ".ini";

    std::ofstream fileStream(file, std::ofstream::out | std::ofstream::trunc);
    fileStream << "; AI configuration file - saved by script";
    fileStream.close();

    CSimpleIniA ini;
    ini.SetUnicode(true);
    ini.LoadFile(file.c_str());

    // General section
    ini.SetBoolValue(sectionGeneral, "AutoRepair", AutoRepair);
    ini.SetBoolValue(sectionGeneral, "DefaultActive", DefaultActive);

    // Lookahead
    ini.SetDoubleValue(sectionLookahead, "LookaheadThrottleSpeedMult", LookaheadThrottleSpeedMult);
    ini.SetDoubleValue(sectionLookahead, "LookaheadThrottleMinDistance", LookaheadThrottleMinDistance);
    ini.SetDoubleValue(sectionLookahead, "LookaheadThrottleMaxDistance", LookaheadThrottleMaxDistance);

    ini.SetDoubleValue(sectionLookahead, "LookaheadBrakeSpeedMult", LookaheadBrakeSpeedMult);
    ini.SetDoubleValue(sectionLookahead, "LookaheadBrakeMinDistance", LookaheadBrakeMinDistance);
    ini.SetDoubleValue(sectionLookahead, "LookaheadBrakeMaxDistance", LookaheadBrakeMaxDistance);

    ini.SetDoubleValue(sectionLookahead, "LookaheadSteerSpeedMult", LookaheadSteerSpeedMult);
    ini.SetDoubleValue(sectionLookahead, "LookaheadSteerMinDistance", LookaheadSteerMinDistance);
    ini.SetDoubleValue(sectionLookahead, "LookaheadSteerMaxDistance", LookaheadSteerMaxDistance);

    // Steering
    ini.SetDoubleValue(sectionSteering, "SteerMult", SteerMult);
    ini.SetDoubleValue(sectionSteering, "CountersteerIncreaseStartAngle", CountersteerIncreaseStartAngle);
    ini.SetDoubleValue(sectionSteering, "CountersteerIncreaseEndAngle", CountersteerIncreaseEndAngle);
    ini.SetDoubleValue(sectionSteering, "ThrottleDecreaseStartAngle", ThrottleDecreaseStartAngle);
    ini.SetDoubleValue(sectionSteering, "ThrottleDecreaseEndAngle", ThrottleDecreaseEndAngle);
    ini.SetDoubleValue(sectionSteering, "OversteerDetectionAngle", OversteerDetectionAngle);
    ini.SetDoubleValue(sectionSteering, "UndersteerHandbrakeTrigger", UndersteerHandbrakeTrigger);

    // Braking
    ini.SetDoubleValue(sectionBraking, "BrakePointDistanceThrottleMult", BrakePointDistanceThrottleMult);
    ini.SetDoubleValue(sectionBraking, "BrakePointDistanceBrakeMult", BrakePointDistanceBrakeMult);
    ini.SetDoubleValue(sectionBraking, "BrakePointHeadingMinAngle", BrakePointHeadingMinAngle);
    ini.SetDoubleValue(sectionBraking, "BrakePointHeadingMaxAngle", BrakePointHeadingMaxAngle);
    ini.SetDoubleValue(sectionBraking, "BrakePointHeadingMinSpeed", BrakePointHeadingMinSpeed);
    ini.SetDoubleValue(sectionBraking, "BrakePointHeadingMaxSpeed", BrakePointHeadingMaxSpeed);

    // Elevation
    ini.SetDoubleValue(sectionElevation, "ElevationDropThreshold", ElevationDropThreshold);
    ini.SetDoubleValue(sectionElevation, "ElevationMin", ElevationMin);
    ini.SetDoubleValue(sectionElevation, "ElevationMax", ElevationMax);
    ini.SetDoubleValue(sectionElevation, "ElevationDangerMin", ElevationDangerMin);
    ini.SetDoubleValue(sectionElevation, "ElevationDangerMax", ElevationDangerMax);
    ini.SetDoubleValue(sectionElevation, "SteerLookAheadPitch", SteerLookAheadPitch);

    // Track Limits
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsAdjustMinOvershoot", TrackLimitsAdjustMinOvershoot);
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsAdjustMaxOvershoot", TrackLimitsAdjustMaxOvershoot);
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsThrottleMultMinOvershoot", TrackLimitsThrottleMultMinOvershoot);
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsThrottleMultMaxOvershoot", TrackLimitsThrottleMultMaxOvershoot);
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsSteerMultMinOvershoot", TrackLimitsSteerMultMinOvershoot);
    ini.SetDoubleValue(sectionTrackLimits, "TrackLimitsSteerMultMaxOvershoot", TrackLimitsSteerMultMaxOvershoot);

    // Debug section
    ini.SetBoolValue(sectionDebug, "ShowDebug", ShowDebug);
    ini.SetBoolValue(sectionDebug, "ShowDebugText", ShowDebugText);

    SI_Error err = ini.SaveFile(file.c_str());
    if (err == SI_FAIL) {
        gLogger.Write(ERROR, "[RacerConfig] Error saving [%s]", name.c_str());
    }
}
