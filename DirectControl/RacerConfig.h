#pragma once
#include <string>

class RacerConfig {
public:
    RacerConfig();

    static RacerConfig Parse(const std::string& name);
    void Save(const std::string& name);

    // [General]
    bool DefaultActive;                       // to default spawn AI with AI active
    bool AutoRepair;

    // [Lookahead]
    float LookaheadThrottleSpeedMult;         // m/s meters lookahead multiplier
    float LookaheadThrottleMinDistance;       // minimal lookahead distance
    float LookaheadBrakeSpeedMult;            // m/s meters lookahead multiplier
    float LookaheadBrakeMinDistance;          // minimal lookahead distance
    float LookaheadSteerSpeedMult;            // m/s meters lookahead multiplier
    float LookaheadSteerMinDistance;          // minimal lookahead distance

    // [Steering]
    float SteerMult;                          // default steering input multiplier

    float CountersteerIncreaseStartAngle;     // oversteer angle for default countersteer multiplier
    float CountersteerIncreaseEndAngle;       // oversteer angle for max countersteer multiplier

    float ThrottleDecreaseStartAngle;         // oversteer angle to start throttle decrease
    float ThrottleDecreaseEndAngle;           // oversteer angle for no throttle

    float OversteerDetectionAngle;            // detect oversteer at this angle
    float SteerLookAheadPitch;
    float UndersteerHandbrakeTrigger;

    // [Braking]
    float BrakePointDistanceThrottleMult;     // m/s meters to throttle lookahead dist mult to start braking
    float BrakePointDistanceBrakeMult;        // m/s meters to brake lookahead dist mult for max braking

    // Influenced by brake and throttle lookahead
    float BrakePointHeadingMinAngle;          // Upcoming angle to start braking at
    float BrakePointHeadingMaxAngle;          // Upcoming angle braking is max at
    float BrakePointHeadingMinSpeed;          // Speed for upcoming angle to start braking 
    float BrakePointHeadingMaxSpeed;          // Speed for upcoming angle for max braking

    float BrakePointRadiusMaxSpeed;           // Speed radius is considered for
    float BrakePointRadiusMaxRadius;          // Considered radius at max speed

    // [Elevation]
    float ElevationDropThreshold;
    float ElevationMin;
    float ElevationMax;
    float ElevationDangerMin;
    float ElevationDangerMax;

    // [TrackLimits]
    float TrackLimitsAdjustMinOvershoot;
    float TrackLimitsAdjustMaxOvershoot;
    float TrackLimitsThrottleMultMinOvershoot;
    float TrackLimitsThrottleMultMaxOvershoot;
    float TrackLimitsSteerMultMinOvershoot;
    float TrackLimitsSteerMultMaxOvershoot;

    // [Debug]
    bool ShowDebug;                           // to default spawn AI with debug info
    bool ShowDebugText;                       // enables debug text if debug normal is enabled
};
