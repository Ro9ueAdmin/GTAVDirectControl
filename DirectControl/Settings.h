#pragma once

class Settings {
public:
    void SetOptimalDefaults();
    Settings();
    ~Settings() = default;

    void ReadSettings(const char *file);
    void WriteDefaults(const char * file);

    bool TrackShowDebug;                        // to default show track line
    bool AIShowDebug;                           // to default spawn AI with debug info
    bool AIShowDebugText;                       // enables debug text if debug normal is enabled
    bool AIDefaultActive;                       // to default spawn AI with AI active

    float AILookaheadThrottleSpeedMult;         // m/s meters lookahead multiplier
    float AILookaheadThrottleMinDistance;       // minimal lookahead distance
    float AILookaheadBrakeSpeedMult;            // m/s meters lookahead multiplier
    float AILookaheadBrakeMinDistance;          // minimal lookahead distance
    float AILookaheadSteerSpeedMult;            // m/s meters lookahead multiplier
    float AILookaheadSteerMinDistance;          // minimal lookahead distance

    float AISteerMult;                          // default steering input multiplier

    float AICountersteerIncreaseStartAngle;     // oversteer angle for default countersteer multiplier
    float AICountersteerIncreaseEndAngle;       // oversteer angle for max countersteer multiplier

    float AIThrottleDecreaseStartAngle;         // oversteer angle to start throttle decrease
    float AIThrottleDecreaseEndAngle;           // oversteer angle for no throttle

    float AIOversteerDetectionAngle;            // detect oversteer at this angle
    
    float AIBrakePointDistanceThrottleMult;     // m/s meters to throttle lookahead dist mult to start braking
    float AIBrakePointDistanceBrakeMult;        // m/s meters to brake lookahead dist mult for max braking

    // Influenced by brake and throttle lookahead
    float AIBrakePointHeadingMinAngle;          // Upcoming angle to start braking at
    float AIBrakePointHeadingMaxAngle;          // Upcoming angle braking is max at
    float AIBrakePointHeadingMinSpeed;          // Speed for upcoming angle to start braking 
    float AIBrakePointHeadingMaxSpeed;          // Speed for upcoming angle for max braking

    float AIBrakePointRadiusMaxSpeed;           // Speed radius is considered for
    float AIBrakePointRadiusMaxRadius;          // Considered radius at max speed

    float AIElevationDropThreshold;
    float AIElevationMin;
    float AIElevationMax;
    float AIElevationDangerMin;
    float AIElevationDangerMax;
};

extern Settings gSettings;
