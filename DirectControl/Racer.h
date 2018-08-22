#pragma once
#include <vector>
#include <inc/types.h>
#include "Blip.h"
#include "Memory/VehicleExtensions.hpp"
#include "XInputControl.h"

class Racer {
public:
    Racer(Vehicle vehicle, VehicleExtensions& ext);
    ~Racer();
    void SetDebugView(bool value);
    bool GetDebugView();
    void UpdateControl(const std::vector<Vector3> &coords);
    Vehicle GetVehicle();
protected:
    void getControls(const std::vector<Vector3> &coords, float limitRadians, bool &handbrake, float &throttle,
                     float &brake, float &steer);
    Vector3 getCoord(const std::vector<Vector3> &coords, float lookAheadDistance);
    float getSteeringAngle();
    float calculateReduction();
    float calculateDesiredHeading(float steeringAngle, float steeringMax, float desiredHeading,
                                  float reduction);

    Vehicle mVehicle;
    VehicleExtensions& mExt;
    std::unique_ptr<BlipX> mBlip;
    bool mDebugView;
};

class PlayerRacer : public Racer {

public:
    PlayerRacer(Vehicle vehicle, VehicleExtensions &ext, int playerNumber);
    void UpdateControl();
private:
    void getControls(float limitRadians, bool &handbrake, float &throttle,
        float &brake, float &steer);
    void drawDebugLines(float steeringAngle, float nextAngle);
    XInputController mXInput;
};