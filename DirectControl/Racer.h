#pragma once
#include <vector>
#include <inc/types.h>
#include "Blip.h"
#include "Memory/VehicleExtensions.hpp"

class Racer {
public:
    Racer(Vehicle vehicle, VehicleExtensions& ext);

    // non-copyable
    Racer(const Racer& other) = delete;
    Racer& operator= (const Racer& other) = delete;

    // move
    Racer(Racer&& other) noexcept;
    Racer& operator= (Racer&& other) noexcept = delete;

    ~Racer();

    void UpdateControl(const std::vector<Vector3> &coords);
    Vehicle GetVehicle();
    void SetActive(bool value);
    bool GetActive();
    void SetDebugView(bool value);
    bool GetDebugView();
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
    bool mActive;
    bool mDebugView;
};
