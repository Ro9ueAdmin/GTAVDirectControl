#pragma once
#include <vector>
#include <inc/types.h>
#include "Blip.h"

class Racer {
public:
    Racer(Vehicle vehicle);

    // non-copyable
    Racer(const Racer& other) = delete;
    Racer& operator= (const Racer& other) = delete;

    // move
    Racer(Racer&& other) noexcept;
    Racer& operator= (Racer&& other) noexcept;

    ~Racer();

    void UpdateControl(const std::vector<Vector3> &coords);
    Vehicle GetVehicle();
    float getCornerRadius(const std::vector<Vector3> &coords, int focus);
    void SetActive(bool value);
    bool GetActive();
    void SetDebugView(bool value);
    bool GetDebugView();
protected:
    void getControls(const std::vector<Vector3> &coords, float limitRadians, float actualAngle, bool &handbrake,
                     float &throttle, float &brake, float &steer);
    Vector3 getCoord(const std::vector<Vector3> &coords, float lookAheadDistance, float actualAngle, float &radiusAtCorner, std::string &source);
    float getSteeringAngle();
    float calculateReduction();
    float calculateDesiredHeading(float steeringAngle, float steeringMax, float desiredHeading,
                                  float reduction);

    Vehicle mVehicle;
    std::unique_ptr<BlipX> mBlip;
    bool mActive;
    bool mDebugView;
};
