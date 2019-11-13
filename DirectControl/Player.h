#pragma once
#include "Racer.h"

class PlayerRacer {
public:
    PlayerRacer(Vehicle v);
    ~PlayerRacer();
    void UpdateControl();
    void SetTrack(const Track& t);

protected:
    void updateLapTimers();
    Point findClosestNode(size_t& trackIdx);
    void notify(const std::string& msg);

    Vehicle mVehicle;               // The vehicle the racer AI uses.

    bool mActive;                   // Active state.
    int mNotifyHandle;              // Single notification handle for each agent.

    const Track* mTrack;            // Current track. Ptr since we don't want a copy per AI.
    size_t mTrackIdx;               // Last valid track index / checkpoint
    Timer mLapTimer;                // Lap timer
    int64_t mLapTime;               // Last lap time
    int64_t mCurrentLap;            // What lap is AI on?
};
