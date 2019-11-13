#include "Player.h"
#include "Util/MathExt.h"
#include "Memory/VehicleExtensions.hpp"
#include "Util/StringFormat.h"
#include "Util/UIUtils.h"

size_t getPointsBetween(const std::vector<Point>& coords, size_t a, size_t b);

PlayerRacer::PlayerRacer(Vehicle vehicle)
    : mVehicle(vehicle)
    , mActive(false)
    , mNotifyHandle(0)
    , mTrack(nullptr)
    , mTrackIdx(0)
    , mLapTimer(0)
    , mLapTime(0)
    , mCurrentLap(0) {}

PlayerRacer::~PlayerRacer() {
    // no implementation
}

void PlayerRacer::UpdateControl() {
    updateLapTimers();
}

void PlayerRacer::SetTrack(const Track& t) {
    mTrack = &t;
    findClosestNode(mTrackIdx);
    if (mTrackIdx == std::numeric_limits<size_t>::max()) {
        notify("Track unset/removed");
        mActive = false;
    }
    else {
        notify(fmt("Track set, closest node = %d", mTrackIdx));
    }
}

void PlayerRacer::notify(const std::string& msg) {
    std::string name = getGxtName(ENTITY::GET_ENTITY_MODEL(mVehicle));
    std::string plate = VEHICLE::GET_VEHICLE_NUMBER_PLATE_TEXT(mVehicle);

    showNotification(fmt("Player ~b~%s (~r~%s~b~)\n~w~%s",
        name.c_str(), plate.c_str(), msg.c_str()),
        &mNotifyHandle);
}

void PlayerRacer::updateLapTimers() {
    const std::vector<Point>& points = mTrack->Points();
    size_t currPointIdx = mTrackIdx;
    bool finishPassed = false;
    // Find coord closest to ourselves
    float smallestToAi = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    for (auto i = 0ull; i < points.size(); ++i) {
        float distanceAi = Distance(aiPosition, points[i].v);
        if (distanceAi < smallestToAi) {
            smallestToAi = distanceAi;
            currPointIdx = i;
        }
    }

    // TODO: Verify if it always successfully passes point zero / no overlap / doesn't jump
    if (getPointsBetween(points, mTrackIdx, currPointIdx) < 16) {
        if (currPointIdx == 0 && mTrackIdx == points.size() - 1)
            finishPassed = true;
        // We've progressed! Great!
        mTrackIdx = currPointIdx;
    }

    if (finishPassed) {
        mLapTime = mLapTimer.Elapsed();
        mLapTimer.Reset();
        uint64_t min = (mLapTime / (1000 * 60)) % 60;
        uint64_t sec = (mLapTime / 1000) % 60;
        uint64_t mil = (mLapTime % 1000);
        notify(fmt("Last lap: %d:%02d.%03d", min, sec, mil));
    }
}

Point PlayerRacer::findClosestNode(size_t& trackIdx) {
    Point closestPoint{};
    trackIdx = std::numeric_limits<size_t>::max();

    float smallestDistanceAI = 10000.0f;
    Vector3 aiPosition = ENTITY::GET_ENTITY_COORDS(mVehicle, true);
    for (size_t i = 0; i < mTrack->Points().size(); ++i) {
        Point point = mTrack->Points()[i];
        Vector3 coord = point.v;
        float distanceAI = Distance(aiPosition, coord);
        if (distanceAI < smallestDistanceAI) {
            smallestDistanceAI = distanceAI;
            closestPoint = point;
            trackIdx = i;
        }
    }
    return closestPoint;
}
