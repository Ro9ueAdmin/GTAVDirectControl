#pragma once
#include "Track.h"

class Session {
public:
    static Session& Get() {
        static Session session;
        return session;
    }

    void Reset() {
        mTrack = Track();
    }

    void SetTrack(const Track& track) {
        mTrack = track;
    }

    const Track& GetTrack() {
        return mTrack;
    }

    void ReverseTrack() {
        mTrack.Reverse();
    }

private:
    Session() = default;
    Track mTrack;
    std::vector<Point> mRecordPoints;
};

class Recorder {
public:
    static Recorder& Get() {
        static Recorder recorder;
        return recorder;
    }

    void ClearPoints() {
        mRecordPoints.clear();
    }

    void Append(Point p) {
        mRecordPoints.push_back(p);
    }

    const std::vector<Point>& Points() {
        return mRecordPoints;
    }
private:
    Recorder() = default;
    std::vector<Point> mRecordPoints;
};
