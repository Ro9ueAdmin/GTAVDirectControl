#include "Track.h"

#include "Util/MathExt.h"

#include <utility>

Track::Track()
    : mName("No track")
    , mLength(0.0f)
    , mType(Type::Circuit) {}

Track::Track(std::string name, std::vector<Point> points)
    : mName(std::move(name))
    , mPoints(std::move(points))
    , mType(Type::Circuit) {
    mLength = 0.0f;
    for (size_t i = 0; i < mPoints.size(); ++i) {
        mLength += Distance(mPoints[i].v, mPoints[(i + 1) % mPoints.size()].v);
    }
    if (Distance(mPoints.front().v, mPoints.back().v) > 50.0f) {
        mType = Type::Sprint;
    }
}

std::string Track::Name() const {
    return mName;
}

const std::vector<Point>& Track::Points() const {
    return mPoints;
}

float Track::Length() const {
    return mLength;
}

Track::Type Track::GetType() const {
    return mType;
}

void Track::Reverse() {
    std::reverse(mPoints.begin(), mPoints.end());
}
