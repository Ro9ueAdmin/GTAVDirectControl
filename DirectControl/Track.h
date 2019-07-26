#pragma once
#include <inc/types.h>
#include <vector>

class Point {
public:
    Point() = default;
    explicit Point(Vector3 coord, float width) : v(coord), w(width) {}
    Vector3 v;
    float w;
};

class Track {
public:
    enum class Type {
        Circuit,
        Sprint
    };

    Track();
    explicit Track(std::string name, std::vector<Point> points);

    const std::vector<Point>& Points() const;

    float Length() const;

    Type GetType() const;

    void Reverse();

protected:
    std::string mName;
    std::vector<Point> mPoints;
    float mLength;
    Type mType;
};

