#pragma once
#include <inc/types.h>
#include <vector>

template <typename T>
constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template<typename T, typename A>
T avg(std::vector<T, A> const& vec) {
    T average{};
    for (auto elem : vec)
        average += elem;
    return average / vec.size();
}

float lerp(float a, float b, float f);

float Length(Vector3 vec);
float Distance(Vector3 vec1, Vector3 vec2);
Vector3 Cross(Vector3 left, Vector3 right);
float Dot(Vector3 a, Vector3 b);
Vector3 operator + (Vector3 left, Vector3 right);
Vector3 operator - (Vector3 left, Vector3 right);
Vector3 operator * (Vector3 value, float scale);
Vector3 operator * (float scale, Vector3 vec);
bool operator== (Vector3 a, Vector3 b);
Vector3 Normalize(Vector3 vec);
Vector3 GetOffsetInWorldCoords(Vector3 position, Vector3 rotation, Vector3 forward, Vector3 offset);
float GetAngleBetween(Vector3 a, Vector3 b);
float GetAngleBetween(float h1, float h2, float separation);

// Return true if line segments AB and CD intersect
bool Intersect(Vector3 A, Vector3 B, Vector3 C, Vector3 D);

template <typename T>
constexpr T rad2deg(T rad) {
    return static_cast<T>(static_cast<double>(rad) * (180.0 / 3.141592653589793238463));
}

template <typename T>
constexpr T deg2rad(T deg) {
    return static_cast<T>(static_cast<double>(deg) * 3.141592653589793238463 / 180.0);
}

template <typename T1, typename T2>
constexpr T2 map(T1 x, T1 in_min, T1 in_max, T2 out_min, T2 out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GetRand(int min, int mod);

Vector3 GetEntityDimensions(Entity e);

Vector3 GetModelDimensions(Hash model);

Vector3 GetPerpendicular(Vector3 a, Vector3 b, float length, bool clockwise);
