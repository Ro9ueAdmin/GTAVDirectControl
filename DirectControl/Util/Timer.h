#pragma once
#include <cstdint>

class Timer
{
public:
    explicit Timer(int64_t timeout);
    ~Timer() = default;
    void Reset();
    void Reset(int64_t newTimeout);
    bool Expired() const;
    int64_t Elapsed() const;
private:
    int64_t mPeriod;
    int64_t mPreviousTime;
};

