#include "Timer.h"
#include <chrono>

using std::chrono::steady_clock;

Timer::Timer(int64_t timeout) :
    mPeriod(timeout),
    mPreviousTime(std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now().time_since_epoch()).count()) {
}

void Timer::Reset() {
    mPreviousTime = std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void Timer::Reset(int64_t newTimeout) {
    mPeriod = newTimeout;
    mPreviousTime = std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool Timer::Expired() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now().time_since_epoch()).count() > mPreviousTime + mPeriod;
}

int64_t Timer::Elapsed() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now().time_since_epoch()).count() - mPreviousTime;
}
