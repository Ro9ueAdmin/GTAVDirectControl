#include "Timer.h"
#include <chrono>

using std::chrono::steady_clock;

Timer::Timer(int64_t timeout) :
    mPeriod(timeout),
    mPreviousTime(steady_clock::now().time_since_epoch().count()) {
}

void Timer::Reset() {
    mPreviousTime = steady_clock::now().time_since_epoch().count();
}

void Timer::Reset(int64_t newTimeout) {
    mPeriod = newTimeout;
    mPreviousTime = steady_clock::now().time_since_epoch().count();
}

bool Timer::Expired() const {
    return steady_clock::now().time_since_epoch().count() > mPreviousTime + mPeriod;
}

int64_t Timer::Elapsed() const {
    return steady_clock::now().time_since_epoch().count() - mPreviousTime;
}
