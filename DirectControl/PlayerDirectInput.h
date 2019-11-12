#pragma once
#include "Racer.h"
#include "XInputControl.h"

class PlayerDirectInput : public Racer {

public:
    PlayerDirectInput(Vehicle vehicle, int playerNumber);
    void UpdateControl();
private:
    void getControls(bool &handbrake, float &throttle, float &brake, float &steer);
    void getControllerControls(bool& handbrake, float& throttle, float& brake, float& steer);
    void getKeyboardControls(bool& handbrake, float& throttle, float& brake, float& steer);
    void drawDebugLines(float steeringAngle, float nextAngle);
    XInputController mXInput;
};
