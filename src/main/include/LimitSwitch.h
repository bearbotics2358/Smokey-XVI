#pragma once

#include "Prefs.h"
#include <frc/DigitalInput.h>

/** Class for a the limit switch
 *  currently used for the climber arms to detect when 
 */
class LimitSwitch {
    public:
        LimitSwitch(int port);
        bool limitSwitchPressed();

    private:
        frc::DigitalInput a_Input;
        bool isPressed = false;
};