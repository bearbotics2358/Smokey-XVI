#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(int port):
a_Input(port)
{
    
}

bool LimitSwitch::limitSwitchPressed(){
    if (a_Input.Get() == 1) {
        isPressed = true;
    } else {
        isPressed = false;
    }
    return isPressed;
}