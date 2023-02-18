#pragma once
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

class Arm {
    public:
        Arm(int pushSolenoidModule, int pullSolenoidModule);
        void setSolenoid(bool deployed);

    private:
        frc::DoubleSolenoid a_armSolenoid;
};