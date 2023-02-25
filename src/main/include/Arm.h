#pragma once
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>

class Arm {
    public:
        Arm(int pushSolenoidModule, int pullSolenoidModule, int openSolenoidModule, int closeSolenoidModule, int carriageID);
        void setSolenoid(bool deployed);
        void ArmPistonUp();
        void ArmPistonDown();
        void ArmMotorUp();
        void ArmMotorDown();
        void ClawOpen();
        void ClawClose();

    private:
        frc::DoubleSolenoid a_clawSolenoid;
        frc::DoubleSolenoid a_armSolenoid;
};