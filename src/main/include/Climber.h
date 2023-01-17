#pragma once
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

class Climber {
    public:
        Climber(int climberMotorId, int pushSolenoidModule, int pullSolenoidModule);
        void setArmSpeed(double percent);
        // true for arms extended, false for arms retracted
        void setSolenoid(bool deployed);
        void resetClimber();
        double getHeight();
        double getTicks();
        double getSpeed();
        bool LifterZero();

    private:
        TalonFX a_climberArmMotor;
        frc::DoubleSolenoid a_climberSolenoid;
        // frc::DigitalInput a_Switch;
};