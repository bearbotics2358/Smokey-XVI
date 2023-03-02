#pragma once

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include "LimitSwitch.h"

class Claw {
    public:
        Claw(int motorPort, int shuttlePort, int pistonPushSolenoidModule, int pistonPullSolenoidModule, int clawPullSolenoidModule, int clawPushSolenoidModule, int limitSwitchId);
        bool transformClaw(double angle, bool extend, double shuttle);
        void clawInit();
        bool zeroShuttle();
        void openClaw();
        void closeClaw();
    private:
        frc::DoubleSolenoid a_Piston;
        frc::DoubleSolenoid a_Claw;
        rev::CANSparkMax shuttleMotor;
        rev::CANSparkMax armMotor;
        rev::SparkMaxRelativeEncoder shuttleEncoder;
        rev::SparkMaxRelativeEncoder armEncoder;

        LimitSwitch shuttleZeroSwitch;
};