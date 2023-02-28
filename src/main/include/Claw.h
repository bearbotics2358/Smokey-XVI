#pragma once

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include "LimitSwitch.h"

class Claw {
    public:
        Claw(int motorPort, int shuttlePort, int pistonPushSolenoidModule, int pistonPullSolenoidModule, int limitSwitchId);
        void transformClaw(double angle, bool extend, double shuttle);
        void clawInit();
        bool zeroShuttle();
    private:
        frc::DoubleSolenoid a_Piston;
        rev::CANSparkMax shuttleMotor;
        rev::CANSparkMax armMotor;
        rev::SparkMaxRelativeEncoder shuttleEncoder;
        rev::SparkMaxRelativeEncoder armEncoder;

        LimitSwitch shuttleZeroSwitch;
};