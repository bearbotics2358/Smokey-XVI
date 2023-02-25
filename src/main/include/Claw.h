#pragma once

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>

class Claw {
    public:
        Claw(int motorPort, int shuttlePort, int pistonPushSolenoidModule, int pistonPullSolenoidModule);
        void transformClaw(double angle, bool extend, double shuttle);
    private:
        frc::DoubleSolenoid a_Piston;
        rev::CANSparkMax shuttleMotor;
        rev::CANSparkMax armMotor;
        rev::SparkMaxRelativeEncoder shuttleEncoder;
        rev::SparkMaxRelativeEncoder armEncoder;

        double getRelativeAngle(rev::SparkMaxRelativeEncoder encoder);
};