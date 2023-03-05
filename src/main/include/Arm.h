#pragma once
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class Arm {
    public:
        Arm(int pushSolenoidModule, int pullSolenoidModule, int openSolenoidModule, int closeSolenoidModule, int carriageID, int clawID, int carriageCANCoderID);
        void setSolenoid(bool deployed);
        void ArmPistonUp();
        void ArmPistonDown();
        void ArmMotorUp();
        void ArmMotorDown();
        void ClawOpen();
        void ClawClose();
        void ClawMotorUp();
        void ClawMotorDown();
        void updateDashboard();
        double getAngle();

    private:
        frc::DoubleSolenoid a_clawSolenoid;
        frc::DoubleSolenoid a_armSolenoid;
        rev::CANSparkMax a_carriageMotor;
        rev::CANSparkMax a_clawMotor;
        CANCoder a_CANCoder;
        rev::SparkMaxRelativeEncoder a_carriageMotorEncoder;
        
        int _CANCoderID;
};