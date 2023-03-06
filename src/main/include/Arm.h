#pragma once
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class Arm {
    public:
        Arm(int pushSolenoidModule, int pullSolenoidModule, // Extend and retract piston control
            int openSolenoidModule, int closeSolenoidModule, // Open and Close Claw
            int coneSolenoidModule, int cubeColenoidModule, // Select pressure for Claw piston
            int carriageID, int clawID, int carriageCANCoderID);
        void setSolenoid(bool deployed);
        void ArmPistonUp();
        void ArmPistonDown();
        void ArmMotorUp();
        void ArmMotorDown();
        void ClawOpen();
        void ClawClose();
        void ClawConePressure();
        void ClawCubePressure();
        void ClawMotorUp();
        void ClawMotorDown();
        void updateDashboard();
        double getAngle();

    private:
        frc::DoubleSolenoid a_clawSolenoid;
        frc::DoubleSolenoid a_armSolenoid;
        frc::DoubleSolenoid a_clawPressureSolenoid;
        rev::CANSparkMax a_carriageMotor;
        rev::CANSparkMax a_clawMotor;
        CANCoder a_CANCoder;
        rev::SparkMaxRelativeEncoder a_carriageMotorEncoder;
        
        int _CANCoderID;
};