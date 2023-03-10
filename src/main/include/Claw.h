#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include "LimitSwitch.h"
#include <ctre/phoenix/sensors/CANCoder.h>

class Claw {
    public:
        Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, 
        int pistonPullSolenoidModule, int clawPushSolenoidModule, 
        int clawPullSolenoidModule, /*int conePressureSolenoidModule, 
        int cubePressureSolenoidModule, */ int carriageCANCoder, int limitSwitchId);
        int transformClaw(double angle, bool extend, double shuttle);
        void clawInit();
        bool zeroShuttle();
        void setSolenoid(bool deployed);
        void ArmPistonUp();
        void ArmPistonDown();
        void ArmMotorUp();
        void ArmMotorDown();
        void ClawOpen();
        void ClawClose();
        // void CubePressure();
        // void ConePressure();
        void ShuttleMotorUp();
        void ShuttleMotorDown();
        double GetShuttlePositionMM();
        double GetShuttlePositionInches();
        void updateDashboard();
        double getAngle();
        void StopShuttle();
        void StopArm();
        void UpdateShuttleEncoder();
        bool IsShuttleSafeToMove();
    private:
        frc::DoubleSolenoid a_Piston;
        frc::DoubleSolenoid a_ClawSolenoid;
        //frc::DoubleSolenoid a_PressureSolenoid;
        rev::CANSparkMax shuttleMotor;
        rev::CANSparkMax armMotor;
        rev::SparkMaxRelativeEncoder shuttleEncoder;
        rev::SparkMaxRelativeEncoder armEncoder;
        CANCoder a_CANCoder;
        
        int _CANCoderID;

        LimitSwitch shuttleZeroSwitch;

        frc2::PIDController armPID;
        frc2::PIDController shuttlePID;
};