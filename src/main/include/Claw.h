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
                int clawPullSolenoidModule, int carriageCANCoder, int limitSwitchId);
        bool TransformClaw(double angle, double shuttle, bool extend);
        void HoldClaw();
        void clawInit();
        void setSolenoid(bool deployed);
        void ArmPistonUp();
        void ArmPistonDown();
        void ArmMotorUp();
        void ArmMotorDown();
        void ClawOpen();
        void ClawClose();
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
        bool ShuttleMoveToMM(double targetPosition);
        bool ShuttleHold();
        bool ArmMoveTo(double targetPosition);
        bool ArmHold();
        bool IsArmPIDAtSetpoint();
        bool IsShuttlePIDAtSetpoint();
    private:
#ifdef COMP_BOT  // Not available on the practice bot
        frc::DoubleSolenoid a_Piston;
        frc::DoubleSolenoid a_ClawSolenoid;
        rev::CANSparkMax shuttleMotor;
        rev::CANSparkMax armMotor;
        rev::SparkMaxRelativeEncoder shuttleEncoder;
        rev::SparkMaxRelativeEncoder armEncoder;
        CANCoder a_CANCoder;
        
        int _CANCoderID;
        double currentArmAngle;
        double currentShuttleAngle;

        LimitSwitch shuttleZeroSwitch;

        frc2::PIDController armPID;
        frc2::PIDController shuttlePID;
#endif
};