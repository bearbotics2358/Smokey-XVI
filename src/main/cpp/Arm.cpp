#include "Arm.h"

Arm::Arm(int pushSolenoidModule, int pullSolenoidModule, int openSolenoidModule, int closeSolenoidModule, int carriageID, int clawID, int carriageCANCoderID):
    a_armSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule),
    a_clawSolenoid(frc::PneumaticsModuleType::REVPH, openSolenoidModule, closeSolenoidModule),
    a_carriageMotor(carriageID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    a_clawMotor(clawID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    a_CANCoder(carriageCANCoderID) {}

void Arm::setSolenoid(bool deployed) {
    if (deployed) {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
}

// MANY OF THESE ARE MANUAL, WILL BE REPLACED WITH AUTOMATION
void Arm::ArmPistonUp(){
    a_armSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Arm::ArmPistonDown(){
    a_armSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}
void Arm::ArmMotorUp(){
    a_carriageMotor.Set(0.1);
}
void Arm::ArmMotorDown(){
    a_carriageMotor.Set(-0.1);
}
void Arm::ClawOpen(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Arm::ClawClose(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Arm::ClawMotorUp() {
    a_clawMotor.Set(0.1);
}
void Arm::ClawMotorDown() {
    a_clawMotor.Set(-0.1);
}