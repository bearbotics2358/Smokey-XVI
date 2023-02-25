#include "Arm.h"

Arm::Arm(int pushSolenoidModule, int pullSolenoidModule, int openSolenoidModule, int closeSolenoidModule, int carriageID):
    a_armSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule),
    a_clawSolenoid(frc::PneumaticsModuleType::REVPH, openSolenoidModule, closeSolenoidModule) {}

void Arm::setSolenoid(bool deployed) {
    if (deployed) {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
}
void Arm::ArmPistonUp(){
    a_armSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Arm::ArmPistonDown(){
    a_armSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}
void Arm::ArmMotorUp(){

}
void Arm::ArmMotorDown(){
    
}
void Arm::ClawOpen(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Arm::ClawClose(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}