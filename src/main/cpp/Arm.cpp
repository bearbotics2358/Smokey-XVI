#include "Arm.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm(int pushSolenoidModule, int pullSolenoidModule, int openSolenoidModule, int closeSolenoidModule, int coneSolenoidModule, int cubeSolenoidModule, int carriageID, int clawID, int carriageCANCoderID):
a_armSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule),
a_clawSolenoid(frc::PneumaticsModuleType::REVPH, openSolenoidModule, closeSolenoidModule),
a_clawPressureSolenoid(frc::PneumaticsModuleType::REVPH, coneSolenoidModule, cubeSolenoidModule),
a_carriageMotor(carriageID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
a_clawMotor(clawID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
a_CANCoder(carriageCANCoderID),
a_carriageMotorEncoder(a_carriageMotor.GetEncoder()) {
    _CANCoderID = carriageCANCoderID - 17;
}

void Arm::updateDashboard(){
    frc::SmartDashboard::PutNumber("arm absolute encoder: ", getAngle());
    frc::SmartDashboard::PutNumber("shuttle motor position: ", a_carriageMotorEncoder.GetPosition());
    if (a_armSolenoid.Get() == frc::DoubleSolenoid::Value::kReverse){
        frc::SmartDashboard::PutString("arm solenoid position: ", "reverse");
    } else if (a_armSolenoid.Get() == frc::DoubleSolenoid::Value::kForward){
        frc::SmartDashboard::PutString("arm solenoid position: ", "forward");
    } else {
        frc::SmartDashboard::PutString("arm solenoid position: ", "off");
    }
    
}

double Arm::getAngle(){
    return a_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID];
}


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

void Arm::ClawConePressure(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Arm::ClawCubePressure(){
    a_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Arm::ClawMotorUp() {
    a_clawMotor.Set(0.1);
}

void Arm::ClawMotorDown() {
    a_clawMotor.Set(-0.1);
}