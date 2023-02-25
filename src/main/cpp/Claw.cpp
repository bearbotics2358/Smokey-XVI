#include "Claw.h"
#include "Prefs.h"

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, int pistonPullSolenoidModule, int limitSwitchId):
a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
armEncoder(armMotor.GetEncoder()),
shuttleEncoder(shuttleMotor.GetEncoder()), 
shuttleZeroSwitch(limitSwitchId) {
    armEncoder.SetPositionConversionFactor(20); // 360 / 18 (ticks per revolution for the rev encoder)
    shuttleEncoder.SetPositionConversionFactor(20);
}

void Claw::clawInit() {

}

bool Claw::zeroShuttle() {
    if (shuttleZeroSwitch.limitSwitchPressed() == true){
        shuttleMotor.StopMotor();
        return true;
    } else {
        shuttleMotor.Set(0.2);
        return false;
    }
}

void Claw::transformClaw(double angle, bool extend, double shuttle) {
    // forward = true, reverse = false
    if (extend == false){
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    int stage = 0;
    switch(stage){
        case 0: // rotate (0 - 175 degrees)
        if (angle < armEncoder.GetPosition()){
            armMotor.Set(-0.2);
            if (armEncoder.GetPosition() <= 1) {
                stage = 1;
            }
        } else if (angle > armEncoder.GetPosition()){
            armMotor.Set(0.2);
            if (armEncoder.GetPosition() >= 174) {
                stage = 1;
            }
        } else {
            stage = 1;
        }
            break; 

        case 1: // shuttle movement
        armMotor.StopMotor();
        shuttleEncoder.SetPosition(shuttle);
        if (shuttleEncoder.GetPosition() == shuttle){
            stage = 2;
        }
            break;
        
        case 2: // finish rotation
        shuttleMotor.StopMotor();
        if (angle < armEncoder.GetPosition()){
            armMotor.Set(-0.2);
            if (armEncoder.GetPosition() <= angle) {
                stage = 3;
            }
        } else if (angle > armEncoder.GetPosition()){
            armMotor.Set(0.2);
            if (armEncoder.GetPosition() >= angle) {
                stage = 3;
            }
        } else {
            stage = 3;
        }
            break;

        case 3: // extend piston if needed
        armMotor.StopMotor();
        if (extend == true){
            a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
        }
            break;
    }
}