#include "Claw.h"
#include "Prefs.h"

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, int pistonPullSolenoidModule):
    a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
    armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    armEncoder(armMotor.GetEncoder()),
    shuttleEncoder(shuttleMotor.GetEncoder()) {
        armEncoder.SetPositionConversionFactor(8.57142857143); // 360 / 42 (ticks per revolution for the rev encoder)
        shuttleEncoder.SetPositionConversionFactor(8.57142857143);
    }

void Claw::transformClaw(double angle, bool extend, double shuttle) {
    // forward = true, reverse = false
    if (extend == false){
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    int stage = 0;
    switch(stage){
        case 0: // rotate (0 - 175 degrees)
        if (angle < 0){
            armEncoder.SetPosition(0);
        } else if (angle > 175){
            armEncoder.SetPosition(175);
        }
        
        if (armEncoder.GetPosition() == angle){
            stage = 1;
        }
            break; 

        case 1: // shuttle movement
        shuttleEncoder.SetPosition(shuttle);
        if (getRelativeAngle(shuttleEncoder) == shuttle){
            stage = 2;
        }
            break;
        
        case 2: // finish rotation
        armEncoder.SetPosition(angle);
        if (armEncoder.GetPosition() == angle) {
            stage = 3;
        }
            break;
        case 3: // extend piston if needed
        if (extend == true){
            a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
        }
            break;
    }
}