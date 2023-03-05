#include "Claw.h"
#include "Prefs.h"

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, int pistonPullSolenoidModule, int clawPushSolenoidModule, int clawPullSolenoidModule, int limitSwitchId):
a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
a_Claw(frc::PneumaticsModuleType::REVPH, clawPushSolenoidModule, clawPullSolenoidModule),
armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
/*armEncoder(armMotor.GetEncoder()),
shuttleEncoder(shuttleMotor.GetEncoder()), */
shuttleZeroSwitch(limitSwitchId) {
    /* armEncoder.SetPositionConversionFactor(20); // 360 / 18 (ticks per revolution for the rev encoder)
    shuttleEncoder.SetPositionConversionFactor(20); */
}

void Claw::clawInit() {
    //transformClaw(30, false, 0);
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

void Claw::openClaw(){
    a_Claw.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::closeClaw(){
    a_Claw.Set(frc::DoubleSolenoid::Value::kForward);
}

/*bool Claw::transformClaw(double desiredAngle, bool extend, double desiredShuttle) {

    double armMotorSpeed = 0.2;
    double shuttleMotorSpeed = 0.2;

    // forward = true, reverse = false
    if (extend == false){
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse); // retract piston if needed
    }

    int stage = 0;
    switch(stage){
        case 0: // rotate just enough to not damage the robot (30 - 120 degrees) 
        // these are placeholder angles that should be changed once we can test the arm's cancoder
        if (abs(armEncoder.GetPosition() - desiredAngle) < 3) {
            stage = 1;
        } else if (desiredAngle < armEncoder.GetPosition()){
            armMotor.Set(-armMotorSpeed);
            if (armEncoder.GetPosition() <= 30) {
                stage = 1;
            }
        } else if (desiredAngle > armEncoder.GetPosition()){
            armMotor.Set(armMotorSpeed);
            if (armEncoder.GetPosition() >= 120) {
                stage = 1;
            }
        }
            break; 

        case 1: // shuttle movement
        armMotor.StopMotor();
        if (desiredShuttle == 0){
            zeroShuttle();
            if (zeroShuttle() == true){
                stage = 2;
            }
        } else {
            if (abs(shuttleEncoder.GetPosition() - desiredShuttle) < 3){
            stage = 2;
            } else if (shuttleEncoder.GetPosition() < desiredShuttle){
                shuttleMotor.Set(shuttleMotorSpeed);
            } else if (shuttleEncoder.GetPosition() > desiredShuttle){
                shuttleMotor.Set(shuttleMotorSpeed);
            }
        }
            break;
        
        case 2: // finish rotation
        shuttleMotor.StopMotor();
        if (abs(armEncoder.GetPosition() - desiredAngle) < 3) {
            stage = 1;
        } else if (desiredAngle < armEncoder.GetPosition()){
            armMotor.Set(-armMotorSpeed);
            if (armEncoder.GetPosition() <= desiredAngle) {
                stage = 3;
            }
        } else if (desiredAngle > armEncoder.GetPosition()){
            armMotor.Set(armMotorSpeed);
            if (armEncoder.GetPosition() >= desiredAngle) {
                stage = 3;
            }
        }
            break;

        case 3: // extend piston if needed
        armMotor.StopMotor();
        if (extend == true){
            a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
        }
        return true;
            break;

        return false;
    }
    
}
*/