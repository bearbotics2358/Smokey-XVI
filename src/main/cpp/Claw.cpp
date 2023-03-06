#include "Claw.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, 
        int pistonPullSolenoidModule, int clawPushSolenoidModule, int clawPullSolenoidModule, 
        int conePressureSolenoidModule, int cubePressureSolenoidModule, 
        int limitSwitchId, int carriageCANCoderID):
a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
a_ClawSolenoid(frc::PneumaticsModuleType::REVPH, clawPushSolenoidModule, clawPullSolenoidModule),
a_PressureSolenoid(frc::PneumaticsModuleType::REVPH, conePressureSolenoidModule, cubePressureSolenoidModule),
armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
armEncoder(armMotor.GetEncoder()),
shuttleEncoder(shuttleMotor.GetEncoder()), 
shuttleZeroSwitch(limitSwitchId),
a_CANCoder(carriageCANCoderID),
a_carriageMotorEncoder(shuttleMotor.GetEncoder()) {
    _CANCoderID = carriageCANCoderID - 17;
    armEncoder.SetPositionConversionFactor(20); // 360 / 18 (ticks per revolution for the rev encoder)
    shuttleEncoder.SetPositionConversionFactor(20); 
}

void Claw::clawInit() {
    transformClaw(30, false, 0, 1);
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

int Claw::transformClaw(double desiredAngle, bool extend, double desiredShuttle, int startStage) {
    // stage denotes the starting stage the function will start at when called. 
    // stage 4 means tells Robot that the transformClaw function has completed the transformation, and 
    // will no longer call it.
    double armMotorSpeed = 0.1;
    double shuttleMotorSpeed = 0.1;
    int stage = startStage;

    // forward = true, reverse = false
    if (extend == false){
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse); // retract piston if needed
    }

    while (stage != 4) {

        switch(stage) {
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
            
            case 2: // finish rotation
            shuttleMotor.StopMotor();
            if (abs(armEncoder.GetPosition() - desiredAngle) < 3) {
                stage = 3;
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

            case 3: // extend piston if needed
            armMotor.StopMotor();
            if (extend == true){
                a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
            }
            stage = 4;
        }
    } 
}

void Claw::updateDashboard(){
    frc::SmartDashboard::PutNumber("arm absolute encoder: ", getAngle());
    frc::SmartDashboard::PutNumber("shuttle motor position: ", a_carriageMotorEncoder.GetPosition());
    if (a_Piston.Get() == frc::DoubleSolenoid::Value::kReverse){
        frc::SmartDashboard::PutString("arm solenoid position: ", "reverse");
    } else if (a_Piston.Get() == frc::DoubleSolenoid::Value::kForward){
        frc::SmartDashboard::PutString("arm solenoid position: ", "forward");
    } else {
        frc::SmartDashboard::PutString("arm solenoid position: ", "off");
    }
    
}

double Claw::getAngle(){
    return a_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID];
}


void Claw::setSolenoid(bool deployed) {
    if (deployed) {
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
    }
}

// MANY OF THESE ARE MANUAL, WILL BE REPLACED WITH AUTOMATION
void Claw::ArmPistonUp(){
    a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
}

void Claw::ArmPistonDown(){
    a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::ArmMotorUp(){
    armMotor.Set(0.1);
}

void Claw::ArmMotorDown(){
    armMotor.Set(-0.1);
}

void Claw::ClawOpen(){
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Claw::ClawClose(){
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::ConePressure(){
    a_PressureSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Claw::CubePressure(){
    a_PressureSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::ShuttleMotorUp() {
    shuttleMotor.Set(0.1);
}

void Claw::ShuttleMotorDown() {
    shuttleMotor.Set(-0.1);
}
