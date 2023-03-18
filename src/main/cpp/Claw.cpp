#include "Claw.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, 
        int pistonPullSolenoidModule, int clawPushSolenoidModule, int clawPullSolenoidModule, 
        int carriageCANCoderID, int limitSwitchId)
#ifdef COMP_BOT  // Not available on the practice bot
:
a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
a_ClawSolenoid(frc::PneumaticsModuleType::REVPH, clawPushSolenoidModule, clawPullSolenoidModule),
armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
armEncoder(armMotor.GetEncoder()),
shuttleEncoder(shuttleMotor.GetEncoder()), 
shuttleZeroSwitch(limitSwitchId),
a_CANCoder(carriageCANCoderID),
armPID(0.005,0,0), //pid set low to not ruin the robot, good speed is 0.007
shuttlePID(0.002,0,0)
#endif
{ //good pid: 0.002
#ifdef COMP_BOT  // Not available on the practice bot
    armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armPID.SetTolerance(15, 10);
    shuttlePID.SetTolerance(50, 20);

    _CANCoderID = carriageCANCoderID - 17;
    armEncoder.SetPositionConversionFactor(20); // 360 / 18 (ticks per revolution for the rev encoder)
    shuttleEncoder.SetPositionConversionFactor(42);  // just want ticks

    armMotor.StopMotor();
    shuttleMotor.StopMotor();

    a_CANCoder.ConfigMagnetOffset(CANCODER_OFFSET_ARM);
#endif
}

void Claw::clawInit() {
    //TransformClaw(51.77, false, 0);
}

void Claw::UpdateShuttleEncoder(){
#ifdef COMP_BOT  // Not available on the practice bot
    if (shuttleZeroSwitch.limitSwitchPressed() == true){
        shuttleEncoder.SetPosition(0);
    }
#endif
}


void Claw::updateDashboard(){
#ifdef COMP_BOT  // Not available on the practice bot
    frc::SmartDashboard::PutNumber("arm absolute encoder: ", getAngle());
    frc::SmartDashboard::PutNumber("shuttle motor position: ", shuttleEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("shuttle position (mm): ", GetShuttlePositionMM());
    frc::SmartDashboard::PutBoolean("limit switch pressed: ", shuttleZeroSwitch.limitSwitchPressed());
    if (a_Piston.Get() == frc::DoubleSolenoid::Value::kReverse){
        frc::SmartDashboard::PutString("arm solenoid position: ", "reverse");
    } else if (a_Piston.Get() == frc::DoubleSolenoid::Value::kForward){
        frc::SmartDashboard::PutString("arm solenoid position: ", "forward");
    } else {
        frc::SmartDashboard::PutString("arm solenoid position: ", "off");
    }
#endif
    
}

void Claw::StopShuttle(){
#ifdef COMP_BOT  // Not available on the practice bot
    shuttleMotor.StopMotor();
#endif
}

void Claw::StopArm(){
#ifdef COMP_BOT  // Not available on the practice bot
    armMotor.StopMotor();
#endif
}

double Claw::getAngle(){
#ifdef COMP_BOT  // Not available on the practice bot
    return a_CANCoder.GetAbsolutePosition();
#else
    return 0.0;
#endif
}


void Claw::setSolenoid(bool deployed) {
#ifdef COMP_BOT  // Not available on the practice bot
    if (deployed) {
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
    }
#endif
}

// MANY OF THESE ARE MANUAL, WILL BE REPLACED WITH AUTOMATION
void Claw::ArmPistonUp(){
#ifdef COMP_BOT  // Not available on the practice bot
    a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
#endif
}

void Claw::ArmPistonDown(){
#ifdef COMP_BOT  // Not available on the practice bot
    a_Piston.Set(frc::DoubleSolenoid::Value::kReverse);
#endif
}

void Claw::ArmMotorUp(){
#ifdef COMP_BOT  // Not available on the practice bot
    armMotor.Set(0.1);
#endif
}

void Claw::ArmMotorDown(){
#ifdef COMP_BOT  // Not available on the practice bot
    armMotor.Set(-0.1);
#endif
}

void Claw::ClawOpen(){
#ifdef COMP_BOT  // Not available on the practice bot
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
#endif
}

void Claw::ClawClose(){
#ifdef COMP_BOT  // Not available on the practice bot
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
#endif
}

void Claw::ShuttleMotorUp() {
#ifdef COMP_BOT  // Not available on the practice bot
    shuttleMotor.Set(0.1);
#endif
}

void Claw::ShuttleMotorDown() {
#ifdef COMP_BOT  // Not available on the practice bot
    shuttleMotor.Set(-0.1);
#endif
}

double Claw::GetShuttlePositionMM() {
#ifdef COMP_BOT  // Not available on the practice bot
    double ret;

    ret = shuttleEncoder.GetPosition() / SHUTTLE_TICKS_PER_MM;
    return ret;
#else
    return 0.0;
#endif
}

double Claw::GetShuttlePositionInches() {
#ifdef COMP_BOT  // Not available on the practice bot
    double ret;

    ret = GetShuttlePositionMM() / 25.4;
    return ret;
#else
    return 0.0;
#endif
}

bool Claw::IsShuttleSafeToMove(){ // shuttle is safe to move as long as the arm is within a certain range
    if ((getAngle() > 15) && (getAngle() < 165)) {
        return true;
    } else {
        return false;
    }
} 

// bool Claw::IsArmSafeToMove(){
//     if ((GetShuttlePositionMM > 15) && (getAngle() < 165)) {
//         return true;
//     } else {
//         return false;
//     }
// }

bool Claw::ShuttleMoveToMM(double targetPosition) {
#ifdef COMP_BOT  // Not available on the practice bot
    currentShuttleAngle = targetPosition;
    double motorDrive = shuttlePID.Calculate(GetShuttlePositionMM(), targetPosition);
    shuttlePID.SetSetpoint(targetPosition);
    motorDrive = std::clamp(motorDrive, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("shuttle pid: ", motorDrive);
    if (IsShuttleSafeToMove()){
        shuttleMotor.Set(motorDrive);
    }
    return shuttlePID.AtSetpoint();
#else
    return true;
#endif
}

bool Claw::ShuttleHold(){
#ifdef COMP_BOT  // Not available on the practice bot
    bool ret = false;

    ret = ShuttleMoveToMM(currentShuttleAngle);
    return ret;
#else
    return true;
#endif
}

bool Claw::ArmMoveTo(double targetPosition) {
#ifdef COMP_BOT  // Not available on the practice bot
    currentArmAngle = targetPosition;
    if (GetShuttlePositionMM() < 540) {
        targetPosition = std::clamp(targetPosition, 5.0, 175.0);
    } else {
        targetPosition = std::clamp(targetPosition, 5.0, 270.0);
    }
    double motorDrive = armPID.Calculate(getAngle(), targetPosition);
    armPID.SetSetpoint(targetPosition);
    motorDrive = std::clamp(motorDrive, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("arm pid: ", motorDrive);
    armMotor.Set(motorDrive);
    return armPID.AtSetpoint();
#else
    return true;
#endif
}

bool Claw::ArmHold(){
#ifdef COMP_BOT  // Not available on the practice bot
    bool ret = false;

    ret = ArmMoveTo(currentArmAngle);
    return ret;
#else
    return true;
#endif
}

bool Claw::TransformClaw(double desiredAngle, double desiredShuttle, bool extend){

#ifdef COMP_BOT  // Not available on the practice bot
    if (extend == false){
        a_Piston.Set(frc::DoubleSolenoid::Value::kReverse); // retract piston if needed
    }

    bool armDoneMoving = false;
    armDoneMoving = ArmMoveTo(desiredAngle);

    bool shuttleDoneMoving = false;
    shuttleDoneMoving = ShuttleMoveToMM(desiredShuttle);

    if (shuttleDoneMoving == true && armDoneMoving == true){
        if (extend == true){
            a_Piston.Set(frc::DoubleSolenoid::Value::kForward);
        }
        return true;
    }

    return false;
#else
    return true;
#endif
}

void Claw::HoldClaw(){
    ArmHold();
    ShuttleHold();
}
