#include "Claw.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>

Claw::Claw(int armMotorId, int shuttleMotorId, int pistonPushSolenoidModule, 
        int pistonPullSolenoidModule, int clawPushSolenoidModule, int clawPullSolenoidModule, 
        int carriageCANCoderID, int limitSwitchId):
a_Piston(frc::PneumaticsModuleType::REVPH, pistonPushSolenoidModule, pistonPullSolenoidModule),
a_ClawSolenoid(frc::PneumaticsModuleType::REVPH, clawPushSolenoidModule, clawPullSolenoidModule),
armMotor(armMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
shuttleMotor(shuttleMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
armEncoder(armMotor.GetEncoder()),
shuttleEncoder(shuttleMotor.GetEncoder()), 
shuttleZeroSwitch(limitSwitchId),
a_CANCoder(carriageCANCoderID),
armPID(0.004,0,0), //pid set low to not ruin the robot, good speed is 0.007
shuttlePID(0.002,0,0) { //good pid: 0.002
    armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armPID.SetTolerance(15, 10);
    shuttlePID.SetTolerance(50, 20);

    _CANCoderID = carriageCANCoderID - 17;
    armEncoder.SetPositionConversionFactor(20); // 360 / 18 (ticks per revolution for the rev encoder)
    shuttleEncoder.SetPositionConversionFactor(42);  // just want ticks

    armMotor.StopMotor();
    shuttleMotor.StopMotor();

    a_CANCoder.ConfigMagnetOffset(CANCODER_OFFSET_ARM);
}

void Claw::clawInit() {
    //TransformClaw(51.77, false, 0);
}

void Claw::UpdateShuttleEncoder(){
    if (shuttleZeroSwitch.limitSwitchPressed() == true){
        shuttleEncoder.SetPosition(0);
    }
}


void Claw::updateDashboard(){
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
    
}

void Claw::StopShuttle(){
    shuttleMotor.StopMotor();
}

void Claw::StopArm(){
    armMotor.StopMotor();
}

double Claw::getAngle(){
    return a_CANCoder.GetAbsolutePosition();
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
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::ClawClose(){
    a_ClawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Claw::ShuttleMotorUp() {
    shuttleMotor.Set(0.1);
}

void Claw::ShuttleMotorDown() {
    shuttleMotor.Set(-0.1);
}

double Claw::GetShuttlePositionMM() {
    double ret;

    ret = shuttleEncoder.GetPosition() / SHUTTLE_TICKS_PER_MM;
    return ret;
}

double Claw::GetShuttlePositionInches() {
    double ret;

    ret = GetShuttlePositionMM() / 25.4;
    return ret;
}

bool Claw::IsShuttleSafeToMove(){ // shuttle is safe to move as long as the arm is within a certain range
    if ((getAngle() > 15) && (getAngle() < 165)) {
        return true;
    } else {
        return false;
    }
}

bool Claw::ShuttleMoveToMM(double targetPosition) {
    currentShuttleAngle = targetPosition;
    double motorDrive = shuttlePID.Calculate(GetShuttlePositionMM(), targetPosition);
    shuttlePID.SetSetpoint(targetPosition);
    motorDrive = std::clamp(motorDrive, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("shuttle pid: ", motorDrive);
    if (IsShuttleSafeToMove()){
        shuttleMotor.Set(motorDrive);
    }
    return shuttlePID.AtSetpoint();
}

bool Claw::ShuttleHold(){
    bool ret = false;

    ret = ShuttleMoveToMM(currentShuttleAngle);
    return ret;
}

bool Claw::ArmMoveTo(double targetPosition) {
    currentArmAngle = targetPosition;
    targetPosition = std::clamp(targetPosition, 5.0, 175.0);
    double motorDrive = armPID.Calculate(getAngle(), targetPosition);
    armPID.SetSetpoint(targetPosition);
    motorDrive = std::clamp(motorDrive, -1.0, 1.0);
    frc::SmartDashboard::PutNumber("arm pid: ", motorDrive);
    armMotor.Set(motorDrive);
    return armPID.AtSetpoint();
}

bool Claw::ArmHold(){
    bool ret = false;

    ret = ArmMoveTo(currentArmAngle);
    return ret;
}

bool Claw::TransformClaw(double desiredAngle, double desiredShuttle, bool extend){

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
}

void Claw::HoldClaw(){
    ArmHold();
    ShuttleHold();
}
