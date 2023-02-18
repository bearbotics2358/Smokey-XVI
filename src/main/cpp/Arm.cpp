#include "Arm.h"


Arm::Arm(int pushSolenoidModule, int pullSolenoidModule):
    a_armSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule) {}

void Arm::setSolenoid(bool deployed) {
    if (deployed) {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_armSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
}