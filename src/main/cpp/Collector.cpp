#include "Collector.h"
#include "Prefs.h"
#include "misc.h"

Collector::Collector(int collectorMotorId, int indexerMotorId, int pushSolenoidModule, int pullSolenoidModule):
a_collectorMotor(collectorMotorId),
a_indexerMotor(indexerMotorId),
a_collectorSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule) {}

void Collector::setSolenoid(bool deployed) {
    if (deployed) {
        a_collectorSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_collectorSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
}

void Collector::resetSolenoid() {
    a_collectorSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // collecter in
}

void Collector::setCollectorMotorSpeed(double percent) {
    a_collectorMotor.Set(ControlMode::PercentOutput, percent);
}
void Collector::setIndexerMotorSpeed(double percent) {
    a_indexerMotor.Set(ControlMode::PercentOutput, percent);
}
bool Collector::getValue() {
    return a_collectorSolenoid.Get();
}