#include "Climber.h"
#include "Prefs.h"
#include "misc.h"
#include <iostream>

Climber::Climber(int climberMotorId, int pushSolenoidModule, int pullSolenoidModule):
a_climberArmMotor(climberMotorId),
a_climberSolenoid(frc::PneumaticsModuleType::REVPH, pushSolenoidModule, pullSolenoidModule) {
    // by default this selects the ingetrated sensor
    // do this to set kp value
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;

    // these settings are present in the documentation example, and since they relate to safety of motor, they are probably a good idea to include
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered

    config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_25Ms;

    // FIXME: pid tune
    config.slot0.kP = 1.0;

    a_climberArmMotor.ConfigAllSettings(config);

    a_climberArmMotor.SetNeutralMode(NeutralMode::Brake);
    a_climberArmMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
}


void Climber::setArmSpeed(double percent) { // sets the power/speed of the climber arm by percent output mode
    a_climberArmMotor.Set(ControlMode::PercentOutput, percent);
}

void Climber::setSolenoid(bool deployed) {
    if (deployed) {
        a_climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        a_climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
}

void Climber::resetClimber() {
    a_climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // kForward means the small arms are inside the robot frame, fReverse means they are extended
    a_climberArmMotor.GetSensorCollection().SetIntegratedSensorPosition(0, 0); // reset arm motor encoder position to 0
}
double Climber::getTicks() {
    return a_climberArmMotor.GetSensorCollection().GetIntegratedSensorAbsolutePosition() * 0.1;
}
double Climber::getHeight() { // returns the height of the arm in millimeters
    double ticks = getTicks();
    return ticks * CLIMBER_MM_PER_TICK;
}

double Climber::getSpeed() { // returns the speed at which the climber arm is moving in millimeters per second
    double ticks = a_climberArmMotor.GetSensorCollection().GetIntegratedSensorVelocity();
    return ticks * CLIMBER_MM_PER_TICK * 10;
}
