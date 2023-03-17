/* Debugging swerve module rotation? CHECK LINE 74 */

#include "SwerveModule.h"
#include "misc.h"
#include <math.h>
#include <stdio.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/sensors/CANCoder.h>

SwerveModule::SwerveModule(int driveID, int steerID, int CANCoderID):
driveMotor(driveID),
steerMotor(steerID),
driveEnc(driveMotor),
steerEncFalcon(steerMotor),
m_CANCoder(CANCoderID),
steerPID(0, 0, 0) {
    // by default this selects the ingetrated sensor
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;
    _steerID = steerID;
    _CANCoderID = CANCoderID - 17;

    steerMotor.SetInverted(true);

    // these settings are present in the documentation example, and since they relate to safety of motor, they are probably a good idea to include
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered

    config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_25Ms;

    // this allows the motor to actually turn, pid values are set later
    config.slot0.kP = 1.0;

    driveMotor.ConfigAllSettings(config);
    steerMotor.ConfigAllSettings(config);
    steerMotor.ConfigNeutralDeadband(0.001);

    steerPID.EnableContinuousInput(0.0, 360.0);
}

float SwerveModule::getDistance() {
    return motorTicksToMeters(driveEnc.GetIntegratedSensorPosition());
}

void SwerveModule::resetDriveEncoder() {
    driveEnc.SetIntegratedSensorPosition(0);
}

double SwerveModule::getRelativeAngle() {
    //float temp = steerEncFalcon.GetIntegratedSensorPosition() * -1;
    double angle = (m_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID]);
    //printf("%f\n",temp);
    //float angle = (fmod(angle, 44000) / 44000) * 360; // convert to angle in degrees -- we were getting 44000 ticks per revolution
    //if (_steerID == 8){ printf("Raw Angle: %f\n",angle); } //TODO: Delete this
    float adjusted = angle;
    if (angle < 0) {
        adjusted += 360; // bounds to 0-360
    }
    //printf("%f\n",adjusted);
    frc::SmartDashboard::PutNumber("Position Error", steerPID.GetPositionError());
    frc::SmartDashboard::PutNumber("Velocity Error", steerPID.GetVelocityError());
    return adjusted;
}

float SwerveModule::getAngle() {
    if((_CANCoderID + 17) == misc::GetBLCANCoder()) {
        double position = (m_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID]);
        //printf("position: %6.2f \n", position);
    }
    return misc::clampDegrees(getRelativeAngle() + encZeroPoint);
}

void SwerveModule::goToPosition(float meters) {
    float ticks = SwerveModule::metersToMotorTicks(-meters);
    driveMotor.Set(TalonFXControlMode::Position, ticks);
}

void SwerveModule::steerToAng(float degrees) {
    float ticks = degrees / 360 * 44000;
    //float trueticks = steerEncFalcon.GetIntegratedSensorPosition() * -1;
    double CANticks = (m_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID]);
    //float trueangle = (fmod(trueticks, 44000) / 44000) * 360;
    float speed = std::clamp(steerPID.Calculate(getAngle(), degrees) / 270.0, -0.5, 0.5);
    steerMotor.Set(TalonFXControlMode::PercentOutput, speed);
    // if(_CANCoderID == misc::GetBLCANCoder()) {
    //     int position = m_CANCoder.GetAbsolutePosition();
    //     printf("position: %6.2f \n", position);
    // }
}

// void SwerveModule::debugSteer(float angle) {
//     float ticks = angle;
//     //float trueticks = steerEncFalcon.GetIntegratedSensorPosition() * -1;
//     double CANticks = (m_CANCoder.GetAbsolutePosition() - CANCODER_OFFSETS[_CANCoderID]) * -1;
//     //float trueangle = (fmod(trueticks, 44000) / 44000) * 360;
//     // if (_steerID == 5) { 
//     //     printf("angle: %6.2f    trueangle: %6.2f   ticks: %6.2f  trueticks: %6.2f\n", angle, trueangle, ticks, trueticks); 
//     // }
//     steerMotor.Set(TalonFXControlMode::Position, angle);
// }

void SwerveModule::setDrivePercent(float percent) {
    driveMotor.Set(TalonFXControlMode::PercentOutput, percent);
}

void SwerveModule::setSteerPercent(float percent) {
    steerMotor.Set(TalonFXControlMode::PercentOutput, percent);
}

float SwerveModule::setDriveSpeed(float speed) {
    float rpm = SwerveModule::wheelSpeedToRpm(speed);

    driveMotor.Set(TalonFXControlMode::Velocity, misc::rpmToTalonVel(rpm));

    return speed;
}

void SwerveModule::setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode mode) {
    driveMotor.SetNeutralMode(mode);
}

void SwerveModule::setDrivePID(double pNew, double iNew, double dNew) {
    driveMotor.Config_kP(0, pNew);
    driveMotor.Config_kI(0, iNew);
    driveMotor.Config_kD(0, dNew);
}

void SwerveModule::setSteerPID(double pNew, double iNew, double dNew) {
    steerPID.SetP(pNew);
    steerPID.SetI(iNew);
    steerPID.SetD(dNew);
}

bool SwerveModule::adjustAngle(float targetAngle) {
    float tempCurrent = getAngle();
    float tempTarget = targetAngle;
    bool changeMade = false;

    if (tempCurrent - tempTarget > 180) {
        tempCurrent -= 360;
    } else if (tempCurrent - tempTarget < -180) {
        tempCurrent += 360;
    }
    float distOfAngle = tempTarget - tempCurrent;

    if (distOfAngle > 90) {
        tempTarget -= 180;
        changeMade = true;
    }

    if (distOfAngle < -90) {
        tempTarget += 180;
        changeMade = true;
    }

    if (tempTarget < 0) {
        tempTarget += 360;
    } else if (tempTarget > 360) {
        tempTarget -= 360;
    }

    steerToAng(tempTarget);

    return changeMade;
}

void SwerveModule::driveDirection(Vec2 direction) {
    if (adjustAngle(direction.angle())) {
        setDriveSpeed(-direction.magnitude());
    } else {
        setDriveSpeed(direction.magnitude());
    }
}

double SwerveModule::wheelSpeedToRpm(double speed) {
    double adjustedSpeed = speed / DISTANCE_ADJUSTMANT_FACTOR;
    // radians per second
    double angularVelocity = adjustedSpeed / (0.5 * WHEEL_DIAMETER);
    // convert to rpm
    double rpm = (60.0 * angularVelocity) / (2 * M_PI);
    // convert from wheel rpm to motor rpm
    return rpm * SWERVE_DRIVE_MOTOR_GEAR_RATIO;
}

double SwerveModule::metersToMotorTicks(double meters) {
    double adjustedMeters = meters / DISTANCE_ADJUSTMANT_FACTOR;
    // angular position in radians
    double angularPosition = adjustedMeters / (0.5 * WHEEL_DIAMETER);
    // convert to encoder ticks
    double ticks = (FALCON_UNITS_PER_REV * angularPosition) / (2 * M_PI);
    // scale by gear ratio
    return ticks * SWERVE_DRIVE_MOTOR_GEAR_RATIO;
}

double SwerveModule::motorTicksToMeters(double motorTicks) {
    // like ticks of the wheel
    double scaledTicks = motorTicks / SWERVE_DRIVE_MOTOR_GEAR_RATIO;
    double rotations = (scaledTicks / FALCON_UNITS_PER_REV);
    // angular position in radians
    double angularPosition = rotations * 2 * M_PI;
    return DISTANCE_ADJUSTMANT_FACTOR * angularPosition * 0.5 * WHEEL_DIAMETER;
}
