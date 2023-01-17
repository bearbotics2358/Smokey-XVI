
#include "SwerveModule.h"
#include "misc.h"
#include <math.h>

SwerveModule::SwerveModule(int driveID, int steerID, AbsoluteEncoder&& absEncoder):
driveMotor(driveID),
steerMotor(steerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
driveEnc(driveMotor),
steerEncNEO(steerMotor.GetEncoder()),
absSteerEnc(std::move(absEncoder)),
steerPID(0, 0, 0) {
    // by default this selects the ingetrated sensor
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;

    // these settings are present in the documentation example, and since they relate to safety of motor, they are probably a good idea to include
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered

    config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_25Ms;

    // this allows the motor to actually turn, pid values are set later
    config.slot0.kP = 1.0;

    driveMotor.ConfigAllSettings(config);

    steerPID.EnableContinuousInput(0.0, 360.0);
}

float SwerveModule::getDistance() {
    return motorTicksToMeters(driveEnc.GetIntegratedSensorPosition());
}

void SwerveModule::resetDriveEncoder() {
    driveEnc.SetIntegratedSensorPosition(0);
}

void SwerveModule::resetSteerEncoder() {
    // need to subtract from 1 because the encoders face oppoosite direction
    double absAngle = 360.0 * (1.0 - absSteerEnc.getRotations());
    float relAngle = getRelativeAngle();
    encZeroPoint = absAngle - relAngle;
}

double SwerveModule::getRelativeAngle() {
    float temp = steerEncNEO.GetPosition();
    float angle = (fmod(temp, TICKS_STEERING) / TICKS_STEERING) * 360; // convert to angle in degrees

    float adjusted = angle;
    if (angle < 0) {
        adjusted += 360; // bounds to 0-360
    }

    return adjusted;
}

float SwerveModule::getAngle() {
    return misc::clampDegrees(getRelativeAngle() + encZeroPoint);
}

float SwerveModule::getAbsAngleDegrees() {
    return absSteerEnc.getRotations() * 360.0;
}

void SwerveModule::goToPosition(float meters) {
    float ticks = SwerveModule::metersToMotorTicks(meters);
    driveMotor.Set(TalonFXControlMode::Position, ticks);
}

void SwerveModule::steerToAng(float degrees) {
    float speed = std::clamp(steerPID.Calculate(getAngle(), degrees) / 270.0, -0.5, 0.5);
    steerMotor.Set(speed);
}

void SwerveModule::setDrivePercent(float percent) {
    driveMotor.Set(TalonFXControlMode::PercentOutput, percent);
}

void SwerveModule::setSteerPercent(float percent) {
    steerMotor.Set(percent);
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

double SwerveModule::getAbsEncoderVolts() const {
    return absSteerEnc.getVolts();
}