#include "SwerveDrive.h"

#include "math/ConstMath.h"
#include "misc.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

#ifdef NEW_SWERVE
SwerveTransform::SwerveTransform(Vec2 direction, float rotSpeed):
direction(direction),
rotSpeed(rotSpeed) {}


SwerveDrive::SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule):
m_fl(flModule),
m_fr(frModule),
m_bl(blModule),
m_br(brModule),
m_anglePid(0.014, 0.0, 0.0) {}

SwerveDrive::~SwerveDrive() {}

void SwerveDrive::update(const SwerveTransform& transform) {
    constexpr float halfLength = DRIVE_LENGTH / 2;
    constexpr float halfWidth = DRIVE_WIDTH / 2;
    constexpr float driveRadius = constSqrt(halfLength * halfLength + halfWidth * halfWidth);

    constexpr Vec2 frPosVec = Vec2(halfWidth, halfLength);
    constexpr Vec2 flPosVec = Vec2(-halfWidth, halfLength);
    constexpr Vec2 brPosVec = Vec2(halfWidth, -halfLength);
    constexpr Vec2 blPosVec = Vec2(-halfWidth, -halfLength);

    constexpr Vec2 frTurn = frPosVec.right_normal().const_as_normalized();
    constexpr Vec2 flTurn = flPosVec.right_normal().const_as_normalized();
    constexpr Vec2 brTurn = brPosVec.right_normal().const_as_normalized();
    constexpr Vec2 blTurn = blPosVec.right_normal().const_as_normalized();

    float angularVelocity = transform.rotSpeed * driveRadius;

    Vec2 frVec = frTurn * angularVelocity + transform.direction;
    Vec2 flVec = flTurn * angularVelocity + transform.direction;
    Vec2 brVec = brTurn * angularVelocity + transform.direction;
    Vec2 blVec = blTurn * angularVelocity + transform.direction;

    m_fr.driveDirection(frVec);
    m_fl.driveDirection(flVec);
    m_br.driveDirection(brVec);
    m_bl.driveDirection(blVec);
}

float SwerveDrive::getAvgDistance() const {
    return (fabs(m_fr.getDistance()) + fabs(m_fl.getDistance()) + fabs(m_br.getDistance()) + fabs(m_bl.getDistance())) / 4.0;
}

#else

SwerveDrive::SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule, Gyro& gyro):
flModule(flModule),
frModule(frModule),
blModule(blModule),
brModule(brModule),
a_gyro(gyro),
turnAnglePid(0.014, 0.0, 0.0),
crabAnglePid(1.5, 0.0, 0.01) {
    turnAnglePid.EnableContinuousInput(0.0, 360.0);
    crabAnglePid.EnableContinuousInput(0.0, 360.0);
}

void SwerveDrive::crabUpdate(float x, float y, bool fieldOriented) {
    float gyroDegrees = a_gyro.getAngleClamped();

    if (!crab) {
        holdAngle = gyroDegrees;
        crab = true;
    }

    swerveUpdateInner(x, y, crabCalcZ(holdAngle, gyroDegrees), gyroDegrees, fieldOriented);
}

void SwerveDrive::swerveUpdate(float x, float y, float z, bool fieldOriented) {
    crab = false;
    swerveUpdateInner(x, y, z, a_gyro.getAngleClamped(), fieldOriented);
}

void SwerveDrive::stop() {
    flModule.setDrivePercent(0.0);
    frModule.setDrivePercent(0.0);
    blModule.setDrivePercent(0.0);
    brModule.setDrivePercent(0.0);

    flModule.setSteerPercent(0.0);
    frModule.setSteerPercent(0.0);
    blModule.setSteerPercent(0.0);
    brModule.setSteerPercent(0.0);
}

void SwerveDrive::brakeOnStop() {
    flModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    frModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    blModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    brModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void SwerveDrive::coastOnStop() {
    flModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    frModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    blModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    brModule.setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void SwerveDrive::setHoldAngle(float degrees) {
    crab = true;
    holdAngle = degrees;
}

void SwerveDrive::unsetHoldAngle() {
    crab = false;
}

void SwerveDrive::resetDrive() {
    flModule.resetDriveEncoder();
    frModule.resetDriveEncoder();
    blModule.resetDriveEncoder();
    brModule.resetDriveEncoder();

    // flModule.resetSteerEncoder();
    // frModule.resetSteerEncoder();
    // blModule.resetSteerEncoder();
    // brModule.resetSteerEncoder();
}

void SwerveDrive::driveDistance(float distMeters, float directionDegrees) {
    flModule.steerToAng(directionDegrees);
    frModule.steerToAng(directionDegrees);
    blModule.steerToAng(directionDegrees);
    brModule.steerToAng(directionDegrees);

    flModule.goToPosition(distMeters);
    frModule.goToPosition(distMeters);
    blModule.goToPosition(distMeters);
    brModule.goToPosition(distMeters);
}

void SwerveDrive::driveDirection(float percent, float directionDegrees) {
    flModule.steerToAng(directionDegrees);
    frModule.steerToAng(directionDegrees);
    blModule.steerToAng(directionDegrees);
    brModule.steerToAng(directionDegrees);

    flModule.setDrivePercent(percent);
    frModule.setDrivePercent(percent);
    blModule.setDrivePercent(percent);
    brModule.setDrivePercent(percent);
}

float SwerveDrive::getAvgDistance() {
    return (fabs(flModule.getDistance()) + fabs(frModule.getDistance()) + fabs(blModule.getDistance()) + fabs(brModule.getDistance())) / 4.0;
}

void SwerveDrive::turnToAngle(float angle) {
    float gyroDegrees = a_gyro.getAngleClamped();
    // calculates a speed we need to go based off our current sensor and target position
    float speed = turnCalcZ(angle, gyroDegrees);

    flModule.steerToAng(135);
    frModule.steerToAng(45);
    blModule.steerToAng(225);
    brModule.steerToAng(315);

    flModule.setDrivePercent(speed);
    frModule.setDrivePercent(speed);
    blModule.setDrivePercent(speed);
    brModule.setDrivePercent(speed);
}

void SwerveDrive::goToTheDon(float speed, float direction, float distance, bool fieldOriented) {
    if (getAvgDistance() <= distance) {
        float radians = direction * M_PI / 180.0;

        float x = speed * sin(radians);
        float y = speed * cos(radians);

        crabUpdate(x, y, fieldOriented);
    } else {
        stop();
    }
}

bool SwerveDrive::goToPosition(Vec2 position, float degrees, float maxSpeed) {
    float gyroDegrees = a_gyro.getAngleClamped();
    auto relPosVector = position - a_position;
    float remainingDistance = relPosVector.magnitude();

    // create a unit vector pointing towards the point we want to go to
    auto directionVector = relPosVector.as_normalized();

    // the desired speed to go at
    // do it as sqrt of remaining distance since kinetic energy is proportional to velocity squared,
    // so with sqrt we will stop at the end
    // tune the constant in front of remainingDistance
    float speed = sqrt(0.35 * remainingDistance);

    // scale this vector by the requested speed, and slow down as we get closer to the target
    directionVector *= std::clamp((double) std::min(speed, maxSpeed), 0.0, 1.0);

    frc::SmartDashboard::PutNumber("speed", directionVector.magnitude());

    // flip sign of x because x is inverted for swerveUpdateInner
    swerveUpdateInner(-directionVector.x(), directionVector.y(), turnCalcZ(degrees, gyroDegrees), gyroDegrees, true);

    return remainingDistance < GO_TO_DIST_DONE && misc::degreesDiff(degrees, gyroDegrees) < GO_TO_ANGLE_DONE;
}

void SwerveDrive::updatePosition() {
    // get the change in position of each wheel
    float flPos = flModule.getDistance();
    float flPosChange = flPos - flLastPos;
    flLastPos = flPos;

    float frPos = frModule.getDistance();
    float frPosChange = frPos - frLastPos;
    frLastPos = frPos;

    float blPos = blModule.getDistance();
    float blPosChange = blPos - blLastPos;
    blLastPos = blPos;

    float brPos = brModule.getDistance();
    float brPosChange = brPos - brLastPos;
    brLastPos = brPos;

    // angle does not need to be clamped for creating the vector
    float gyroDegrees = a_gyro.getAngle();
    // TODO: figure out if angle for swerve turn motors is clockwise our counterclockwise
    // these angles are with 0 radians pointing in the direction of positive y, and they go counterclockwise
    float flAngle = misc::degToRad(flModule.getAngle() + gyroDegrees);
    float frAngle = misc::degToRad(frModule.getAngle() + gyroDegrees);
    float blAngle = misc::degToRad(blModule.getAngle() + gyroDegrees);
    float brAngle = misc::degToRad(brModule.getAngle() + gyroDegrees);

    // create unit vectors pointing in the direction of the wheels
    Vec2 flVec(-sin(flAngle), cos(flAngle));
    Vec2 frVec(-sin(frAngle), cos(frAngle));
    Vec2 blVec(-sin(blAngle), cos(blAngle));
    Vec2 brVec(-sin(brAngle), cos(brAngle));

    // scale the wheel vector by the position change of each wheel
    flVec *= flPosChange;
    frVec *= frPosChange;
    blVec *= blPosChange;
    brVec *= brPosChange;

    // get the position change vector
    // adding all the vectors together will cancel out the turn part, leaving just the movement part * 4, so divide by 4
    // this doesn't check if the vectors are a valid wheel configuration for swerve to work, but the drive commands ensure this should work
    // if it isn't a valid configuration, some wheels will be slipping, so we can't predict the position anyways
    a_position += (flVec + frVec + blVec + brVec) / 4.0;
}

Vec2 SwerveDrive::getPosition() const {
    return a_position;
}

void SwerveDrive::setPosition(Vec2 position) {
    a_position = position;
}

void SwerveDrive::swerveUpdateInner(float x, float y, float z, float gyroDegrees, bool fieldOriented) {
    // Makes joystick inputs field oriented
    if (fieldOriented) {
        float gyroRadians = gyroDegrees * M_PI / 180;
        float temp = y * cos(gyroRadians) + x * sin(gyroRadians);
        x = -y * sin(gyroRadians) + x * cos(gyroRadians);
        y = temp;
    }

    float r = sqrt((DRIVE_LENGTH * DRIVE_LENGTH) + (DRIVE_WIDTH * DRIVE_WIDTH)); // radius of the drive base

    float a = x - z * (DRIVE_LENGTH / r); // temp variables to simplify math
    float b = x + z * (DRIVE_LENGTH / r);
    float c = y - z * (DRIVE_WIDTH / r);
    float d = y + z * (DRIVE_WIDTH / r);

    float flSpeed = sqrt(b * b + c * c);
    float frSpeed = sqrt(b * b + d * d);
    float blSpeed = sqrt(a * a + d * d);
    float brSpeed = sqrt(a * a + c * c);

    float flAngle = atan2(b, c) * 180 / M_PI; // calculates wheel angles and converts to radians
    float frAngle = atan2(b, d) * 180 / M_PI;
    float blAngle = atan2(a, c) * 180 / M_PI;
    float brAngle = atan2(a, d) * 180 / M_PI;

    if (flAngle < 0) {
        flAngle = flAngle + 360;
    }

    if (frAngle < 0) {
        frAngle = frAngle + 360;
    }

    if (blAngle < 0) {
        blAngle = blAngle + 360;
    }

    if (brAngle < 0) {
        brAngle = brAngle + 360;
    }

    float max = std::max(std::max(frSpeed, flSpeed), std::max(brSpeed, blSpeed)); // find max speed value

    // scale inputs respectively so no speed is greater than 1
    if (max > 1) {
        flSpeed /= max;
        frSpeed /= max;
        blSpeed /= max;
        brSpeed /= max;
    }

    float scalar = 1; // scalar to adjust if speed is too high
    flSpeed *= scalar;
    frSpeed *= scalar;
    blSpeed *= scalar;
    brSpeed *= scalar;

    float currentFL = flModule.getAngle();
    float currentFR = frModule.getAngle();
    float currentBR = brModule.getAngle();
    float currentBL = blModule.getAngle();

    float deadzoneCheck = sqrt(x * x + y * y);

    /*if (deadzoneCheck < 0.15 && fabs(z) < 0.01) {
        flSpeed = 0;
        frSpeed = 0;
        blSpeed = 0;
        brSpeed = 0;

        flAngle = currentFL;
        frAngle = currentFR;
        blAngle = currentBL;
        brAngle = currentBR;
    }*/

    // update speeds and angles
    if (flModule.adjustAngle(flAngle)) {
        flModule.setDrivePercent(-flSpeed);
    } else {
        flModule.setDrivePercent(flSpeed);
    }

    if (frModule.adjustAngle(frAngle)) {
        frModule.setDrivePercent(-frSpeed);
    } else {
        frModule.setDrivePercent(frSpeed);
    }

    if (blModule.adjustAngle(blAngle)) {
        blModule.setDrivePercent(-blSpeed);
    } else {
        blModule.setDrivePercent(blSpeed);
    }

    if (brModule.adjustAngle(brAngle)) {
        brModule.setDrivePercent(-brSpeed);
    } else {
        brModule.setDrivePercent(brSpeed);
    }
}

float SwerveDrive::crabCalcZ(float angle, float gyroDegrees) {
    return std::clamp(crabAnglePid.Calculate(gyroDegrees, angle) / 270.0, -0.5, 0.5);
}

float SwerveDrive::turnCalcZ(float angle, float gyroDegrees) {
    return std::clamp(turnAnglePid.Calculate(gyroDegrees, angle), -0.2, 0.2);
}

#endif