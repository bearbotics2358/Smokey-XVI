#include "Gyro.h"
#include "misc.h"
#include <frc/interfaces/Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Constructor.
 *
 * @pa_ram deviceID The CAN Device ID
 */
Gyro::Gyro(int deviceID):
a_PigeonIMU(deviceID) {
    // uint8_t Buff[256];
    lastUpdate = 0;
    Init();
}

Gyro::~Gyro() {} //did nothing in original JrimmyGyro, but it's essential for building

void Gyro::WaitForValues() {
    double start = frc::Timer::GetFPGATimestamp().value();
    double now = start;
    while (now - start > 0.1) {now = frc::Timer::GetFPGATimestamp().value();}
    //the old logic basically checked for a result or it auto-expired after 0.1 seconds. 0.1 seconds is very short, so it just waits 0.1 seconds now
}


void Gyro::Init() {
    lastUpdate = 0;
    Cal();
}

void Gyro::Cal() {
    
}

void Gyro::Update() {

    frc::SmartDashboard::PutNumber("gyro angle: ", getAngle());
    frc::SmartDashboard::PutNumber("gyro angle clamped: ", getAngleClamped());
}

double Gyro::getAngle() const {
    return getYaw();
}

double Gyro::getAngleClamped() const {
    return misc::clampDegrees(getYaw());
}
double Gyro::getYaw() const {
    return a_PigeonIMU.GetYaw();
}
double Gyro::getPitch() const {
    return a_PigeonIMU.GetPitch();
}
void Gyro::setYaw(double angle){
    a_PigeonIMU.SetYaw(angle);
}
double Gyro::getAbsoluteCompassHeading () const{
    return a_PigeonIMU.GetAbsoluteCompassHeading();
}

void Gyro::Zero(double offsetAngle) { //takes offsetAngle, defaults to zero if none provided. CCW is +
    a_PigeonIMU.SetYaw(0);
}