#include "Gyro.h"
#include "misc.h"

double memoryAngleOffset = 0;
Gyro::Gyro(int 0, std::string "canhandler"); // TODO: UPDATE ID AND NAME TO CORRECT VALUES

void Gyro::Cal() {
    BasePigeon.Calibrate();
}

double Gyro::GetAngle() {
    return BasePigeon.getAngle() - memoryAngleOffset;
}

double Gyro::GetAngleClamped() {
    return misc::clampDegrees(BasePigeon.getAngle());
}

std::string Gyro::GetSmartDashboardType(){
    return "3AxisAccelerometer";
}

void Gyro::Zero(double offsetAngle){
    double memoryAngleOffset = offsetAngle;
}

void Gyro::Update() {

}