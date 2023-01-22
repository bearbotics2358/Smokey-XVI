#include "Gyro.h"
#include "misc.h"

/* External Functions of Gyro:
-Zero
-getAngle - done
-init
-update
-cal
-getAngleClamped - done

*/

const uint8_t Gyro::kPowerMgmRegister;
// const uint8_t JrimmyGyro::kDataFormatRegister;
// const uint8_t JrimmyGyro::kDataRegister;
// constexpr double JrimmyGyro::kGsPerLSB;

/**
 * Constructor.
 *
 * @param deviceID The I2C port the gyro is attached to
 */
Gyro::Gyro(int deviceID):
a_WPI_Pigeon2(0, "ee") {
    // uint8_t Buff[256];
    lastUpdate = 0;
    Init();
    // printf("Reg 0 is: %d", GetReg0());
    // m_i2c = new I2C((I2C::Port)port, kAddress);
    // int ret = Re ad(0, 1, Buff);
    // printf("Jake Buff: %2.2X\n", Buff[0] & 0x00ff);

    // Specify the data format to r ead
    // SetRange(range);

    // HALReport(HALUsageReporting::kResourceType_ADXL345, HALUsageReporting::kJrimmyGyro, 0);
}

Gyro::~Gyro() {
    // delete m_i2c;
    // m_i2c = NULL;
}

void Gyro::WaitForValues() { //internal
    double start = frc::Timer::GetFPGATimestamp().value();
    double now = start;
    while (now - start > 0.1) {now = frc::Timer::GetFPGATimestamp().value();}
    //the old logic basically checked for a result or it auto-expired after 0.1 seconds. 0.1 seconds is very short, so it just waits 0.1 seconds now
}

void Gyro::Init() { //EXTERNAL
    lastUpdate = 0;
    Write(kDLPFRegister, 0x1B);
    Write(kSampleRateDivider, 9);
    Write(kPowerMgmRegister, 1); // set to more accurate clock
    Write(kIntCfg, 5);
    //Only #3 exists elsewhere in this codebase
    Cal();
}

void Gyro::Cal() { //EXTERNAL
    // get gyro drift biases
    int i;
    double tstart = frc::Timer::GetFPGATimestamp().value();
    while (frc::Timer::GetFPGATimestamp().value() < tstart + 0.100) {} // wait 100 ms for re adings

    for (i = 0; i < 3; i++) { angleBias[i] = 0; }

    // throw out first re ading, it is 0

    WaitForValues();
    Update();
    angleBias[0] += XAxis;
    angleBias[1] += YAxis;
    angleBias[2] += ZAxis;

    // printf("XAxis: %6.2lf  ", XAxis);
    // printf("YAxis: %6.2lf  ", YAxis);
    // printf("ZAxis: %6.2lf\n", ZAxis);
    // printf("Bias re ad time %6.3lf\n", GetTime() - tstart);
    // printf("AngleBias: %6.3lf %6.3lf %6.3lf\n", angleBias[0], angleBias[1], angleBias[2]);
    // SmartDashboard::PutNumber("Angle Bias X", angleBias[0]);
    // SmartDashboard::PutNumber("Angle Bias Y", angleBias[1]);
    // SmartDashboard::PutNumber("Angle Bias Z", angleBias[2]);
}

int16_t Gyro::GetReg(uint8_t regNum) { //internal method, seems to convert 8-bit data to 16-bit?
    uint16_t ret;
    uint8_t buff[2];

    Read(regNum, 2, buff);
    ret = (buff[0] << 8) | buff[1];
    return (int16_t) ret;
}

void Gyro::Update() { //EXTERNAL
    if (lastUpdate == 0) {
        lastUpdate = frc::Timer::GetFPGATimestamp().value();
        return;
    }
    double time = frc::Timer::GetFPGATimestamp().value();
    double timeDelta = (time - lastUpdate);

    temperature = GetReg(kTempRegister);
    temperature = -13200 - temp;
    temperature = temperature / 280;
    temperature += 35;

    XAxis = GetReg(kDataRegister + kAxis_X);
    XAxis = XAxis / 14.375;

    YAxis = GetReg(kDataRegister + kAxis_Y);
    YAxis = YAxis / 14.375;

    ZAxis = GetReg(kDataRegister + kAxis_Z);
    ZAxis = ZAxis / 14.375;

    angle[0] += ((XAxis - angleBias[0]) * timeDelta);
    angle[1] += ((YAxis - angleBias[1]) * timeDelta);
    angle[2] += ((ZAxis - angleBias[2]) * timeDelta);
    lastUpdate = time;

    /* for(int i = 0; i < 3; i++) {
        if (angle[i] > 360) {
            angle[i] -= 360;
        }	else if (angle[i] < 0) {
            angle[i] += 360;
        }
     } */
    // printf("X: %f, Y: %f, Z: %f\n", angle[0], angle[1], angle[2]);
}

double Gyro::getAngle() const { //EXTERNAL
    // update this depending on how the gyro is mounted in future years
    return a_WPI_Pigeon2.GetAngle();
}

double Gyro::getAngleClamped() const { //EXTERNAL
    // update this depending on how gyro is mounted in future years
    return misc::clampDegrees(a_WPI_Pigeon2.GetAngle());
}

void Gyro::Zero(double offsetAngle) { //EXTERNAL - takes offsetAngle, defaults to zero if none provided. CCW is +
    for (int i = 0; i < 3; i++) {
        angle[i] = 0;
    }
    angle[2] = offsetAngle;
}