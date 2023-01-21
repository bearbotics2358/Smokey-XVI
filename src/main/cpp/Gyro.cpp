#include "Gyro.h"
#include "misc.h"

const uint8_t Gyro::kPowerMgmRegister;
// const uint8_t JrimmyGyro::kDataFormatRegister;
// const uint8_t JrimmyGyro::kDataRegister;
// constexpr double JrimmyGyro::kGsPerLSB;

/**
 * Constructor.
 *
 * @param port The I2C port the gyro is attached to
 */
Gyro::Gyro(Port port):
CAN(port, 0x68) {
    // uint8_t Buff[256];
    lastUpdate = 0;
    Init();
    // printf("Reg 0 is: %d", GetReg0());
    // m_i2c = new I2C((I2C::Port)port, kAddress);
    // int ret = Read(0, 1, Buff);
    // printf("Jake Buff: %2.2X\n", Buff[0] & 0x00ff);

    // Specify the data format to read
    // SetRange(range);

    // HALReport(HALUsageReporting::kResourceType_ADXL345, HALUsageReporting::kJrimmyGyro, 0);
}

Gyro::~Gyro() {
    // delete m_i2c;
    // m_i2c = NULL;
}

void Gyro::WaitForValues() {
    uint8_t stat;
    bool result;
    double start = frc::Timer::GetFPGATimestamp().value();
    double now;

    do {
        result = Read(kIntStatus, 1, &stat);
        now = frc::Timer::GetFPGATimestamp().value();
    } while ((((stat & 5) != 5) || (result == 0)) && ((now - start) < 0.100));
    // TODO: report errors/timeouts

    // might be a required time delay - cmm, 3/12/22
    // check this logic - it is very sketch (might need to test if the read failed before testing the stats) - cmm, 3/12/22
    // https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
    // https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1_i2_c.html#a329494a29c39976524e6414575515718

    // also even if this logic is right please write it so it is more eligable 
}

void Gyro::Init() {
    lastUpdate = 0;
    Write(kDLPFRegister, 0x1B);
    Write(kSampleRateDivider, 9);
    Write(kPowerMgmRegister, 1); // set to more accurate clock
    Write(kIntCfg, 5);

    Cal();
}

void Gyro::Cal() {
    // get gyro drift biases
    int i;
    double tstart = frc::Timer::GetFPGATimestamp().value();
    while (frc::Timer::GetFPGATimestamp().value() < tstart + 0.100) {
        // wait 100 ms for readings
    }

    for (i = 0; i < 3; i++) {
        angleBias[i] = 0;
    }

    // throw out first reading, it is 0
    WaitForValues();
    Update();

    for (int i = 0; i < 10; i++) {
        WaitForValues();
        Update();
        angleBias[0] += XAxis;
        angleBias[1] += YAxis;
        angleBias[2] += ZAxis;

        // printf("XAxis: %6.2lf  ", XAxis);
        // printf("YAxis: %6.2lf  ", YAxis);
        // printf("ZAxis: %6.2lf\n", ZAxis);
    }

    for (i = 0; i < 3; i++) {
        angleBias[i] /= 10;
        angle[i] = 0;
    }
    // printf("Bias read time %6.3lf\n", GetTime() - tstart);
    // printf("AngleBias: %6.3lf %6.3lf %6.3lf\n", angleBias[0], angleBias[1], angleBias[2]);
    // SmartDashboard::PutNumber("Angle Bias X", angleBias[0]);
    // SmartDashboard::PutNumber("Angle Bias Y", angleBias[1]);
    // SmartDashboard::PutNumber("Angle Bias Z", angleBias[2]);
}

uint8_t Gyro::GetReg0() {
    uint8_t id;
    Read(0, 1, &id);
    // SmartDashboard::PutNumber("Gyro ID", id);

    return id;
}

int16_t Gyro::GetReg(uint8_t regNum) {
    uint16_t ret;
    uint8_t buff[2];

    Read(regNum, 2, buff);
    ret = (buff[0] << 8) | buff[1];
    return (int16_t) ret;
}

void Gyro::Update() {
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

double Gyro::GetX() {
    return XAxis;
}

double Gyro::GetY() {
    return YAxis;
}

double Gyro::GetZ() {
    return ZAxis;
}

int Gyro::GetTemp() {
    return temperature;
}

double Gyro::GetAxisAngle(int xyz) {
    return angle[xyz];
}

double Gyro::getAngle() const {
    // update this depending on how the gyro is mounted in future years
    return angle[2];
}

double Gyro::getAngleClamped() const {
    // update this depending on how gyro is mounted in future years
    return misc::clampDegrees(angle[2]);
}

void Gyro::Zero(double offsetAngle) { //takes offsetAngle, defaults to zero if none provided. CCW is +
    for (int i = 0; i < 3; i++) {
        angle[i] = 0;
    }
    angle[2] = offsetAngle;
}

std::string Gyro::GetSmartDashboardType() {
    return "3AxisAccelerometer";
}