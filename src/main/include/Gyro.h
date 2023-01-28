#pragma once

//#include <frc/CAN.h>
#include <frc/Timer.h>
#include <hal/HAL.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

class Gyro {
    /* protected:
        static const uint8_t kPowerMgmRegister = 0x3E;
        // static const uint8_t kDataFormatRegister = 0x31;
        static const uint8_t kSampleRateDivider = 0x15;
        static const uint8_t kDLPFRegister = 0x16;
        static const uint8_t kTempRegister = 0x1B;
        static const uint8_t kDataRegister = 0x1D;
        static const uint8_t kIntCfg = 0x17;
        static const uint8_t kIntStatus = 0x1A;
        static constexpr double kGsPerLSB = 0.00390625;
        enum DataFormatFields { kDataFormat_SelfTest = 0x80,
            kDataFormat_SPI = 0x40,
            kDataFormat_IntInvert = 0x20,
            kDataFormat_FullRes = 0x08,
            kDataFormat_Justify = 0x04 }; */

    public:
        double XAxis;
        double YAxis;
        double ZAxis;

    public:
        explicit Gyro(int deviceID);
        virtual ~Gyro();

        // Gyro interface
        void WaitForValues();
        virtual void Init();
        void Cal();
        virtual void Update();
        // only use this method to get angle, and getAngleClamped
        // the angle of the gyro increases when turning in a counterclockwise direction
        double getAngle() const;
        // returns the same angle as get angle, but clamped to be within 0 - 360 degrees
        double getAngleClamped() const;
        void Zero(double offsetAngle = 0); // takes offsetAngle, defaults to zero if none provided. CCW is +

        virtual std::string GetSmartDashboardType();
    private:
        double AxisX;
        double AxisY;
        double AxisZ;
        double angle[3];
        double angleBias[3];
        double lastUpdate;
        ctre::phoenix::sensors::PigeonIMU a_PigeonIMU;
};