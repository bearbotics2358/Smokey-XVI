
#pragma once

#include <frc/I2C.h>
#include <frc/Timer.h>
#include <hal/HAL.h>

class JrimmyGyro : public frc::I2C {
    protected:
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
            kDataFormat_Justify = 0x04 };

    public:
        enum Axes { kAxis_X = 0x00,
            kAxis_Y = 0x02,
            kAxis_Z = 0x04 };
        double temp;
        double XAxis;
        double YAxis;
        double ZAxis;

    public:
        explicit JrimmyGyro(Port port);
        virtual ~JrimmyGyro();

        // Gyro interface
        void WaitForValues();
        virtual void Init();
        void Cal();
        uint8_t GetReg0();
        virtual int16_t GetReg(uint8_t regNum);
        virtual void Update();
        virtual double GetX();
        virtual double GetY();
        virtual double GetZ();
        virtual int GetTemp();
        double GetAxisAngle(int axis = 1);
        // only use this method to get angle, and getAngleClamped
        // the angle of the gyro increases when turning in a counterclockwise direction
        double getAngle() const;
        // returns the same angle as get angle, but clamped to be within 0 - 360 degrees
        double getAngleClamped() const;
        void Zero(double offsetAngle = 0); // takes offsetAngle, defaults to zero if none provided. CCW is +

        virtual std::string GetSmartDashboardType();

    protected:
        // I2C m_i2c;
    private:
        double temperature;
        double AxisX;
        double AxisY;
        double AxisZ;
        double angle[3];
        double angleBias[3];
        double lastUpdate;
};