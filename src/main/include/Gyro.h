#pragma once

//#include <frc/CAN.h>
#include <frc/Timer.h>
#include <hal/HAL.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

class Gyro {
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
        double getYaw() const;
        double getAbsoluteCompassHeading () const;
        // returns the same angle as get angle, but clamped to be within 0 - 360 degrees
        double getAngleClamped() const;
        void Zero(double offsetAngle = 0); // takes offsetAngle, defaults to zero if none provided. CCW is +

    private:
        double angle[3];
        double angleBias[3];
        double storeAngles[3];
        double lastUpdate;
        ctre::phoenix::sensors::PigeonIMU a_PigeonIMU;
};