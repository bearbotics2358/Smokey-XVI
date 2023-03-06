#pragma once

#include "Prefs.h"
#include "math/LinAlg.h"
#include <ctre/Phoenix.h>
#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <math.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
class SwerveModule // Handles steering and driving of each Swerve Module
{
    public:
        SwerveModule(int driveID, int steerID, int CANCoderID); // CAN IDs, analog port for steer encoder
    
        // Returns position of the distance encoder in meters
        float getDistance();
        // sets the drive encoder to 0 ticks
        void resetDriveEncoder();

        // recalibrates the relative steering encoder using the absolute steering encoder
        void resetSteerEncoder();

        // scaled angle between 0 and 360
        float getAngle();
        // get angle from relative encoder in degrees, does not take into consideration currently set zero point
        double getRelativeAngle();
        // TEMP
        // TODO: remove
        float getAbsAngleDegrees();

        void goToPosition(float meters); // Position PID control, moves drive wheel to specified position
        void steerToAng(float degrees); // Angle PID control

        // sets drive speed in percent
        void setDrivePercent(float percent);
        // sets steer speed in percent
        void setSteerPercent(float percent);

        // set wheel speed in meters per second
        float setDriveSpeed(float speed);

        // set the brake mode of the drive motor
        void setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode mode);

        // sets drive and steer p, i, and d values for pid
        void setDrivePID(double pNew, double iNew, double dNew);
        void setSteerPID(double pNew, double iNew, double dNew);

        // steers to the given targetAngle by taking the shortest possible path of rotation
        // this means the wheel may end up facing backwards
        // if that is the case, this returns true to indicate that the wheel speed should be opposite of what it would normally be
        bool adjustAngle(float targetAngle);

        // drives in the direction of the vector
        // the magnitude of the vector is the speed in meters per second
        void driveDirection(Vec2 direction);

        // temp
        // TODO: remove
        double getAbsEncoderVolts() const;

        void debugSteer(float angle);


    private:
        // speed is meters per second
        static double wheelSpeedToRpm(double speed);
        static double metersToMotorTicks(double meters);
        static double motorTicksToMeters(double motorTicks);

        TalonFX driveMotor;
        TalonFX steerMotor;

        TalonFXSensorCollection driveEnc; // built in TalonFX sensors
        TalonFXSensorCollection steerEncFalcon;
        CANCoder m_CANCoder;

        frc2::PIDController steerPID;

        int _steerID = 0;
        int _CANCoderID = 0;
        // how many degrees away from the actual zero degrees
        // that the relative encoder's zero point is
        double encZeroPoint { 0.0 };
};