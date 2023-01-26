#pragma once

#include <frc/controller/PIDController.h>

#include "JrimmyGyro.h"
#include "Prefs.h"
#include "SwerveModule.h"
#include "math/LinAlg.h"

#ifdef NEW_SWERVE
struct SwerveTransform {
        SwerveTransform(Vec2 direction, float rotSpeed);

        static SwerveTransform translate(Vec2 direction, float gyroAngle, bool fieldOriented = true);
        static SwerveTransform translateRotate(Vec2 direction, float rotSpeed, float gyroAngle, bool fieldOriented);

        // direction for robot to move in
        Vec2 direction;

        // speed of rotation in radians
        // if this is 0, the robot will try and hold its current angle
        float rotSpeed;
};

class SwerveDrive {
    public:
        SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule);
        ~SwerveDrive();

        void update(const SwerveTransform& transform);

        float getAvgDistance() const;

    private:
        SwerveModule& m_fl;
        SwerveModule& m_fr;
        SwerveModule& m_bl;
        SwerveModule& m_br;

        // angle to hold if rotSpeed is 0
        float m_holdAngle { 0 };

        // pid controller to use when holding angle
        frc2::PIDController m_anglePid;

        static constexpr float DRIVE_LENGTH { 29.75 };
        static constexpr float DRIVE_WIDTH { 29.75 };
};

#else

class SwerveDrive // Class to handle the kinematics of Swerve Drive
{
    public:
        SwerveDrive(SwerveModule& flModule, SwerveModule& frModule, SwerveModule& blModule, SwerveModule& brModule, JrimmyGyro& gyro);

        // TODO: change the signs of x, because the positive is left thing is wierd
        // TODO: use meters/second vector for crabUpdate and swerveUpdate, instead of x and y going from 0 to 1
        // for crab drive update and swerve drive update, +y is forward, -y is backward, x is left, and -x is right
        // crab drive is like swerve drive update, except it maintains a constant turn angle
        void crabUpdate(float x, float y, bool fieldOriented = true);
        void swerveUpdate(float x, float y, float z, bool fieldOriented);
        /*
            x = x axis on joystick
            y = y axis on joystick
            z = z axis on joystick (rotation)
            gyroDegrees = sensor
            fieldOriented = if true, translation movement is relative to the field
            if false, translational movement is relative to the front of the robot,
            and it is affected by the robot's current turn angle
        */

        // stops the robot from moving, just coasts
        void stop();

        // configures the swerve drive to brake when stopped
        void brakeOnStop();

        // configures the swerve drive to coast when stopped
        void coastOnStop();

        // sets the hold angle used by crab drive update
        void setHoldAngle(float degrees);

        // unsets the hold angle, so the next call to crabUpdate will set the hold angle to the reading from the gyro
        void unsetHoldAngle();

        // resets steering and driving encoders
        void resetDrive();

        // dist in meters and angle 0-360
        void driveDistance(float distMeters, float directionDegrees);

        /** Drives at a given percent speed in a given direction.
         *  @param percent percent output of drive motors, from -1 to 1.
         *  @param directionDegrees direction to drive in degress from 0 to 360.
         */
        void driveDirection(float percent, float directionDegrees);

        // returns the average of the total distance of the drive encoders in all 4 modules in meters
        float getAvgDistance();

        // angle is in degrees
        void turnToAngle(float angle);

        // drives at a given speed (units uknown), in a given direction in degrees, for a given distance in meters
        void goToTheDon(float speed, float direction, float distance, bool fieldOriented = true);

        // goes to the specified position in meters and the specified angle in degrees at the specified percent speed
        // returns true when it has reached the position and angle
        bool goToPosition(Vec2 position, float degrees, float maxSpeed);

        // updates the current position of the robot based on the change in the wheel positions
        void updatePosition();

        // gets the current position in meters
        Vec2 getPosition() const;

        // sets the current position in meters to the passed in value
        void setPosition(Vec2 position);

    private:
#ifdef NEW_SWERVE
        void swerveUpdateInner(Vec2 direction, float rotation, float gyroDegrees, bool fieldOriented);
#else
        // called by both crabUpdate and swerveUpdata
        // does the bulk of the swerve drive work
        // x and y are translation, z is rotation
        void swerveUpdateInner(float x, float y, float z, float gyroDegrees, bool fieldOriented);
#endif

        // uses the crab pid to calulate the required z drive to get to the specified angle
        float crabCalcZ(float angle, float gyroDegrees);

        // uses the turn pid to calculate the required z drive
        float turnCalcZ(float angle, float gyroDegrees);

        SwerveModule& flModule;
        SwerveModule& frModule;
        SwerveModule& blModule;
        SwerveModule& brModule;
        JrimmyGyro& a_gyro;

        // pid when using turn to angle
        frc2::PIDController turnAnglePid;

        // pid when using crabUpdate
        frc2::PIDController crabAnglePid;

        // if we're in crab drive mode
        bool crab;
        // angle to hold in crab drive mode
        float holdAngle;

        // current position of the robot
        Vec2 a_position { 0.0, 0.0 };
        // last position of each drive wheel in meters
        float flLastPos { 0.0 };
        float frLastPos { 0.0 };
        float blLastPos { 0.0 };
        float brLastPos { 0.0 };

        constexpr static float DRIVE_LENGTH = 29.75;
        constexpr static float DRIVE_WIDTH = 29.75;

        // for goToPosition, when the distance to the target position is within this amount, say that we are done (assuming angle is also close enough)
        constexpr static float GO_TO_DIST_DONE = 0.2;
        // for goToPosition, when the angle difference from the target angle is within this amount, say that we are done (assuming distance is also close enough)
        constexpr static float GO_TO_ANGLE_DONE = 5.0;
};
#endif