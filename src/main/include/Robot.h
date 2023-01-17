
#pragma once

#include "Autonomous.h"
#include "CanHandler.h"
#include "CompressorController.h"
#include "JrimmyGyro.h" // Gyro wrapper class; takes in I2C port
#include "LimitSwitch.h"
#include "SwerveDrive.h" // Swerve kinematics
#include "SwerveModule.h" // Swerve modules
#include "vision/photon.h"
#include <frc/Joystick.h> // Joystick
#include <frc/TimedRobot.h> // "Timed Robot" template
#include <frc/Timer.h>

enum class DriveBackState {
    Inactive,
    Start,
    Active,
};

class Robot : public frc::TimedRobot {
    public:
        Robot();
        void RobotInit();
        void RobotPeriodic();

        void DisabledInit();
        void DisabledPeriodic();

        // called whenever the robot transitions from disabled to either autonomous, teleop, or test
        // this means it is basically called whenever the robot is enabled
        // is not called when the robot moves between autonomous, teleop, or test
        void EnabledInit();
        // called during autonomous, teleop, and test periodic
        void EnabledPeriodic();

        void AutonomousInit();
        void AutonomousPeriodic();

        void TeleopInit();
        void TeleopPeriodic();

        void TestInit();
        void TestPeriodic();

    private:
        // keeps track of when to call enabled init
        bool a_doEnabledInit { true };

        JrimmyGyro a_Gyro;

        SwerveModule a_FLModule;
        SwerveModule a_FRModule;
        SwerveModule a_BLModule;
        SwerveModule a_BRModule;
        SwerveDrive a_SwerveDrive;

        // speed multiplier for driver controls for the swerve
        bool a_slowSpeed { false };

        Autonomous a_Autonomous;

        frc::Joystick joystickOne; // 3D flightstick (Logitech Attack 3?)
        frc::Joystick a_XboxController;

        CompressorController a_CompressorController;

        // CanHandler a_canHandler;

        // stuff that autonomous needs

        TargetTracker a_shooterVision;
        TargetTracker a_ballTracker;
};
