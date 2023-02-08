
#include "Robot.h"
#include "Autonomous.h"
#include "Prefs.h"
#include "buttons.h"
#include "misc.h"
#include "Gyro.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <stdio.h>
#include <frc/interfaces/Gyro.h>





/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_FLModule(FL_DRIVE_ID, FL_STEER_ID, AbsoluteEncoder(FL_SWERVE_ABS_ENC_PORT, FL_SWERVE_ABS_ENC_MIN_VOLTS, FL_SWERVE_ABS_ENC_MAX_VOLTS, FL_SWERVE_ABS_ENC_OFFSET / 360)),
a_FRModule(FR_DRIVE_ID, FR_STEER_ID, AbsoluteEncoder(FR_SWERVE_ABS_ENC_PORT, FR_SWERVE_ABS_ENC_MIN_VOLTS, FR_SWERVE_ABS_ENC_MAX_VOLTS, FR_SWERVE_ABS_ENC_OFFSET / 360)),
a_BLModule(BL_DRIVE_ID, BL_STEER_ID, AbsoluteEncoder(BL_SWERVE_ABS_ENC_PORT, BL_SWERVE_ABS_ENC_MIN_VOLTS, BL_SWERVE_ABS_ENC_MAX_VOLTS, BL_SWERVE_ABS_ENC_OFFSET / 360)),
a_BRModule(BR_DRIVE_ID, BR_STEER_ID, AbsoluteEncoder(BR_SWERVE_ABS_ENC_PORT, BR_SWERVE_ABS_ENC_MIN_VOLTS, BR_SWERVE_ABS_ENC_MAX_VOLTS, BR_SWERVE_ABS_ENC_OFFSET / 360)),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_Autonomous(&a_Gyro, &a_XboxController, &a_SwerveDrive),
joystickOne(JOYSTICK_PORT),
a_XboxController(XBOX_CONTROLLER),
a_CompressorController(),
// NEEDED A PORT, THIS IS PROBABLY WRONG, PLEASE FIX IT LATER
//  handler("169.254.179.144", "1185", "data"),
//  handler("raspberrypi.local", 1883, "PI/CV/SHOOT/DATA"),
//  a_canHandler(CanHandler::layout2022()),
a_shooterVision(SHOOTER_CAMERA_NAME, TargetTracker::Mode::target(0)),
a_ballTracker(SHOOTER_CAMERA_NAME, TargetTracker::Mode::ball(0)) {
    /*if (!handler.ready()) {
        // do something if handler failed to connect
    }*/

    a_FLModule.setDrivePID(0.001, 0, 0);
    a_FLModule.setSteerPID(0.8, 0, 0.01);

    a_FRModule.setDrivePID(0.001, 0, 0);
    a_FRModule.setSteerPID(0.8, 0, 0.01);

    a_BLModule.setDrivePID(0.001, 0, 0);
    a_BLModule.setSteerPID(0.8, 0, 0.01);

    a_BRModule.setDrivePID(0.001, 0, 0);
    a_BRModule.setSteerPID(0.8, 0, 0.01);

    a_SwerveDrive.brakeOnStop();
}

void Robot::RobotInit() {
    frc::SmartDashboard::init();
    a_Gyro.Init();
    a_Gyro.Zero();
}

void Robot::RobotPeriodic() {
    a_Gyro.Update();
    a_SwerveDrive.updatePosition();
    /*

    frc::SmartDashboard::PutNumber("Distance Driven: ", a_SwerveDrive.getAvgDistance());
    frc::SmartDashboard::PutNumber("Gyro Angle: ", a_Gyro.getAngle());
    frc::SmartDashboard::PutNumber("Gyro Yaw: ", a_Gyro.getYaw());
    frc::SmartDashboard::PutNumber("Gyro Compass: ", a_Gyro.getAbsoluteCompassHeading());
    frc::SmartDashboard::PutNumber("Robot x Position", a_SwerveDrive.getPosition().x());
    frc::SmartDashboard::PutNumber("Robot y Position", a_SwerveDrive.getPosition().y());

    frc::SmartDashboard::PutBoolean("Slow speed enabled", a_slowSpeed);

    frc::SmartDashboard::PutNumber("Tank Pressure", a_CompressorController.getTankPressure());

    */
}

void Robot::DisabledInit() {
    a_doEnabledInit = true;
    a_SwerveDrive.resetDrive();
}

void Robot::DisabledPeriodic() {
    a_Autonomous.DecidePath();
    frc::SmartDashboard::PutString("Selected Autonomous", a_Autonomous.GetCurrentPath());
}

void Robot::EnabledInit() {
    
}

void Robot::EnabledPeriodic() {
    a_CompressorController.update();
}

void Robot::AutonomousInit() {
    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    a_SwerveDrive.unsetHoldAngle();
    a_Gyro.Zero();
    a_Autonomous.StartAuto();
}

void Robot::AutonomousPeriodic() {
    EnabledPeriodic();

    a_Autonomous.PeriodicAuto();
}

void Robot::TeleopInit() {
    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    pChange = 0;
    dChange = 0;

}

// main loop
void Robot::TeleopPeriodic() {
    EnabledPeriodic();

    if (joystickOne.GetRawButtonReleased(DriverButton::Button12)) {
        pChange += 0.01;
    } else if (joystickOne.GetRawButtonReleased(DriverButton::Button11)) {
        pChange -= 0.01;
    }
    if (joystickOne.GetRawButtonReleased(DriverButton::Button10)) {
        dChange += 0.001;
    } else if (joystickOne.GetRawButtonReleased(DriverButton::Button9)) {
        dChange -= 0.001;
    }
    a_FRModule.setSteerPID(0.8 + pChange, 0, 0.01 + dChange);
    frc::SmartDashboard::PutNumber("P value", 0.8 + pChange);
    frc::SmartDashboard::PutNumber("D value", 0.01 + dChange);

    /* =-=-=-=-=-=-=-=-=-=-= Swerve Controls =-=-=-=-=-=-=-=-=-=-= */

    // dpad up for full speed,
    // down for half speed
    if (a_XboxController.GetPOV() == 0) {
        a_slowSpeed = false;
    } else if (a_XboxController.GetPOV() == 180) {
        a_slowSpeed = true;
    }

    float multiplier = 1.0;
    if (a_slowSpeed) {
        multiplier = 0.25;
    }

    float x = -1 * joystickOne.GetRawAxis(DriverJoystick::XAxis);
    float y = -1 * joystickOne.GetRawAxis(DriverJoystick::YAxis);
    float z = -1 * joystickOne.GetRawAxis(DriverJoystick::ZAxis);

    if (fabs(x) < 0.10) {
        x = 0;
    }
    if (fabs(y) < 0.10) {
        y = 0;
    }
    if (fabs(z) < 0.10) {
        z = 0;
    }

    bool inDeadzone = (sqrt(x * x + y * y) < JOYSTICK_DEADZONE) && (fabs(z) < JOYSTICK_DEADZONE); // Checks joystick deadzones

    // scale by multiplier for slow mode, do this after deadzone check
    x *= multiplier;
    y *= multiplier;
    z *= multiplier;

    // turn field oriented mode off if button 3 is pressed
    bool fieldOreo = !joystickOne.GetRawButton(DriverButton::Button3);

    // calibrate gyro
    if (joystickOne.GetRawButton(DriverButton::Button5)) {
        a_Gyro.Cal();
        a_Gyro.Zero();
    }

    if (!inDeadzone) {
        if (joystickOne.GetRawButton(DriverButton::Trigger)) {
            a_SwerveDrive.swerveUpdate(x, y, 0.5 * z, fieldOreo);
        } else {
            a_SwerveDrive.crabUpdate(x, y, fieldOreo);
        }
    } else {
            a_SwerveDrive.swerveUpdate(0, 0, 0, fieldOreo);
    }
    

    // turn to the right angle for climbing
    if (joystickOne.GetRawButton(DriverButton::Button10)) {
        a_SwerveDrive.turnToAngle(180.0);
    }
}

void Robot::TestInit() {
    TeleopInit();
    a_SwerveDrive.setPosition(Vec2(0.0, 0.0));
}


void Robot::TestPeriodic() {
    TeleopPeriodic();
}

int main() { return frc::StartRobot<Robot>(); } // Initiate main loop
