
#include "Robot.h"
#include "Autonomous.h"
#include "Climber.h"
#include "Prefs.h"
#include "buttons.h"
#include "misc.h"
#include <JrimmyGyro.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <stdio.h>

/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(frc::I2C::kMXP),
a_FLModule(FL_DRIVE_ID, FL_STEER_ID, AbsoluteEncoder(FL_SWERVE_ABS_ENC_PORT, FL_SWERVE_ABS_ENC_MIN_VOLTS, FL_SWERVE_ABS_ENC_MAX_VOLTS, FL_SWERVE_ABS_ENC_OFFSET / 360)),
a_FRModule(FR_DRIVE_ID, FR_STEER_ID, AbsoluteEncoder(FR_SWERVE_ABS_ENC_PORT, FR_SWERVE_ABS_ENC_MIN_VOLTS, FR_SWERVE_ABS_ENC_MAX_VOLTS, FR_SWERVE_ABS_ENC_OFFSET / 360)),
a_BLModule(BL_DRIVE_ID, BL_STEER_ID, AbsoluteEncoder(BL_SWERVE_ABS_ENC_PORT, BL_SWERVE_ABS_ENC_MIN_VOLTS, BL_SWERVE_ABS_ENC_MAX_VOLTS, BL_SWERVE_ABS_ENC_OFFSET / 360)),
a_BRModule(BR_DRIVE_ID, BR_STEER_ID, AbsoluteEncoder(BR_SWERVE_ABS_ENC_PORT, BR_SWERVE_ABS_ENC_MIN_VOLTS, BR_SWERVE_ABS_ENC_MAX_VOLTS, BR_SWERVE_ABS_ENC_OFFSET / 360)),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_Autonomous(&a_Gyro, &a_XboxController, &a_SwerveDrive, &a_Shooter, &a_Collector),
joystickOne(JOYSTICK_PORT),
a_XboxController(XBOX_CONTROLLER),
a_Shooter(LEFT_SHOOTER_ID, RIGHT_SHOOTER_ID),
a_Collector(COLLECTOR_MOTOR_ID, INDEXER_MOTOR_ID, COLLECTOR_PUSH_SOLENOID_MODULE, COLLECTOR_PULL_SOLENOID_MODULE),
a_LimitSwitch(CLIMBER_SWITCH_PORT),
a_Climber(CLIMBER_MOTOR_ID, CLIMBER_PUSH_SOLENOID_MODULE, CLIMBER_PULL_SOLENOID_MODULE),
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
    a_FLModule.setSteerPID(2.0, 0, 0.01);

    a_FRModule.setDrivePID(0.001, 0, 0);
    a_FRModule.setSteerPID(2.0, 0, 0.01);

    a_BLModule.setDrivePID(0.001, 0, 0);
    a_BLModule.setSteerPID(2.0, 0, 0.01);

    a_BRModule.setDrivePID(0.001, 0, 0);
    a_BRModule.setSteerPID(2.0, 0, 0.01);

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

    frc::SmartDashboard::PutNumber("Distance Driven: ", a_SwerveDrive.getAvgDistance());
    frc::SmartDashboard::PutNumber("Gyro Angle: ", a_Gyro.getAngle());
    frc::SmartDashboard::PutNumber("Robot x Position", a_SwerveDrive.getPosition().x());
    frc::SmartDashboard::PutNumber("Robot y Position", a_SwerveDrive.getPosition().y());

    frc::SmartDashboard::PutBoolean("Slow speed enabled", a_slowSpeed);
    frc::SmartDashboard::PutBoolean("Collector Solenoid Toggle: ", a_Collector.getValue());

    frc::SmartDashboard::PutNumber("Tank Pressure", a_CompressorController.getTankPressure());

    frc::SmartDashboard::PutNumber("Current Shooter RPM", a_Shooter.getSpeed());

    frc::SmartDashboard::PutNumber("Climber Arm Height (mm)", a_Climber.getHeight());
    frc::SmartDashboard::PutNumber("Climber Arm Speed (mm/s)", a_Climber.getSpeed());
    frc::SmartDashboard::PutNumber("Climber Arm Ticks Raised", a_Climber.getTicks());
    frc::SmartDashboard::PutNumber("Climber Limit Switch Pressed", a_LimitSwitch.limitSwitchPressed());
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
    a_Collector.resetSolenoid();
    a_Climber.resetClimber();
}

void Robot::EnabledPeriodic() {
    a_shooterVision.update();
    a_ballTracker.update();
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
}

// main loop
void Robot::TeleopPeriodic() {
    EnabledPeriodic();

    /* =-=-=-=-=-=-=-=-=-=-= Climber Controls =-=-=-=-=-=-=-=-=-=-= */

    if (joystickOne.GetRawButton(DriverButton::Button8)) { // arms up
        a_Climber.setArmSpeed(CLIMBER_MOTOR_PERCENT_OUTPUT);
    } else if (joystickOne.GetRawButton(DriverButton::Button7)) { // arms down
        a_Climber.setArmSpeed(-CLIMBER_MOTOR_PERCENT_OUTPUT);
    } else {
        a_Climber.setArmSpeed(0);
    }
    if (joystickOne.GetRawButtonPressed(DriverButton::Button6)) {
        a_Climber.setSolenoid(true); // arms out
    }
    if (joystickOne.GetRawButtonPressed(DriverButton::Button4)) {
        a_Climber.setSolenoid(false); // arms in
    }


    /* Limit Switch Automatic Climb

    if (a_LimitSwitch.limitSwitchPressed() == true){
        a_Climber.setSolenoid(frc::DoubleSolenoid::Value::kReverse);
    }

    */



    /* =-=-=-=-=-=-=-=-=-=-= Shooter Controls =-=-=-=-=-=-=-=-=-=-= */

    if (joystickOne.GetRawButton(DriverButton::ThumbButton)) {
        a_Collector.setIndexerMotorSpeed(INDEXER_MOTOR_PERCENT_OUTPUT);
    } else {
        a_Collector.setIndexerMotorSpeed(0);
    }
    if (joystickOne.GetRawButton(DriverButton::Button11)) {
        a_Shooter.stop();
    }
    if (joystickOne.GetRawButton(DriverButton::Button12)) {
        a_Shooter.setSpeed(SHOOTER_SPEED);
    }
    if (joystickOne.GetRawButton(DriverButton::Button9)) {
        a_Shooter.setSpeed(LOW_SHOOTER_SPEED);
    }

    /* =-=-=-=-=-=-=-=-=-=- Collector Controls -=-=-=-=-=-=-=-=-=-= */

    if (a_XboxController.GetRawButtonPressed(OperatorButton::LeftBumper)) {
        a_Collector.setSolenoid(false); // collecter in
    }
    if (a_XboxController.GetRawButtonPressed(OperatorButton::RightBumper)) {
        a_Collector.setSolenoid(true); // collecter out
    }
    if (a_XboxController.GetRawButton(OperatorButton::Y)) {
        a_Collector.setCollectorMotorSpeed(COLLECTOR_MOTOR_PERCENT_OUTPUT);
    } else if (a_XboxController.GetRawButton(OperatorButton::X)) {
        a_Collector.setCollectorMotorSpeed(-COLLECTOR_MOTOR_PERCENT_OUTPUT);
    } else {
        a_Collector.setCollectorMotorSpeed(0);
    }

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
    a_Shooter.stop();
}


void Robot::TestPeriodic() {
    TeleopPeriodic();
}

int main() { return frc::StartRobot<Robot>(); } // Initiate main loop
