
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
#include "Arm.h"





/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_Arm(ARM_PUSH_SOLENOID_MODULE, ARM_PULL_SOLENOID_MODULE, ARM_OPEN_SOLENOID_MODULE, ARM_CLOSE_SOLENOID_MODULE, ARM_CARRIAGE_MOTOR, ARM_CLAW_MOTOR, ARM_CARRIAGE_CANCODER), //Get the IDs for the arms solenoids
a_FLModule(misc::GetFLDrive(), misc::GetFLSteer(), AbsoluteEncoder(FL_SWERVE_ABS_ENC_PORT, FL_SWERVE_ABS_ENC_MIN_VOLTS, FL_SWERVE_ABS_ENC_MAX_VOLTS, FL_SWERVE_ABS_ENC_OFFSET / 360), misc::GetFLCANCoder()),
a_FRModule(misc::GetFRDrive(), misc::GetFRSteer(), AbsoluteEncoder(FR_SWERVE_ABS_ENC_PORT, FR_SWERVE_ABS_ENC_MIN_VOLTS, FR_SWERVE_ABS_ENC_MAX_VOLTS, FR_SWERVE_ABS_ENC_OFFSET / 360), misc::GetFRCANCoder()),
a_BLModule(misc::GetBLDrive(), misc::GetBLSteer(), AbsoluteEncoder(BL_SWERVE_ABS_ENC_PORT, BL_SWERVE_ABS_ENC_MIN_VOLTS, BL_SWERVE_ABS_ENC_MAX_VOLTS, BL_SWERVE_ABS_ENC_OFFSET / 360), misc::GetBLCANCoder()),
a_BRModule(misc::GetBRDrive(), misc::GetBRSteer(), AbsoluteEncoder(BR_SWERVE_ABS_ENC_PORT, BR_SWERVE_ABS_ENC_MIN_VOLTS, BR_SWERVE_ABS_ENC_MAX_VOLTS, BR_SWERVE_ABS_ENC_OFFSET / 360), misc::GetBRCANCoder()),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_Autonomous(&a_Gyro, &a_XboxController, &a_SwerveDrive, &a_Arm),
joystickOne(JOYSTICK_PORT),
a_XboxController(XBOX_CONTROLLER),
a_CompressorController()
// NEEDED A PORT, THIS IS PROBABLY WRONG, PLEASE FIX IT LATER
//  handler("169.254.179.144", "1185", "data"),
//  handler("raspberrypi.local", 1883, "PI/CV/SHOOT/DATA"),
//  a_canHandler(CanHandler::layout2022()),
{
    /*if (!handler.ready()) {
        // do something if handler failed to connect
    }*/

    a_FLModule.setDrivePID(0.001, 0, 0);
    a_FLModule.setSteerPID(0.6, 1.0, 0.06);

    a_FRModule.setDrivePID(0.001, 0, 0);
    a_FRModule.setSteerPID(0.6, 1.0, 0.06);

    a_BLModule.setDrivePID(0.001, 0, 0);
    a_BLModule.setSteerPID(0.6, 1.0, 0.06);

    a_BRModule.setDrivePID(0.001, 0, 0);
    a_BRModule.setSteerPID(0.6, 1.0, 0.06);

    a_SwerveDrive.brakeOnStop();
}

void Robot::RobotInit() {
    frc::SmartDashboard::init();
    a_Gyro.Init();
    a_Gyro.Zero();
}

void Robot::RobotPeriodic() {
    a_Gyro.Update();
    a_Arm.updateDashboard();
    //a_SwerveDrive.updatePosition();

//testing code block for PID tuning

    if(joystickOne.GetRawButton(3)) {
        a_FRModule.steerToAng(120);
        a_FLModule.steerToAng(120);
        a_BRModule.steerToAng(120);
        a_BLModule.steerToAng(120);
    } 
    else {
        a_FRModule.steerToAng(150);
        a_FLModule.steerToAng(150);
        a_BRModule.steerToAng(150);
        a_BLModule.steerToAng(150);
    }



    
    //printf("beam: %f/n", bstate);
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
    //a_Autonomous.DecidePath();
   // frc::SmartDashboard::PutString("Selected Autonomous", a_Autonomous.GetCurrentPath());
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
   // a_Autonomous.StartAuto();
}

void Robot::AutonomousPeriodic() {
    EnabledPeriodic();

  //  a_Autonomous.PeriodicAuto();
}

void Robot::TeleopInit() {
    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    pChange = 0;
    iChange = 0;
    dChange = 0;

}

// main loop
void Robot::TeleopPeriodic() {
    EnabledPeriodic();

    if (joystickOne.GetRawButtonReleased(DriverButton::Button12)) {
        pChange += 0.1;
    } else if (joystickOne.GetRawButtonReleased(DriverButton::Button11)) {
        pChange -= 0.1;
    }
    if (joystickOne.GetRawButtonReleased(DriverButton::Button8)) {
        iChange += 0.1;
    } else if (joystickOne.GetRawButtonReleased(DriverButton::Button7)) {
        iChange -= 0.1;
    }
    if (joystickOne.GetRawButtonReleased(DriverButton::Button10)) {
        dChange += 0.01;
    } else if (joystickOne.GetRawButtonReleased(DriverButton::Button9)) {
        dChange -= 0.01;
    }
    
    a_FRModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    a_FLModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    a_BRModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    a_BLModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange); //P 0.6, I 1.0 D 0.06
    frc::SmartDashboard::PutNumber("P value", 0.6 + pChange);
    frc::SmartDashboard::PutNumber("I value", 1.0 + iChange);
    frc::SmartDashboard::PutNumber("D value", 0.06 + dChange);

    /* =-=-=-=-=-=-=-=-=-=-= Arm Controls =-=-=-=-=-=-=-=-=-=-= */

    if(a_XboxController.GetYButton()) {
        a_Arm.ClawMotorUp();
    }
    if(a_XboxController.GetAButton()) {
        a_Arm.ClawMotorDown();
    }
    if(a_XboxController.GetXButton()) {
        a_Arm.ClawOpen();
    }
    if(a_XboxController.GetBButton()) {
        a_Arm.ClawClose();
    }

    if(a_XboxController.GetPOV() == 90) {
        a_Arm.ArmMotorUp();
    }
    if(a_XboxController.GetPOV() == 270) {
        a_Arm.ArmMotorDown();
    }
    if(a_XboxController.GetPOV() == 0) {
        a_Arm.ArmPistonUp();
    }
    if(a_XboxController.GetPOV() == 180) {
        a_Arm.ArmPistonUp();
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
 
    float x = a_XboxController.GetLeftX();
    float y = a_XboxController.GetLeftY();
    float z = a_XboxController.GetRightX();

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
    

    //turn to the right angle for climbing
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
