
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
#include "Claw.h"
#include <frc/XboxController.h>

//TODO: FIX LINES 68, 152-164, AND 241-261

/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_Claw(ARM_MOTOR, SHUTTLE_MOTOR, PISTON_PUSH_SOLENOID_MODULE, PISTON_PULL_SOLENOID_MODULE, CLAW_OPEN_SOLENOID_MODULE, CLAW_CLOSE_SOLENOID_MODULE, /*CONE_PRESSURE_SOLENOID, CUBE_PRESSURE_SOLENOID,*/ CANCODER_ID_ARM, LIMIT_SWITCH), //Get the IDs for the arms solenoids
a_FLModule(misc::GetFLDrive(), misc::GetFLSteer(), misc::GetFLCANCoder()),
a_FRModule(misc::GetFRDrive(), misc::GetFRSteer(), misc::GetFRCANCoder()),
a_BLModule(misc::GetBLDrive(), misc::GetBLSteer(), misc::GetBLCANCoder()),
a_BRModule(misc::GetBRDrive(), misc::GetBRSteer(), misc::GetBRCANCoder()),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_Autonomous(&a_Gyro, &a_SwerveDrive, &a_Claw),
a_DriverXboxController(DRIVER_PORT),
a_OperatorXboxController(OPERATOR_PORT),
a_CompressorController(),
a_TOF(), 
a_LED()
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

    m_AutoModeSelector.SetDefaultOption(RobotDoNothing, RobotDoNothing);
    m_AutoModeSelector.AddOption(BlueDropAndGoLeft, BlueDropAndGoLeft);
    m_AutoModeSelector.AddOption(BlueChargeStationLeft, BlueChargeStationLeft);
    m_AutoModeSelector.AddOption(BlueDropAndGoMiddle, BlueDropAndGoMiddle);
    m_AutoModeSelector.AddOption(BlueChargeStationMiddle, BlueChargeStationMiddle);
    m_AutoModeSelector.AddOption(BlueDropAndGoRight, BlueDropAndGoRight);
    m_AutoModeSelector.AddOption(BlueChargeStationRight, BlueChargeStationRight);
    m_AutoModeSelector.AddOption(RedDropAndGoLeft, RedDropAndGoLeft);
    m_AutoModeSelector.AddOption(RedChargeStationLeft, RedChargeStationLeft);
    m_AutoModeSelector.AddOption(RedDropAndGoMiddle, RedDropAndGoMiddle);
    m_AutoModeSelector.AddOption(RedChargeStationMiddle, RedChargeStationMiddle);
    m_AutoModeSelector.AddOption(RedDropAndGoRight, RedDropAndGoRight);
    m_AutoModeSelector.AddOption(RedChargeStationRight, RedChargeStationRight);
    frc::SmartDashboard::PutData("Auto Modes", &m_AutoModeSelector); 

    a_LED.Init();

    SetTargetType(target_type_enum::CONE);
}

void Robot::RobotPeriodic() {
    a_Gyro.Update();
    a_Claw.updateDashboard();
    a_LED.Update();
    a_TOF.Update();
    //a_SwerveDrive.updatePosition();

//testing code block for PID tuning

    // if(a_DriverXboxController.GetRawButton(3)) {
    //     a_FRModule.steerToAng(120);
    //     a_FLModule.steerToAng(120);
    //     a_BRModule.steerToAng(120);
    //     a_BLModule.steerToAng(120);
    // } 
    // else {
    //     a_FRModule.steerToAng(150);
    //     a_FLModule.steerToAng(150);
    //     a_BRModule.steerToAng(150);
    //     a_BLModule.steerToAng(150);
    // }
}

void Robot::DisabledInit() {
    a_doEnabledInit = true;
    a_SwerveDrive.resetDrive();
}
void Robot::EnabledInit(){}

void Robot::EnabledPeriodic() {
    a_CompressorController.update();
}
void Robot::DisabledPeriodic(){}


void Robot::AutonomousInit() {
    SetTargetType(target_type_enum::CONE);

    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    a_SwerveDrive.unsetHoldAngle();
    a_Gyro.Zero();
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable
  
    a_Autonomous.StartAuto(SelectedRoute); //starts auto from selected route
    
}

void Robot::AutonomousPeriodic() {
    std::string SelectedRoute = m_AutoModeSelector.GetSelected(); //assigns value frm smart dashboard to a string variable
    a_Autonomous.PeriodicAuto(SelectedRoute);
    EnabledPeriodic();
}

void Robot::TeleopInit() {
    SetTargetType(target_type_enum::CONE);

    a_Gyro.setYaw(180 + a_Gyro.getYaw());

    if (a_doEnabledInit) {
        EnabledInit();
        a_doEnabledInit = false;
    }

    // pChange = 0;
    // iChange = 0;
    // dChange = 0;

}

// main loop
void Robot::TeleopPeriodic() {
    EnabledPeriodic();

    // if (joystickOne.GetRawButtonReleased(DriverButton::Button12)) {
    //     pChange += 0.1;
    // } else if (joystickOne.GetRawButtonReleased(DriverButton::Button11)) {
    //     pChange -= 0.1;
    // }
    // if (joystickOne.GetRawButtonReleased(DriverButton::Button8)) {
    //     iChange += 0.1;
    // } else if (joystickOne.GetRawButtonReleased(DriverButton::Button7)) {
    //     iChange -= 0.1;
    // }
    // if (joystickOne.GetRawButtonReleased(DriverButton::Button10)) {
    //     dChange += 0.01;
    // } else if (joystickOne.GetRawButtonReleased(DriverButton::Button9)) {
    //     dChange -= 0.01;
    // }
    
    // a_FRModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    // a_FLModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    // a_BRModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange);
    // a_BLModule.setSteerPID(0.6 + pChange, 1.0 + iChange, 0.06 + dChange); //P 0.6, I 1.0 D 0.06
    // frc::SmartDashboard::PutNumber("P value", 0.6 + pChange);
    // frc::SmartDashboard::PutNumber("I value", 1.0 + iChange);
    // frc::SmartDashboard::PutNumber("D value", 0.06 + dChange);

    /* =-=-=-=-=-=-=-=-=-=-= Claw Controls =-=-=-=-=-=-=-=-=-=-= */
    a_Claw.UpdateShuttleEncoder(); //automatically sets the shuttle's encoder to 0 if hitting the limit switch
    if (a_TOF.GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE && a_DriverXboxController.GetYButton()) {
        a_Claw.ClawClose();
        //later: move claw up into scoring position but 
        // don't score/ let go
    } 

    // arm rotation controls
    if(a_OperatorXboxController.GetXButton()) {
        a_Claw.ArmMotorUp();
    } else if (a_OperatorXboxController.GetBButton()) {
        a_Claw.ArmMotorDown();
    } else {
        a_Claw.StopArm();
    }

    // shuttle movement controls
    //if (a_Claw.IsShuttleSafeToMove() == true) {
        if(a_OperatorXboxController.GetYButton()) {
            a_Claw.ShuttleMotorUp();
        } else if (a_OperatorXboxController.GetAButton()) {
            a_Claw.ShuttleMotorDown();
        } else {
            a_Claw.StopShuttle();
        }
    //}   

    // piston extension controls
    // if(a_OperatorXboxController.GetPOV() == 270) { // left
    //     a_Claw.ArmPistonUp();
    // } else if (a_OperatorXboxController.GetPOV() == 90) { // right
    //     a_Claw.ArmPistonDown();
    // }

    // claw open/close controls
    if(a_DriverXboxController.GetRightBumper()) {
        a_Claw.ClawOpen();
    } else if (a_DriverXboxController.GetLeftBumper()) {
        a_Claw.ClawClose();
    }

    /* =-=-=-=-=-=-=-=-=-=-= Alignment Controls =-=-=-=-=-=-=-=-=-=-= */

    if((a_DriverXboxController.GetPOV() == 270) || (a_DriverXboxController.GetPOV() == 0) || (a_DriverXboxController.GetPOV() == 90)) {
        photonlib::PhotonPipelineResult result = a_camera.GetLatestResult();
        double angle = a_Gyro.getAngle();
        if(result.HasTargets()){
            units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(TARGET_CAMERA_HEIGHT, TARGET_HEIGHT, TARGET_CAMERA_PITCH, units::degree_t{result.GetBestTarget().GetPitch()});
            units::meter_t xComponent = range * sin(angle);
            units::meter_t yComponent = (range * cos(angle)) - units::meter_t(0.36195);
            if(a_DriverXboxController.GetPOV() == 270){ // go to cone spot to left of target
                newXComponent = xComponent - units::meter_t(.5588);
            }
            else if(a_DriverXboxController.GetPOV() == 0){ // go to cube spot in line with target
                newXComponent = xComponent;
            }
            else if(a_DriverXboxController.GetPOV() == 90){ // go to cone spot to right of target
                newXComponent = xComponent + units::meter_t(.5588);
            }
            a_SwerveDrive.goToPosition(Vec2(double(newXComponent), double(yComponent)), 0, 0.2);
        }
    }

    /* =-=-=-=-=-=-=-=-=-=-= Swerve Controls =-=-=-=-=-=-=-=-=-=-= */

    // dpad up for full speed,
    // down for half speed
    if (a_DriverXboxController.GetPOV() == 0) {
        a_slowSpeed = false;
    } else if (a_DriverXboxController.GetPOV() == 180) {
        a_slowSpeed = true;
    }

    float multiplier = 1.25;
    if (a_slowSpeed) {
        multiplier = 0.25;
    }
 
    float x = a_DriverXboxController.GetLeftX();
    float y = a_DriverXboxController.GetLeftY();
    float z = a_DriverXboxController.GetRightX();

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
    bool fieldOreo = true; // !joystickOne.GetRawButton(DriverButton::Button3);

    // calibrate gyro
    if (a_DriverXboxController.GetAButton()) {
        a_Gyro.Cal();
        a_Gyro.Zero();
    }

    if (!inDeadzone) {
        a_SwerveDrive.swerveUpdate(x, y, 0.5 * z, fieldOreo);
    } else {
        a_SwerveDrive.swerveUpdate(0, 0, 0, fieldOreo);
    }

    /* =-=-=-=-=-=-=-=-=-=-= Change Cone/ Cube Mode =-=-=-=-=-=-=-=-=-=-= */

    if(a_DriverXboxController.GetRawButton(1)) { //can change button later
        SetTargetType(target_type_enum::CONE);
    } 
    else if(a_DriverXboxController.GetRawButton(2)) { //can change button later
        SetTargetType(target_type_enum::CUBE);
    }
}

void Robot::TestInit() {
    TeleopInit();
    a_SwerveDrive.setPosition(Vec2(0.0, 0.0));
}


void Robot::TestPeriodic() {
    TeleopPeriodic();
}

void Robot::SetTargetType(target_type_enum target) {
    target_type = target;
    if(target_type == target_type_enum::CONE) {
        // Set target type to CONE
        a_LED.SetTargetType(target_type_enum::CONE);
        a_TOF.SetTargetType(target_type_enum::CONE);
        //a_Claw.ConePressure();
    } else if(target_type == target_type_enum::CUBE) {
        // Set target type to CUBE
        a_LED.SetTargetType(target_type_enum::CUBE);
        a_TOF.SetTargetType(target_type_enum::CUBE);
        //a_Claw.CubePressure();

    }
}

int main() { return frc::StartRobot<Robot>(); } // Initiate main loop
