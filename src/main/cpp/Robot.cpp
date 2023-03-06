
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
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonTrackedTarget.h>

//TODO: FIX LINES 68, 152-164, AND 241-261



/*~~ hi :) ~~ */
Robot::Robot():
a_Gyro(GYRO_ID),
a_Arm(ARM_PUSH_SOLENOID_MODULE, ARM_PULL_SOLENOID_MODULE, ARM_OPEN_SOLENOID_MODULE, ARM_CLOSE_SOLENOID_MODULE, ARM_CLAW_PRESSURE_CONE, ARM_CLAW_PRESSURE_CUBE, ARM_CARRIAGE_MOTOR, ARM_CLAW_MOTOR, ARM_CARRIAGE_CANCODER), //Get the IDs for the arms solenoids
a_FLModule(misc::GetFLDrive(), misc::GetFLSteer(), misc::GetFLCANCoder()),
a_FRModule(misc::GetFRDrive(), misc::GetFRSteer(), misc::GetFRCANCoder()),
a_BLModule(misc::GetBLDrive(), misc::GetBLSteer(), misc::GetBLCANCoder()),
a_BRModule(misc::GetBRDrive(), misc::GetBRSteer(), misc::GetBRCANCoder()),
a_SwerveDrive(a_FLModule, a_FRModule, a_BLModule, a_BRModule, a_Gyro),
a_Autonomous(&a_Gyro, &a_SwerveDrive, &a_Arm),
a_DriverXboxController(JOYSTICK_PORT),
a_OperatorXboxController(XBOX_CONTROLLER),
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
    a_Arm.updateDashboard();
    a_LED.Update();
    a_TOF.Update();
    //a_SwerveDrive.updatePosition();

//testing code block for PID tuning

    if(a_DriverXboxController.GetRawButton(3)) {
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

    /* =-=-=-=-=-=-=-=-=-=-= Arm Controls =-=-=-=-=-=-=-=-=-=-= */

    if (a_TOF.GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE && a_OperatorXboxController.GetRawButton(4)) {
        a_Arm.ClawClose();
        //later: move claw up into scoring position but don't score/ let go
    } 

    if(a_OperatorXboxController.GetYButton()) {
        a_Arm.ClawMotorUp();
    }
    if(a_OperatorXboxController.GetAButton()) {
        a_Arm.ClawMotorDown();
    }
    if(a_OperatorXboxController.GetXButton()) {
        a_Arm.ClawOpen();
    }
    if(a_OperatorXboxController.GetBButton()) {
        a_Arm.ClawClose();
    }

    if(a_OperatorXboxController.GetPOV() == 90) {
        a_Arm.ArmMotorUp();
    }
    if(a_OperatorXboxController.GetPOV() == 270) {
        a_Arm.ArmMotorDown();
    }
    if(a_OperatorXboxController.GetPOV() == 0) {
        a_Arm.ArmPistonUp();
    }
    if(a_OperatorXboxController.GetPOV() == 180) {
        a_Arm.ArmPistonUp();
    }

    /* =-=-=-=-=-=-=-=-=-=-= Alignment Controls =-=-=-=-=-=-=-=-=-=-= */

    if((a_DriverXboxController.GetPOV() == 270) || (a_DriverXboxController.GetPOV() == 0) || (a_DriverXboxController.GetPOV() == 90)) { //same as nick's

        float turn;
        // Query the latest result from PhotonVision
        photonlib::PhotonPipelineResult result = a_camera.GetLatestResult();
        frc::Transform3d translation;

        if (result.HasTargets()) { 
            turn = a_Gyro.getAngle(); 
            photonlib::PhotonTrackedTarget target = result.GetBestTarget();
            translation = target.GetBestCameraToTarget();
            //frc::Translation2d translation = photonlib::PhotonUtils::EstimateCameraToTargetTranslation(distance, 
            //frc::Rotation2d(units::degree_t(-target.GetYaw())));
        }

        a_SwerveDrive.goToPosition(Vec2(double(translation.X()), double(translation.Y())), turn, 50); //maxspeed 50 idk :P 
    } //else, Manual Driver Mode

    /* =-=-=-=-=-=-=-=-=-=-= Swerve Controls =-=-=-=-=-=-=-=-=-=-= */

    // dpad up for full speed,
    // down for half speed
    if (a_OperatorXboxController.GetPOV() == 0) {
        a_slowSpeed = false;
    } else if (a_OperatorXboxController.GetPOV() == 180) {
        a_slowSpeed = true;
    }

    float multiplier = 1.0;
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
    if (a_DriverXboxController.GetLeftBumper()) {
        a_Gyro.Cal();
        a_Gyro.Zero();
    }

    if (!inDeadzone) {
        a_SwerveDrive.swerveUpdate(x, y, 0.5 * z, fieldOreo);
    } else {
        a_SwerveDrive.swerveUpdate(0, 0, 0, fieldOreo);
    }

    /* =-=-=-=-=-=-=-=-=-=-= Change Cone/ Cube Mode =-=-=-=-=-=-=-=-=-=-= */

    if(a_OperatorXboxController.GetRawButton(1)) { //can change button later
        SetTargetType(target_type_enum::CONE);
    } 
    else if(a_OperatorXboxController.GetRawButton(2)) { //can change button later
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
        a_Arm.ClawConePressure();
    } else if(target_type == target_type_enum::CUBE) {
        // Set target type to CUBE
        a_LED.SetTargetType(target_type_enum::CUBE);
        a_TOF.SetTargetType(target_type_enum::CUBE);
        a_Arm.ClawCubePressure();

    }
}

int main() { return frc::StartRobot<Robot>(); } // Initiate main loop
