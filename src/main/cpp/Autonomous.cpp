#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, Arm *Arm, frc::XboxController *Xbox_Controller):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_OperatorXboxController(XBOX_CONTROLLER),
a_Arm(Arm),
a_AutoState0(kBlueAutoIdle0),
a_AutoState1(kBlueAutoIdle1),
a_AutoState2(kBlueAutoIdle2){}

//-------------------------------------Auto Stuff---------------------------------------------//
void Autonomous::DecidePath() {
    if (a_OperatorXboxController.GetLeftStickButtonPressed()) {
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::Y)) {
            autoPathMaster = BlueDropAndGoLeft;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::X)) {
            autoPathMaster = BlueChargeStationLeft;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::B)) {
            autoPathMaster = BlueDropGoMiddle;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::A)) {
            autoPathMaster = BlueChargeStationMiddle;
        }
        if (a_OperatorXboxController.GetLeftBumperPressed()) {
            autoPathMaster = BlueDropGoRight;
        }
        if (a_OperatorXboxController.GetRightBumperPressed()) {
            autoPathMaster = BlueChargeStationRight;
        }      
    }
    if (a_OperatorXboxController.GetRightStickButtonPressed()) {
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::Y)) {
            autoPathMaster = RedDropAndGoLeft;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::X)) {
            autoPathMaster = RedChargeStationLeft;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::B)) {
            autoPathMaster = RedDropGoMiddle;
        }
        if (a_OperatorXboxController.GetRawButtonPressed(OperatorButton::A)) {
            autoPathMaster = RedChargeStationMiddle;
        }
        if (a_OperatorXboxController.GetLeftBumperPressed()) {
            autoPathMaster = RedDropGoRight;
        }
        if (a_OperatorXboxController.GetRightBumperPressed()) {
            autoPathMaster = RedChargeStationRight;
        }      
    }
}


const char *Autonomous::GetCurrentPath() {
    switch (autoPathMaster) {
        case BlueDropAndGoLeft:
            return "drop and go left vroom vroom";
        case BlueChargeStationLeft:
            return "left charge station";
        case BlueDropGoMiddle:
            return "drop and go middle";
        case BlueChargeStationMiddle:
            return "charge station from";
        case BlueDropGoRight:
            return "drop and go right";
        case BlueChargeStationRight:
            return "charge station from right";
            case RedDropAndGoLeft:
            return "red drop and go left";
        case RedChargeStationLeft:
            return "red left charge station";
        case RedDropGoMiddle:
            return "drop and go middle";
        case RedChargeStationMiddle:
            return "charge station from";
        case RedDropGoRight:
            return "drop and go right";
        case RedChargeStationRight:
            return "charge station from right";
        default:
            return "no autonomous selected, this shouldn't happen";
    }
}


void Autonomous::StartAuto() {
    switch (autoPathMaster) {
        case BlueDropAndGoLeft:
            BDGL();
            break;
        case BlueChargeStationLeft:
            BCSL();
            break;
        case BlueDropGoMiddle:
            BDGM();
            break;
        case BlueChargeStationMiddle:
            BCSM();
            break;
        case BlueDropGoRight:
            BDGR();
            break;
        case BlueChargeStationRight:
            BCSR();
            break;
            case RedDropAndGoLeft:
            RDGL();
            break;
        case RedChargeStationLeft:
            RCSL();
            break;
        case RedDropGoMiddle:
            RDGM();
            break;
        case RedChargeStationMiddle:
            RCSM();
            break;
        case RedDropGoRight:
            RDGR();
            break;
        case RedChargeStationRight:
            RCSR();
            break;
        default:
            //"no auto selector";
            break;
    }
}


void Autonomous::PeriodicAuto() {
    switch (autoPathMaster) {
        case BlueDropAndGoLeft:
            PeriodicBDGL();
            break;
        case BlueChargeStationLeft:
            PeriodicBCSL();
            break;
        case BlueDropGoMiddle:
            PeriodicBDGM();
            break;
        case BlueChargeStationMiddle:
            PeriodicBCSM();
            break;
        case BlueDropGoRight:
            PeriodicBDGR();
            break;
        case BlueChargeStationRight:
            PeriodicBCSR();
            break;
            case RedDropAndGoLeft:
            PeriodicRDGL();
            break;
        case RedChargeStationLeft:
            PeriodicRCSL();
            break;
        case RedDropGoMiddle:
            PeriodicRDGM();
            break;
        case RedChargeStationMiddle:
            PeriodicRCSM();
            break;
        case RedDropGoRight:
            PeriodicRDGR();
            break;
        case RedChargeStationRight:
            PeriodicRCSR();
            break;
        default:
            break;
    }
}




// ----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //

void Autonomous::BDGL() {
    a_AutoState0 = kBlueExtend0;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicBDGL() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {

        case kBlueDrop0:
            a_Arm->ClawOpen();
            break;
        case kBlueRetract0:
            a_Arm->ArmPistonDown();
            break;
        case kBlueDriveAway0:
            if (DriveDirection(4.2926, 0, 0.05, false)) {
                nextState = kBlueAutoIdle0;
            }
            break;
    }
    a_AutoState0 = nextState;
}

void Autonomous::BCSL() {
    a_AutoState1 = kBlueExtend1;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicBCSL() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {

        case kBlueDrop1:
             a_Arm->ClawOpen();
            break;

        case kBlueRetract1:
           a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway1:
            if (DriveDirection(4.2926, 0, 0.25, false)) {
                nextState = kBlueAutoIdle1;
            }
            //need to actually use drivedirection
            break;

        case kBlueGoToStation1:
            if (DriveDirection(2.54, 90, 0.25, false)) {
                nextState = kBlueAutoIdle1;
            }
            //need to actually use drivedirection
            break;

        case kBlueBalance1:
            Balance(90);
            break;

        case kBlueWait1:
            if (WaitForTime(1)) {
                nextState = kBlueAutoIdle1;
            }
            break;
    }
    a_AutoState1 = nextState;
}

void Autonomous::BDGM() {
    a_AutoState2 = kBlueExtend2;
    a_Gyro->Zero(133);
}

void Autonomous::PeriodicBDGM() {

    AutoState2 nextState = a_AutoState2;

    switch (a_AutoState2) {
        case kBlueAutoIdle2:
            StopSwerves();

            break;

        case kBlueDrop2:
            a_Arm->ClawOpen();
            break;

        case kBlueRetract2:
            a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway2:
            // we might be stuck on the wall, so move to the next state after some time
            if (DriveDirection(4.2926, 0, 0.4, false)) {
                nextState = kBlueAutoIdle2;
            }
            break;     
    }
    a_AutoState2 = nextState;
}


void Autonomous::BCSM() 
{
    a_AutoState3 = kBlueExtend3;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicBCSM() {
    AutoState3 nextState = a_AutoState3;
   
    switch (a_AutoState3) {
        case kBlueDrop3:
            a_Arm->ClawOpen();
            break;
        case kBlueRetract3:
            a_Arm -> ArmPistonDown();
            
            break;
        case kBlueDriveAway3:
            if (DriveDirection(4.2926, 0, 0.4, false)) {
                nextState = kBlueAutoIdle3;
                //need the actual numbers
            }
            break;
        case kBlueGoToStation3:
            if(DriveDirection(.2, 180, .25, false)) {
                nextState = kBlueAutoIdle3;
                //need the actual numbers
            }
            break; 
        case kBlueBalance3:
            Balance(180);
            
            break;
    }
    a_AutoState3 = nextState;
}

void Autonomous::BDGR(){
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicBDGR() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {

        case kBlueDrop4:
            a_Arm->ClawOpen();
            break;

        case kBlueRetract4:
            a_Arm->ArmPistonDown();
            
            break;

        case kBlueDriveAway4:
            // need the real drive numbers
            if (DriveDirection(4.2926, 0, 0.3, true)) {
                nextState = kBlueRetract4;
            }
            break;
    }
    a_AutoState4 = nextState;
 }
 void Autonomous::BCSR() 
{
    a_AutoState5 = kBlueExtend5;
    a_Arm->ArmPistonUp();
}
    
void Autonomous::PeriodicBCSR() {

    AutoState5 nextState = a_AutoState5;

    switch (a_AutoState5) {
        case kBlueDrop5:
            a_Arm->ClawOpen();
            break;

        case kBlueRetract5:
           //arm close code
           a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway5:
            if (DriveDirection(4.2926, 0, 0.3, false)) {
                nextState = kBlueRetract5;
            }
            break;

        case kBlueGoToStation5:
             if (DriveDirection(2.667, -90, 0.25, false)) {
                nextState = kBlueRetract5;
            }
            break;

        case kBlueBalance5:
            Balance(-90);
            break;

    }
    a_AutoState5 = nextState;
}

void Autonomous::RDGL(){
    a_AutoState6 = kRedExtend6;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRDGL() {
    AutoState6 nextState = a_AutoState6;

    switch(a_AutoState6){
        case kRedDrop6:
            a_Arm->ClawOpen();
            break;
        case kRedRetract6:
            a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway6:
            if(DriveDirection(4.2926, 0, .3, false)) { // need real numbers
                nextState = kRedRetract6;
            }
    }
    a_AutoState6 = nextState;
}

void Autonomous::RCSL() 
{
    a_AutoState7 = kRedExtend7;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRCSL() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kRedDrop7:
            a_Arm->ClawOpen();
            break;
        case kRedRetract7:
            a_Arm -> ArmPistonDown();
            
            break;
        case kRedDriveAway7:
            if (DriveDirection(4.2926, 0, 0.3, false)) {
                nextState = kRedAutoIdle7;
                //need the actual numbers
            }
            break;
        case kRedGoToStation7:
            if (DriveDirection(2.667, 90, 0.25, false)) {
                nextState = kRedAutoIdle7;
                //need the actual numbers
            }
            break; 
        case kRedBalance7:
            Balance(90);
            
            break;
    }
    a_AutoState7 = nextState;
}

void Autonomous::RDGM(){
    a_AutoState8 =  kRedExtend8;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRDGM(){
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kRedDrop8:
            a_Arm->ClawOpen();
            break;
        case kRedRetract8:
            a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway8:
         if (DriveDirection(4.2926, 0, 0.4, false)) {
                nextState = kRedAutoIdle8;
                //need the actual numbers
            }
            break;
    }
    a_AutoState8 = nextState;
}


void Autonomous::RCSM() {
    a_AutoState9 = kRedExtend9;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRCSM() {
    
    AutoState9 nextState = a_AutoState9;
    
    switch(a_AutoState9){
        case kRedDrop9:
            a_Arm->ClawOpen();
            break;
            
        case kRedRetract9:
            a_Arm->ArmPistonDown();
            break;

        case kRedDriveAway9:
            if(DriveDirection(4.2926, 0, .4, false)) { // need real numbers please & ty
                nextState = kRedRetract9;
            }
            break;
            
        case kRedGoToStation9:
            if(DriveDirection(.2, 180, .25, false)) { // need real numbers please & ty
                nextState = kRedRetract9;
            }
            break;
        
        case kRedBalance9:
           Balance(180);
            break;
    }   
    a_AutoState9 = nextState;
}

void Autonomous::RDGR(){
    a_AutoState10 = kRedExtend10;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRDGR() {

    AutoState10 nextState = a_AutoState10;

    switch (a_AutoState10) {

        case kRedDrop10:
            a_Arm->ClawOpen();
            break;

        case kRedRetract10:
            a_Arm->ArmPistonDown();
            
            break;

        case kRedDriveAway10:
            // need the real drive numbers
            if (DriveDirection(4.2926, 0, 0.25, false)) {
                nextState = kRedAutoIdle10;
            }
            break;
    }
    a_AutoState10 = nextState;
 }

void Autonomous::RCSR() {
    a_AutoState11 = kRedExtend11;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicRCSR() {
    AutoState11 nextState = a_AutoState11;

    switch(a_AutoState11){
        case kRedDrop11:
            a_Arm->ClawOpen();
            break;
        case kRedRetract11:
            a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway11:
            if(DriveDirection(4.2926, 0, .25, false)) { // need real numbers
                nextState = kRedRetract11;
            }
            break;
        case kRedGoToStation11:
            if(DriveDirection(2.54, -90, .25,false)) { // need real numbers
                nextState = kRedRetract11;
            }//18ft wide by 11 ft 3/8 
            break;
        case kRedBalance11:
        Balance(-90);
           //need code
           break; 
        
    }
a_AutoState11 = nextState;

}


void Autonomous::StopSwerves() {
    a_SwerveDrive->stop();
}

void Autonomous::StartTimer() {
    waitTimeStart = misc::getSeconds();
}

bool Autonomous::WaitForTime(double time) {
    return misc::getSeconds() > waitTimeStart + time;
}

bool Autonomous::TurnToAngle(float angle) { // rotates bot in place to specific angle

    if (fabs(a_Gyro->getAngle() - angle) >= 1) {
        a_SwerveDrive->turnToAngle(angle);
        return false;


    } else {
        a_SwerveDrive->stop();
        return true;
    }
}

bool Autonomous::DriveDirection(double dist, double angle, double speed, bool fieldOriented) { // true is done, false is not done

    if (fabs(a_SwerveDrive->getAvgDistance()) < (dist + drivestart)) {

        if (a_SwerveDrive->getAvgDistance() > (0.80 * (dist + drivestart))) {
            a_SwerveDrive->goToTheDon(speed / 2, angle, dist, fieldOriented);

        } else {
            a_SwerveDrive->goToTheDon(speed, angle, dist, fieldOriented);
        }
        return false;

    } else {
        a_SwerveDrive->stop();
        a_SwerveDrive->unsetHoldAngle();
        return true;
    }
}





bool Autonomous::Balance(float direction) {
    a_SwerveDrive->brakeOnStop();
    float currentTime = frc::Timer::GetFPGATimestamp().value();
    double tiltAngle = a_Gyro->getPitch();
    double percentTilt = tiltAngle / 15;
    double speed = percentTilt * MAX_CLIMB_PERCENT * MAX_FREE_SPEED;
    if(startedClimb) {
        a_SwerveDrive->driveDirectionVelocity(speed, direction);
        if(abs(tiltAngle) < 1) {
            startTime = frc::Timer::GetFPGATimestamp().value();
        }
        return false;
    }
    if ((currentTime - startTime > 0.5) && (abs(tiltAngle) < 1) && startedClimb){
        return true;
    }
    else{
        a_SwerveDrive->driveDirection(MAX_CLIMB_PERCENT, direction);
        if(abs(tiltAngle) > 1){
            startedClimb = true;
        }
        return false;
    }
}