#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


Autonomous::Autonomous(Gyro *Gyro, frc::XboxController *Xbox_Controller, SwerveDrive *SwerveDrive, Arm *Arm):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_Xbox(Xbox_Controller),
a_Arm(Arm),
a_AutoState0(kBlueAutoIdle0),
a_AutoState1(kBlueAutoIdle1),
a_AutoState2(kBlueAutoIdle2) {
    autoPathMaster = k5Ball;
}
/*
void Autonomous::DecidePath() {
    if (a_Xbox->GetRawAxis(OperatorJoystick::LeftTrigger) > 0.5) {
        if (a_Xbox->GetRawButtonPressed(OperatorButton::Y)) {
            if (autoPathMaster == k0Ball) {
                autoPathMaster = k5BallVision;
            } else {
                autoPathMaster = (AutoType) (autoPathMaster - 1);
            }
        }
        if (a_Xbox->GetRawButtonPressed(OperatorButton::A)) {
            if (autoPathMaster == k5BallVision) {
                autoPathMaster = k0Ball;
            } else {
                autoPathMaster = (AutoType) (autoPathMaster + 1);
            }
        }
    }
}
*/
/*
const char *Autonomous::GetCurrentPath() {
    switch (autoPathMaster) {
        case k0Ball:
            return "0 ball auto chosen";
        case kLeft1Ball:
            return "left 1 ball auto chosen";
        case kMiddle1Ball:
            return "middle 1 ball auto chosen";
        case kRight1Ball:
            return "right 1 ball auto chosen";
        case k2Ball:
            return "2 ball auto chosen";
        case k3Ball:
            return "3 ball auto chosen";
        case k5Ball:
            return "5 ball auto chosen";
        case k5BallVision:
            return "5 ball vision auto chosen";
        default:
            return "no autonous selected, this shouldn't happen";
    }
}
*/
/*
void Autonomous::StartAuto() {
    switch (autoPathMaster) {
        case k0Ball:
            Start0Ball();
            break;
        case kLeft1Ball:
            StartLeft1Ball();
            break;
        case kMiddle1Ball:
            StartMiddle1Ball();
            break;
        case kRight1Ball:
            StartRight1Ball();
            break;
        case k2Ball:
            Start2Ball();
            break;
        case k3Ball:
        case k5Ball:
        case k5BallVision:
           
            break;
    }
}
*/
/*
void Autonomous::PeriodicAuto() {
    switch (autoPathMaster) {
        case k0Ball:
            Periodic0Ball();
            break;
        case kLeft1Ball:
        case kMiddle1Ball:
        case kRight1Ball:
            Periodic1Ball();
            break;
        case k2Ball:
            Periodic2Ball();
            break;
        case k3Ball:
            Periodic3Ball();
            break;
        case k5Ball:
            Periodic5Ball();
            break;
        case k5BallVision:
            Periodic5BallVision();
            break;
    }
}
*/

// ----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //

void Autonomous::BDGL() {
    a_AutoState0 = kBlueExtend0;
    a_Arm->ArmPistonUp();
}

void Autonomous::PeriodicBDGL() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {

        case kBlueDrop0:
            //need function to drop the piece

            break;
        case kBlueRetract0:
            a_Arm->ArmPistonDown();
            break;
        case kBlueDriveAway0:
            if (DriveDirection(1.0, 0, 0.25, false)) {
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
            //add dropping stuff
            break;

        case kBlueRetract1:
           a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway1:
            if (DriveDirection(1.0, 90, 0.25, false)) {
                nextState = kBlueAutoIdle1;
            }
            //need to actually use drivedirection
            break;

        case kBlueGoToStation1:
            if (DriveDirection(1.0, 90, 0.25, false)) {
                nextState = kBlueAutoIdle1;
            }
            //need to actually use drivedirection
            break;

        case kBlueBalance1:
            //add balance code
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
            if (DriveDirection(1.32, 133, 0.25, true)) {
                nextState = kBlueRetract2;
            }
            break;

        case kBlueRetract2:
            if (TurnToAngle(-21)) {
                nextState = kBlueDriveAway2;
                StartTimer();
            }
            break;

        case kBlueDriveAway2:
            // we might be stuck on the wall, so move to the next state after some time
            if (DriveDirection(2.23, -37, 0.25, true) || WaitForTime(5)) {
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
            //need drop code

            break;
        case kBlueRetract3:
            a_Arm -> ArmPistonDown();
            
            break;
        case kBlueDriveAway3:
            if (DriveDirection(1.0, 0, 0.25, false)) {
                nextState = kBlueAutoIdle3;
                //need the actual numbers
            }
            break;
        case kBlueGoToStation3:
            if (DriveDirection(1.0, 0, 0.25, false)) {
                nextState = kBlueAutoIdle3;
                //need the actual numbers
            }
            break; 
        case kBlueBalance3:
            //no code for balance yet
            
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
            //Drop code
            
            break;

        case kBlueRetract4:
            a_Arm->ArmPistonDown();
            
            break;

        case kBlueDriveAway4:
            // need the real drive numbers
            if (DriveDirection(1.32, 133, 0.25, true)) {
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
            //Drop Code
            break;

        case kBlueRetract5:
           //arm close code
           a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway5:
            //code to drive away
            break;

        case kBlueGoToStation5:
            //code to go to base station
            break;

        case kBlueBalance5:
            // code to balance
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
            //add code
            break;
        case kRedRetract6:
            a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway6:
            if(DriveDirection(1.0,1.0,1.0,false)) { // need real numbers
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
            //need drop code

            break;
        case kRedRetract7:
            a_Arm -> ArmPistonDown();
            
            break;
        case kRedDriveAway7:
            if (DriveDirection(1.0, 0, 0.25, false)) {
                nextState = kRedAutoIdle7;
                //need the actual numbers
            }
            break;
        case kRedGoToStation7:
            if (DriveDirection(1.0, 0, 0.25, false)) {
                nextState = kRedAutoIdle7;
                //need the actual numbers
            }
            break; 
        case kRedBalance7:
            //no code for balance yet
            
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
            //need drop code
        break;
        case kRedRetract8:
        a_Arm->ArmPistonDown();
        break;
        case kRedDriveAway8:
         if (DriveDirection(1.0, 0, 0.25, false)) {
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
            //drop code missing D:
            break;
            
        case kRedRetract9:
            a_Arm->ArmPistonDown();
            break;

        case kRedDriveAway9:
            if(DriveDirection(1.0,1.0,1.0,false)) { // need real numbers please & ty
                nextState = kRedRetract9;
            }
            break;
            
        case kRedGoToStation9:
            if(DriveDirection(1.0,1.0,1.0,false)) { // need real numbers please & ty
                nextState = kRedRetract9;
            }
            break;
        
        case kRedBalance9:
            //no balance code D:
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
            //Drop code
            
            break;

        case kRedRetract10:
            a_Arm->ArmPistonDown();
            
            break;

        case kRedDriveAway10:
            // need the real drive numbers
            if (DriveDirection(1.32, 133, 0.25, true)) {
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
            //add code
            break;
        case kRedRetract11:
            a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway11:
            if(DriveDirection(1.0,1.0,1.0,false)) { // need real numbers
                nextState = kRedRetract11;
            }
            break;
        case kRedGoToStation11:
            if(DriveDirection(1.0,1.0,1.0,false)) { // need real numbers
                nextState = kRedRetract11;
            }
            break;
        case kRedBalance11:
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