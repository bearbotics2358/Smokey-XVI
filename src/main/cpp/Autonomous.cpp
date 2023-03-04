#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_OperatorXboxController(XBOX_CONTROLLER),
//a_Arm(Arm),


a_AutoState0(kBlueAutoIdle0),
a_AutoState1(kBlueAutoIdle1),
a_AutoState2(kBlueAutoIdle2){}

//-------------------------------------Auto Stuff---------------------------------------------//
void Autonomous::StartAuto(const std::string autoMode) {
    a_AutoSelected = autoMode; 
}




// s----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //

void Autonomous::BDGL() {
    a_AutoState0 = kBlueExtend0;
}

void Autonomous::PeriodicBDGL() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {
        case kBlueAutoIdle0:
            StopSwerves();
            break;
        case kBlueExtend0:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop0:
            //a_Arm->ClawOpen();
            break;
        case kBlueRetract0:
            //a_Arm->ArmPistonDown();
            break;
        case kBlueDriveAway0:
            if (DriveDirection(4.8768, 0, .25, false)) {
                nextState = kBlueAutoIdle0;
            }
            break;
    }
    a_AutoState0 = nextState;
}

void Autonomous::BCSL() {
    a_AutoState1 = kBlueExtend1;
}

void Autonomous::PeriodicBCSL() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {
         case kBlueAutoIdle1:
            StopSwerves();
            break;
        case kBlueExtend1:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop1:
            //a_Arm->ClawOpen();
            break;

        case kBlueRetract1:
            //a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway1:
            if (DriveDirection(3.6576, 0, 0.25, false)) {
                nextState = kBlueAutoIdle1;
            }
            //need to actually use drivedirection
            break;

        case kBlueGoToStation1:
            if (DriveDirection(2.667, 90, 0.25, false)) {
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
}

void Autonomous::PeriodicBDGM() {

    AutoState2 nextState = a_AutoState2;

    switch (a_AutoState2) {
       case kBlueAutoIdle2:
            StopSwerves();
            break;
        case kBlueExtend2:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop2:
            //a_Arm->ClawOpen();
            break;
        case kBlueRetract2:
            //a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway2:
            // we might be stuck on the wall, so move to the next state after some time
            if (DriveDirection(4.8768, 0, 0.4, false)) {
                nextState = kBlueAutoIdle2;
            }
            break;     
    }
    a_AutoState2 = nextState;
}


void Autonomous::BCSM() 
{
    a_AutoState3 = kBlueExtend3;
}

void Autonomous::PeriodicBCSM() {
    AutoState3 nextState = a_AutoState3;
   
    switch (a_AutoState3) {
         case kBlueAutoIdle3:
            StopSwerves();
            break;
        case kBlueExtend3:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop3:
            //a_Arm->ClawOpen();
            break;
        case kBlueRetract3:
            //a_Arm -> ArmPistonDown();
            break;
        case kBlueDriveAway3:
            if (DriveDirection(3.6576, 0, 0.4, false)) {
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
    a_AutoState4 = kBlueExtend4;
}

void Autonomous::PeriodicBDGR() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {
         case kBlueAutoIdle4:
            StopSwerves();
            break;
        case kBlueExtend4:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop4:
            //a_Arm->ClawOpen();
            break;

        case kBlueRetract4:
            //a_Arm->ArmPistonDown();
            
            break;

        case kBlueDriveAway4:
            // need the real drive numbers
            if (DriveDirection(4.8768, 0, 0.3, true)) {
                nextState = kBlueRetract4;
            }
            break;
    }
    a_AutoState4 = nextState;
 }
 void Autonomous::BCSR() 
{
    a_AutoState5 = kBlueExtend5;
}
    
void Autonomous::PeriodicBCSR() {

    AutoState5 nextState = a_AutoState5;

    switch (a_AutoState5) {
        case kBlueAutoIdle5:
            StopSwerves();
            break;
        case kBlueExtend5:
            //a_Arm->ArmPistonUp();
            break;
        case kBlueDrop5:
            //a_Arm->ClawOpen();
            break;

        case kBlueRetract5:
           //arm close code
            //a_Arm->ArmPistonDown();
            break;

        case kBlueDriveAway5:
            if (DriveDirection(3.6576, 0, 0.3, false)) {
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
}

void Autonomous::PeriodicRDGL() {
    AutoState6 nextState = a_AutoState6;

    switch(a_AutoState6){
         case kRedAutoIdle6:
            StopSwerves();
            break;
        case kRedExtend6:
            //a_Arm->ArmPistonUp();
            break;
        case kRedDrop6:
            //a_Arm->ClawOpen();
            break;
        case kRedRetract6:
            //a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway6:
            if(DriveDirection(4.8768, 0, .3, false)) { // need real numbers
                nextState = kRedRetract6;
            }
    }
    a_AutoState6 = nextState;
}

void Autonomous::RCSL() 
{
    a_AutoState7 = kRedExtend7;
}

void Autonomous::PeriodicRCSL() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kRedAutoIdle7:
            StopSwerves();
            break;
        case kRedExtend7:
            //a_Arm->ArmPistonUp();
            break;
        case kRedDrop7:
            //a_Arm->ClawOpen();
            break;
        case kRedRetract7:
            //a_Arm -> ArmPistonDown();
            
            break;
        case kRedDriveAway7:
            if (DriveDirection(3.6576, 0, 0.3, false)) {
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
}

void Autonomous::PeriodicRDGM(){
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kRedAutoIdle8:
            StopSwerves();
            break;
        case kRedExtend8:
            //a_Arm->ArmPistonUp();
            break;
        case kRedDrop8:
            //a_Arm->ClawOpen();
            break;
        case kRedRetract8:
            //a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway8:
         if (DriveDirection(4.8768, 0, 0.4, false)) {
                nextState = kRedAutoIdle8;
                //need the actual numbers
            }
            break;
    }
    a_AutoState8 = nextState;
}


void Autonomous::RCSM() {
    a_AutoState9 = kRedExtend9;
}

void Autonomous::PeriodicRCSM() {
    
    AutoState9 nextState = a_AutoState9;
    
    switch(a_AutoState9){
        case kRedAutoIdle9:
            StopSwerves();
            break;
        case kRedExtend9:
            //a_Arm->ArmPistonUp();
            break;
        case kRedDrop9:
            //a_Arm->ClawOpen();
            break;
            
        case kRedRetract9:
            //a_Arm->ArmPistonDown();
            break;

        case kRedDriveAway9:
            if(DriveDirection(3.6576, 0, .4, false)) { // need real numbers please & ty
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
}

void Autonomous::PeriodicRDGR() {

    AutoState10 nextState = a_AutoState10;

    switch (a_AutoState10) {
        case kRedAutoIdle10:
            StopSwerves();
            break;
        case kRedExtend10:
            //a_Arm->ArmPistonUp();
            break;

        case kRedDrop10:
            //a_Arm->ClawOpen();
            break;

        case kRedRetract10:
            //a_Arm->ArmPistonDown();
            
            break;

        case kRedDriveAway10:
            // need the real drive numbers
            if (DriveDirection(4.8768, 0, 0.25, false)) {
                nextState = kRedAutoIdle10;
            }
            break;
    }
    a_AutoState10 = nextState;
 }

void Autonomous::RCSR() {
    a_AutoState11 = kRedExtend11;
}

void Autonomous::PeriodicRCSR() {
    AutoState11 nextState = a_AutoState11;

    switch(a_AutoState11){
        case kRedAutoIdle11:
            StopSwerves();
            break;
        case kRedExtend11:
            //a_Arm->ArmPistonUp();
            break;
        case kRedDrop11:
            //a_Arm->ClawOpen();
            break;
        case kRedRetract11:
            //a_Arm->ArmPistonDown();
            break;
        case kRedDriveAway11:
            if(DriveDirection(3.6576, 0, .25, false)) { // need real numbers
                nextState = kRedRetract11;
            }
            break;
        case kRedGoToStation11:
            if(DriveDirection(2.667, -90, .25,false)) { // need real numbers
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

void Autonomous::DoNothing() {
    a_AutoState12 = kIdle;
}

void Autonomous::PeriodicDoNothing() {
    AutoState12 nextState = a_AutoState12;

    switch(a_AutoState12){
        case kIdle:
            StopSwerves();
            break;
    }
a_AutoState12 = nextState;

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