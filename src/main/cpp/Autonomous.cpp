#include "Autonomous.h"
#include "buttons.h"
#include "misc.h"
#include "Prefs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <sys/time.h>

//left if positive degrees right is negative
Autonomous::Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, Claw *Claw, TOF *tof):
a_Gyro(Gyro),
a_SwerveDrive(SwerveDrive),
a_Claw(Claw),
a_TOF(tof){}



//-------------------------------------Auto Stuff---------------------------------------------//

void Autonomous::StartAuto(const std::string autoMode) {
    if(autoMode == BlueDropAndGoLeft){
        BDGL();
    }
    else if (autoMode == BlueChargeStationLeft){
        BCSL();
    }
    else if (autoMode == BlueDropAndGoMiddle){
        BDGM();
    }
    else if (autoMode == BlueChargeStationMiddle){
        BCSM();
    }
    else if (autoMode == BlueDropAndGoRight){
        BDGR();
    }
    else if (autoMode == BlueChargeStationRight){
        BCSR();
    }
    else if (autoMode == RedDropAndGoLeft){
        RDGL();
    }
    else if (autoMode ==  RedChargeStationLeft){
        RCSL();
    }
    else if (autoMode == RedDropAndGoMiddle){
        RDGM();
    }
    else if (autoMode == RedChargeStationMiddle){
        RCSM();
    }
    else if (autoMode == RedDropAndGoRight ){
        RDGR();
    }
    else if (autoMode == RedChargeStationRight){
        RCSR();
    }
    else if (autoMode == RobotDoNothing){
        DoNothing();
    }
    else if(autoMode == LeftTwoPiece){
        LeftPiece2();
    }
    else if(autoMode == RightTwoPiece){
        RightPiece2();
    }

    a_AutoSelected = autoMode; 
    
}
void Autonomous::PeriodicAuto(const std::string periodicAutoMode) {
    if(periodicAutoMode == BlueDropAndGoLeft){
        PeriodicBDGL();
    }
    else if (periodicAutoMode == BlueChargeStationLeft){
        PeriodicBCSL();
    }
    else if (periodicAutoMode == BlueDropAndGoMiddle){
        PeriodicBDGM();
    }
    else if (periodicAutoMode == BlueChargeStationMiddle){
        PeriodicBCSM();
    }
    else if (periodicAutoMode == BlueDropAndGoRight){
        PeriodicBDGR();
    }
    else if (periodicAutoMode == BlueChargeStationRight){
        PeriodicBCSR();
    }
    else if (periodicAutoMode == RedDropAndGoLeft){
        PeriodicRDGL();
    }
    else if (periodicAutoMode == RedChargeStationLeft){
        PeriodicRCSL();
    }
    else if (periodicAutoMode == RedDropAndGoMiddle){
        PeriodicRDGM();
    }
    else if (periodicAutoMode == RedChargeStationMiddle){
        PeriodicRCSM();
    }
    else if (periodicAutoMode == RedDropAndGoRight){
        PeriodicRDGR();
    }
    else if (periodicAutoMode == RedChargeStationRight){
        PeriodicRCSR();
    }
    else if (periodicAutoMode == RobotDoNothing){
        PeriodicDoNothing();
    }
    else if(periodicAutoMode == LeftTwoPiece){
        LeftPeriodicPiece2();
    }
    else if(periodicAutoMode == RightTwoPiece){
        RightPeriodicPiece2();
    }

    a_PeriodicAutoSelected = periodicAutoMode; 
    
}




// s----------------------------------AUTONOMOUS ROUTINES---------------------------------------- //

void Autonomous::BDGL() {
    a_AutoState0 = kBlueExtend0;

    // reset state time
    state_time = gettime_d();
}

void Autonomous::PeriodicBDGL() {
    
    AutoState0 nextState = a_AutoState0;

    switch (a_AutoState0) {
        case kBlueAutoIdle0:
            StopSwerves();
            break;
        case kBlueExtend0:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
                state_time = gettime_d();
            nextState = kBlueDrop0;
            }
            break;
        case kBlueDrop0:
            a_Claw->ClawOpen();
        if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract0;
        }
            break;
        case kBlueRetract0:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
                state_time = gettime_d();
            nextState = kBlueDriveAway0;
             }
            break;
        case kBlueDriveAway0:
            if (DriveDirection(4.8768, 0, .25, true)) {
                nextState = kBlueAutoIdle0;
            }
            break;
    }
    a_AutoState0 = nextState;
}


void Autonomous::BCSL() {
    a_AutoState1 = kBlueExtend1;
    state_time = gettime_d();
}

void Autonomous::PeriodicBCSL() {

    AutoState1 nextState = a_AutoState1;

    switch (a_AutoState1) {
         case kBlueAutoIdle1:
            StopSwerves();
            break;
        case kBlueExtend1:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
                state_time = gettime_d();
             nextState = kBlueDrop1;
            }
            break;
        case kBlueDrop1:
             a_Claw->ClawOpen();
               if(gettime_d() > state_time + CLAW_PISTON_TIME){
                state_time = gettime_d();
             nextState = kBlueRetract1;
        }
            
            break;

        case kBlueRetract1:
            a_Claw->TransformClaw(125, -15, false);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
                state_time = gettime_d();
             nextState = kBlueDriveAway1;
            }
            break;

        case kBlueDriveAway1:
            if (DriveDirection(3.6576, 0, 0.25, true)) {
                nextState = kBlueGoToStation1;
            }
            break;

        case kBlueGoToStation1:
            if (DriveDirection(2.667, -90, 0.25, true)) {
                nextState = kBlueBalance1;
            }
            //need to actually use drivedirection
            break;

        case kBlueBalance1:
            Balance(-90);
            nextState = kBlueAutoIdle1;
            break;

       }
    a_AutoState1 = nextState;
}

void Autonomous::BDGM() {
    a_AutoState2 = kBlueExtend2;   
    state_time = gettime_d();
}

void Autonomous::PeriodicBDGM() {

    AutoState2 nextState = a_AutoState2;

    switch (a_AutoState2) {
       case kBlueAutoIdle2:
            StopSwerves();
            break;
        case kBlueExtend2:
            a_Claw->TransformClaw(170, 650, true);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
             state_time = gettime_d();
             nextState = kBlueDrop2;
             }
            break;
        case kBlueDrop2:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
             nextState = kBlueRetract2;
        }
            
            break;
        case kBlueRetract2:
             a_Claw->TransformClaw(125, -15, false);     
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
                state_time = gettime_d();
             nextState = kBlueDriveAway2;
            }
            break;

        case kBlueDriveAway2:
            // we might be stuck on the wall, so move to the next state after some time
            if (DriveDirection(4.8768, 0, 0.4, true)) {
                nextState = kBlueAutoIdle2;
            }
            break;     
    }
    a_AutoState2 = nextState;
}


void Autonomous::BCSM() 
{
    a_AutoState3 = kBlueExtend3;
    state_time = gettime_d();
}

void Autonomous::PeriodicBCSM() {
    AutoState3 nextState = a_AutoState3;
   
    switch (a_AutoState3) {
         case kBlueAutoIdle3:
            StopSwerves();
            break;
        case kBlueExtend3:
            a_Claw->TransformClaw(170, 650, true);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDrop3;
             }
            break;
        case kBlueDrop3:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract3;
        }
            break;
        case kBlueRetract3:
            a_Claw->TransformClaw(125, -15, false); 
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway3;
             }
            break;
        case kBlueDriveAway3:
            if (DriveDirection(3.6576, 0, 0.4, true)) {
                nextState = kBlueGoToStation3;
                //need the actual numbers
            }
            break;
        case kBlueGoToStation3:
            if(DriveDirection(.2, 180, .25, true)) {
                nextState = kBlueBalance3;
                //need the actual numbers
            }
            break; 
        case kBlueBalance3:
            Balance(180);
            nextState = kBlueAutoIdle3;
            break;
    }
    a_AutoState3 = nextState;
}

void Autonomous::BDGR(){
    a_AutoState4 = kBlueExtend4;
    state_time = gettime_d();
}

void Autonomous::PeriodicBDGR() {

    AutoState4 nextState = a_AutoState4;

    switch (a_AutoState4) {
         case kBlueAutoIdle4:
            StopSwerves();
            break;
        case kBlueExtend4:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();    
            nextState = kBlueDrop4;
            }
            break;
        case kBlueDrop4:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract4;
        }
            break;
        case kBlueRetract4:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway4;
             }
            break;
        case kBlueDriveAway4:
            // need the real drive numbers
            if (DriveDirection(4.8768, 0, 0.3, true)) {
                nextState = kBlueAutoIdle4;
            }
            break;
    }
    a_AutoState4 = nextState;
 }
 void Autonomous::BCSR() 
{
    state_time = gettime_d();
    a_AutoState5 = kBlueExtend5;
}
    
void Autonomous::PeriodicBCSR() {

    AutoState5 nextState = a_AutoState5;

    switch (a_AutoState5) {
        case kBlueAutoIdle5:
            StopSwerves();
            break;
        case kBlueExtend5:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDrop5;
            }
            break;
        case kBlueDrop5:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract5;
        }
            break;
        case kBlueRetract5:
            a_Claw->TransformClaw(125, -15, false);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway5;
            }
            break;

        case kBlueDriveAway5:
            if (DriveDirection(3.6576, 0, 0.3, true)) {
                nextState = kBlueGoToStation5;
            }
            break;

        case kBlueGoToStation5:
             if (DriveDirection(2.667, 90, 0.25, true)) {
                nextState = kBlueBalance5;
            }
            break;

        case kBlueBalance5:
            Balance(90);
            nextState = kBlueAutoIdle5;
            break;

    }
    a_AutoState5 = nextState;
}

void Autonomous::RDGL(){
    state_time = gettime_d();
    a_AutoState6 = kRedExtend6;
}

void Autonomous::PeriodicRDGL() {
    AutoState6 nextState = a_AutoState6;

    switch(a_AutoState6){
         case kRedAutoIdle6:
            StopSwerves();
            break;
        case kRedExtend6:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop6;
            }
            break;
        case kRedDrop6:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();    
            nextState = kRedRetract6;
        }
            break;
        case kRedRetract6:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway6;
             }
            break;
        case kRedDriveAway6:
            if(DriveDirection(4.8768, 0, .3, true)) { // need real numbers
                nextState = kRedAutoIdle6;
            }
    }
    a_AutoState6 = nextState;
}

void Autonomous::RCSL() 
{
    state_time = gettime_d();
    a_AutoState7 = kRedExtend7;
}

void Autonomous::PeriodicRCSL() {
    AutoState7 nextState = a_AutoState7;
   
    switch (a_AutoState7) {
        case kRedAutoIdle7:
            StopSwerves();
            break;
        case kRedExtend7:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop7;
            }
            break;
        case kRedDrop7:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract7;
        }
            break;
        case kRedRetract7:
            a_Claw->TransformClaw(125, -15, false);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway7;
            }
            break;
        case kRedDriveAway7:
            if (DriveDirection(3.6576, 0, 0.3, true)) {
                nextState = kRedGoToStation7;
            }
            break;
        case kRedGoToStation7:
            if (DriveDirection(2.667, -90, 0.25, true)) {
                nextState = kRedBalance7;
                //need the actual numbers
            }
            break; 
        case kRedBalance7:
            Balance(-90);
            nextState = kRedAutoIdle7;
            break;
    }
    a_AutoState7 = nextState;
}

void Autonomous::RDGM(){
    state_time = gettime_d();
    a_AutoState8 =  kRedExtend8;
}

void Autonomous::PeriodicRDGM(){
    AutoState8 nextState = a_AutoState8;
   
    switch (a_AutoState8) {
        case kRedAutoIdle8:
            StopSwerves();
            break;
        case kRedExtend8:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop8;
            }
            break;
        case kRedDrop8:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract8;
        }
            break;
        case kRedRetract8:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway8;
             }
            break;
        case kRedDriveAway8:
         if (DriveDirection(4.8768, 0, 0.4, true)) {
                nextState = kRedAutoIdle8;
                //need the actual numbers
            }
            break;
    }
    a_AutoState8 = nextState;
}


void Autonomous::RCSM() {
    state_time = gettime_d();
    a_AutoState9 = kRedExtend9;
}

void Autonomous::PeriodicRCSM() {
    
    AutoState9 nextState = a_AutoState9;
    
    switch(a_AutoState9){
        case kRedAutoIdle9:
            StopSwerves();
            break;
        case kRedExtend9:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop9;
            }
            break;
        case kRedDrop9:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract9;
        }
            break;
        case kRedRetract9:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway9;
             }
            break;

        case kRedDriveAway9:
            if(DriveDirection(3.6576, 0, .4, true)) { // need real numbers please & ty
                nextState = kRedGoToStation9;
            }
            break;
            
        case kRedGoToStation9:
            if(DriveDirection(.2, 180, .25, true)) { // need real numbers please & ty
                nextState = kRedBalance9;
            }
            break;
        
        case kRedBalance9:
           Balance(180);
           nextState = kRedAutoIdle9;
            break;
    }   
    a_AutoState9 = nextState;
}

void Autonomous::RDGR(){
    state_time = gettime_d();
    a_AutoState10 = kRedExtend10;
}

void Autonomous::PeriodicRDGR() {

    AutoState10 nextState = a_AutoState10;

    switch (a_AutoState10) {
        case kRedAutoIdle10:
            StopSwerves();
            break;
        case kRedExtend10:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop10;
            }
            break;
        case kRedDrop10:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();    
            nextState = kRedRetract10;
        }
            break;

        case kRedRetract10:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway10;
             }
            break;
        case kRedDriveAway10:
            // need the real drive numbers
            if (DriveDirection(4.8768, 0, 0.25, true)) {
                nextState = kRedAutoIdle10;
            }
            break;
    }
    a_AutoState10 = nextState;
 }

void Autonomous::RCSR() {
    state_time = gettime_d();
    a_AutoState11 = kRedExtend11;
}

void Autonomous::PeriodicRCSR() {
    AutoState11 nextState = a_AutoState11;

    switch(a_AutoState11){
        case kRedAutoIdle11:
            StopSwerves();
            break;
        case kRedExtend11:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDrop11;
            }
            break;
        case kRedDrop11:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedRetract11;
        }
            break;
        case kRedRetract11:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kRedDriveAway11;
             }
            break;
        case kRedDriveAway11:
            if(DriveDirection(3.6576, 0, .25, true)) { // need real numbers
                nextState = kRedGoToStation11;
            }
            break;
        case kRedGoToStation11:
            if(DriveDirection(2.667, 90, .25,true)) { // need real numbers
                nextState = kRedBalance11;
            }//18ft wide by 11 ft 3/8 
            break;
        case kRedBalance11:
        Balance(90);
            nextState = kRedAutoIdle11;
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
void Autonomous::LeftPiece2() {
    a_AutoState13 = kBlueExtend13;
}
void Autonomous::LeftPeriodicPiece2(){
    AutoState13 nextState = a_AutoState13;

    switch(a_AutoState13){
        case kBlueAutoIdle13:
            StopSwerves();
            break;
        case kBlueExtend13:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            distance = 
            state_time = gettime_d();
            nextState = kBlueDrop13;
            }
            break;
        case kBlueDrop13:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract13;
            }
            break;
        case kBlueRetract13:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway13;
             }
            break;
        case kBlueDriveAway13:
            if(DriveDirection(5.69, 0, .25, true)) { // need real numbers
                nextState = kTurn13;
            }
            break;
        case kTurn13:
            if(TurnToAngle(-90)){
                nextState = kBluePickUp13;
            }
            break;
        case kBluePickUp13:
            if(DriveDirection(3, 90, .25, true)||a_TOF->GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE){
                distance = a_SwerveDrive->getAvgDistance();
                a_Claw->ClawClose();
                nextState = kGoBack13;
            }
            break;
        case kGoBack13:
        //TODO subtract the distance we need to get to the cube platform
            if(DriveDirection(distance, -90, .25, true)){
                nextState = kTurnBack13;
            }
            break;
        case kTurnBack13:
            if(TurnToAngle(90)){
                nextState = kGoToGrid13;
            }
            break;
        case kGoToGrid13:
            if(DriveDirection(5.69, 180, .25, true)){
                nextState = kBlueAutoIdle13;
                }
            
        case kExtendAgain13:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kPlace13;
            }
            break;
        case kPlace13:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueAutoIdle13;
            }
            break;
    }
   a_AutoState13 = nextState;
}
void Autonomous::RightPiece2() {
    a_AutoState13 = kBlueExtend13;
}
void Autonomous::RightPeriodicPiece2(){
AutoState14 nextState = a_AutoState14;

    switch(a_AutoState14){
        case kBlueAutoIdle14:
            StopSwerves();
            break;
        case kBlueExtend14:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            distance = 
            state_time = gettime_d();
            nextState = kBlueDrop14;
            }
            break;
        case kBlueDrop14:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueRetract14;
            }
            break;
        case kBlueRetract14:
             a_Claw->TransformClaw(125, -15, false);
             if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueDriveAway14;
             }
            break;
        case kBlueDriveAway14:
            if(DriveDirection(5.69, 0, .25, true)) { // need real numbers
                nextState = kTurn14;
            }
            break;
        case kTurn14:
            if(TurnToAngle(90)){
                nextState = kBluePickUp14;
            }
            break;
        case kBluePickUp14:
            if(DriveDirection(3, -90, .25, true)||a_TOF->GetTargetRangeIndicator() == target_range_enum::TARGET_IN_RANGE){
                distance = a_SwerveDrive->getAvgDistance();
                a_Claw->ClawClose();
                nextState = kGoBack14;
            }
            break;
        case kGoBack14:
        //TODO subtract the distance we need to get to the cube platform
            if(DriveDirection(distance, 90, .25, true)){
                nextState = kTurnBack14;
            }
            break;
        case kTurnBack14:
            if(TurnToAngle(-90)){
                nextState = kGoToGrid14;
            }
            break;
        case kGoToGrid14:
            if(DriveDirection(5.69, 180, .25, true)){
                nextState = kBlueAutoIdle14;
                }
            
        case kExtendAgain14:
            a_Claw->TransformClaw(170, 650, true);
            if(gettime_d() > state_time + EXTEND_PISTON_TIME){
            state_time = gettime_d();
            nextState = kPlace14;
            }
            break;
        case kPlace14:
            a_Claw->ClawOpen();
            if(gettime_d() > state_time + CLAW_PISTON_TIME){
            state_time = gettime_d();
            nextState = kBlueAutoIdle14;
            }
            break;
    }
   a_AutoState14 = nextState;
}


void Autonomous::StopSwerves() {
    a_SwerveDrive->stop();
}

void Autonomous::StartTimer() {
    waitTimeStart = misc::getSeconds();
}

double Autonomous::gettime_d(){
	// return time in seconds as a double
	double t0;
	struct timeval tv0;

	gettimeofday(&tv0, NULL);
	t0 = 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
	// printf("seconds: %ld\n", tv0.tv_sec);
	// printf("usecs:   %ld\n", tv0.tv_usec);
	// printf("time:    %lf\n", t0);

	return t0;
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
            // for the second part of the move, drive slower
            // after the second part of the move, allow goToTheDon() to slam on the brakes
            a_SwerveDrive->goToTheDon(speed / 2, angle, dist, fieldOriented, true);

        } else {
            // first part of the move, at the user specified speed
            // after the first part of the move, do not slam on the brakes
            a_SwerveDrive->goToTheDon(speed, angle, dist, fieldOriented, false);
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
    float currentTime = gettime_d();
    double tiltAngle = a_Gyro->getPitch() - PITCH_OFFSET;
    double percentTilt = tiltAngle / 15;
    double speed = percentTilt * MAX_FREE_SPEED;
    startedClimb = true;
    if(startedClimb) {
        a_SwerveDrive->driveDirectionVelocity(speed, direction);
        if(abs(tiltAngle) < 5) {
            startTime = gettime_d();
        }
        return false;
    }
    if ((currentTime - startTime > 1.0) && (abs(tiltAngle) < 5) && startedClimb){
        StopSwerves();
        return true;
    }
    else{
        a_SwerveDrive->driveDirection(MAX_CLIMB_PERCENT, direction);
        if(abs(tiltAngle) > 5){
            startedClimb = true;
            a_SwerveDrive->driveDirection(0.2, direction);
        }
        return false;
    }
}