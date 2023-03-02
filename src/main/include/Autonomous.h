

#pragma once

// #include <frc/WPILib.h>
#include "SwerveDrive.h"
#include "Gyro.h"
#include <Prefs.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <units/math.h>
#include "Arm.h"


enum AutoType {
    BlueDropAndGoLeft = 0,
    BCSL = 1,
    BDGM = 2,
    BCSM = 3,
    BDGR = 4,
    BCSR = 5,
};
enum AutoState0 { // Encoders
    kBlueAutoIdle0 = 0,
    kBlueExtend0,
    kBlueDrop0,
    kBlueRetract0,
    kBlueDriveAway0

};

enum AutoState1 { // Encoders
    kBlueAutoIdle1,
    kBlueExtend1,
    kBlueDrop1,
    kBlueRetract1,
    kBlueDriveAway1,
    kBlueGoToStation1,
    kBlueBalance1,
    kBlueWait1
};

enum AutoState2 { // T.O.F and Encoders
    kBlueAutoIdle2,
    kBlueExtend2,
    kBlueDrop2,
    kBlueRetract2,
    kBlueDriveAway2
};

// states for 3 ball auto
enum AutoState3 {
    kBlueAutoIdle3,
    kBlueExtend3,
    kBlueDrop3,
    kBlueRetract3,
    kBlueDriveAway3,
    kBlueGoToStation3,
    kBlueBalance3
};

enum AutoState4 {
    kBlueAutoIdle4,
    kBlueExtend4,
    kBlueDrop4,
    kBlueRetract4,
    kBlueDriveAway4
};

// states for 5 ball auto
enum AutoState5 {
    kBlueAutoIdle5,
    kBlueExtend5,
    kBlueDrop5,
    kBlueRetract5,
    kBlueDriveAway5,
    kBlueGoToStation5,
    kBlueBalance5
};

enum AutoState6 {
    kRedAutoIdle6,
    kRedExtend6,
    kRedDrop6,
    kRedRetract6,
    kRedDriveAway6
};
 enum AutoState7{
    kRedAutoIdle7,
    kRedExtend7,
    kRedDrop7,
    kRedRetract7,
    kRedDriveAway7,
    kRedGoToStation7,
    kRedBalance7


 };

  enum AutoState8{
    kRedAutoIdle8,
    kRedExtend8,
    kRedDrop8,
    kRedRetract8,
    kRedDriveAway8,

  };

  enum AutoState9{
    kRedAutoIdle9,
    kRedExtend9,
    kRedDrop9,
    kRedRetract9,
    kRedDriveAway9,
    kRedGoToStation9,
    kRedBalance9
  };
  enum AutoState10{
    kRedAutoIdle10,
    kRedExtend10,
    kRedDrop10,
    kRedRetract10,
    kRedDriveAway10


  };
    enum AutoState11{
        kRedAutoIdle11,
        kRedExtend11,
        kRedDrop11,
        kRedRetract11,
        kRedDriveAway11,
        kRedGoToStation11,
        kRedBalance11
    };


class Autonomous {
    public:
        Autonomous(Gyro *Gyro, SwerveDrive *SwerveDrive, Arm *Arm);

        void DecidePath();
        const char *GetCurrentPath();

        void StartAuto();
        void PeriodicAuto();

        void BDGL(); //Blue Drop and Go Left AutoState0
        void PeriodicBDGL(); //Periodic Blue Drop and Go Left AutoState0

        void BCSL(); //Blue Charge Station Left AutoState1
        void PeriodicBCSL(); //Periodic Blue Charge Station Left AutoState1

        void BDGM(); //Blue Drop and Go Middle AutoState2
        void PeriodicBDGM();//Periodic Blue Drop and Go Middle AutoState2
        
        void BCSM();//Blue Charge Station Middle AutoState3
        void PeriodicBCSM();//Periodic Blue Charge Station Middle AutoState3

        void BDGR();//Blue Drop and Go Right AutoState4
        void PeriodicBDGR();//Periodic Blue Drop and Go Right AutoState4

        void BCSR();//Blue Charge Station Right AutoState5
        void PeriodicBCSR();//Periodic Blue Charge Station Right AutoState5
        
        void RDGL();//Red Drop and Go Left AutoState6
        void PeriodicRDGL();//Periodic Red Drop and Go Left AutoState6

        void RCSL(); // Red Charge Station Left AutoState7
        void PeriodicRCSL();// Periodic Red Charge Station Left AutoState7

        void RDGM(); // Red Drop and Go Middle AutoState8
        void PeriodicRDGM(); // Periodic Red Drop and Go Middle AutoState8

        void RCSM(); //Red Charge Station Middle AutoState9
        void PeriodicRCSM(); //Periodic Red Charge Station Middle AutoState9

        void RDGR();//Red Drop and Go Right AutoState10
        void PeriodicRDGR(); //Periodic Red Drop and Go Right AutoState10

        void RCSR(); // Red Charge Station Right AutoState11
        void PeriodicRCSR(); // Periodic Red Charge Station Right AutoState11

        // ------------------Sub-Routines-------------------------//

        void StopSwerves(); // IDLE

        // Timer System
        // Note: you MUST have a separate case to start the timer, though WaitForTime handles stopping & resetting
        void StartTimer();
        bool WaitForTime(double time); // Wait for specified time in seconds

        // Drives in direction at speed for distance. If going straight backwards, set angle to 180, not dist as a negative
        bool DriveDirection(double dist, double angle, double speed, bool fieldOriented);

        bool TurnToAngle(float angle); // turns to a specific angle
        bool Balance(float direction);


    private:
        Gyro *a_Gyro;
        Arm *a_Arm;
        SwerveDrive *a_SwerveDrive;
        frc::XboxController *a_Xbox;
        

        AutoState0 a_AutoState0;
        AutoState1 a_AutoState1;
        AutoState2 a_AutoState2;
        AutoState3 a_AutoState3;
        AutoState4 a_AutoState4;
        AutoState5 a_AutoState5;
        AutoState6 a_AutoState6;
        AutoState7 a_AutoState7;
        AutoState8 a_AutoState8;
        AutoState9 a_AutoState9;
        AutoState10 a_AutoState10;
        AutoState11 a_AutoState11;


        AutoType autoPathMaster;
        float drivestart { 0.0 };

        // used for waitForTime method
        double waitTimeStart { 0.0 };

        // TEMP
        double autoStartTime { 0.0 };
        // TEMP
        double autoScale { 1.0 };

        bool startedClimb { false };
        float startTime { 0.0 };

        // start position of robot during 5 ball auto relative to near left corner of field
        // FIXME: this is a very innacurate guess, more so than the other measurements
        constexpr static Vec2 AUTO35_START_POS { 5.52, 7.69 };
};