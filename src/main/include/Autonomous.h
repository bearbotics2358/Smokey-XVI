

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
    k0Ball = 0,
    kLeft1Ball = 1,
    kMiddle1Ball = 2,
    kRight1Ball = 3,
    k2Ball = 4,
    k3Ball = 5,
    k5Ball = 6,
    k5BallVision = 7,
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

class Autonomous {
    public:
        Autonomous(Gyro *Gyro, frc::XboxController *XboxController, SwerveDrive *SwerveDrive, Arm *Arm);

        void DecidePath();
        const char *GetCurrentPath();

        void StartAuto();
        void PeriodicAuto();

        void BDGL(); //Blue Drop and Go
        void PeriodicBDGL(); //Periodic Blue Drop and Go

        void BCSL(); //Blue Charge Station Left
        void PeriodicBCSL(); //Periodic Blue Charge Station Left

        void BDGM(); //Blue Drop and Go Middle
        void PeriodicBDGM();//Periodic Blue Drop and Go Middle
        
        void BCSM();//Blue Charge Station Middle
        void PeriodicBCSM();//Periodic Blue Charge Station Middle

        void BDGR();//Blue Drop and Go Right
        void PeriodicBDGR();//Periodic Blue Drop and Go Right

        void BCSR();//Blue Charge Station Right
        void PeriodicBCSR();//Periodic Blue Charge Station Right
        void Periodic5BallVision();

        Arm *a_Arm;

        // ------------------Sub-Routines-------------------------//

        void StopSwerves(); // IDLE

        // Timer System
        // Note: you MUST have a separate case to start the timer, though WaitForTime handles stopping & resetting
        void StartTimer();
        bool WaitForTime(double time); // Wait for specified time in seconds

        void SpoolShooter(float speed); // Spools up shooter ahead of time to improve efficiency

        bool IndexAndShoot(float speed); // Shooting a ball when the shooter is spinning fast enough

        // Drives in direction at speed for distance. If going straight backwards, set angle to 180, not dist as a negative
        bool DriveDirection(double dist, double angle, double speed, bool fieldOriented);

        bool TurnToAngle(float angle); // turns to a specific angle


    private:
        Gyro *a_Gyro;
        SwerveDrive *a_SwerveDrive;
        frc::XboxController *a_Xbox;
        

        AutoState0 a_AutoState0;
        AutoState1 a_AutoState1;
        AutoState2 a_AutoState2;
        AutoState3 a_AutoState3;
        AutoState4 a_AutoState4;
        AutoState5 a_AutoState5;

        AutoType autoPathMaster;
        float drivestart { 0.0 };

        // used for waitForTime method
        double waitTimeStart { 0.0 };

        // TEMP
        double autoStartTime { 0.0 };
        // TEMP
        double autoScale { 1.0 };

        // start position of robot during 5 ball auto relative to near left corner of field
        // FIXME: this is a very innacurate guess, more so than the other measurements
        constexpr static Vec2 AUTO35_START_POS { 5.52, 7.69 };
};