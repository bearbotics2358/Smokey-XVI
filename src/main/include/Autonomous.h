

#pragma once

// #include <frc/WPILib.h>
#include "BallShooter.h"
#include "Collector.h"
#include "SwerveDrive.h"
#include <JrimmyGyro.h>
#include <Prefs.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <units/math.h>


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
    kAutoIdle0 = 0,
    kDriveAway0
};

enum AutoState1 { // Encoders
    kAutoIdle1 = 0,
    kStartShooter1,
    kWaitShooter1,
    kShoot1,
    kStartTimer1,
    kWait1,
    kDoneShooting1,
    kTaxi1
};

enum AutoState2 { // T.O.F and Encoders
    kAutoIdle2 = 0,
    kDriveBackThroughBall2,
    kTurn2,
    kDriveToWall2,
    kShoot2,
    kWait2
    /*
    kSecondShoot2,
    kCheckSecondShot2,
    kWait2_2*/
};

// states for 3 ball auto
enum class A3 {
    Idle,
    SpoolShooter,
    WaitShooter,
    Shoot1,
    Pickup2,
    Pickup3,
    GoToShoot23,
    Shoot23,
};

// states for 5 ball auto
enum class A5 {
    Idle,
    SpoolShooter,
    WaitShooter,
    Shoot1,
    Pickup2,
    Pickup3,
    GoToShoot23,
    Shoot23,
    Pickup4,
    WaitPickup5,
    GoToShoot45,
    Shoot45,
};

// states for 5 ball auto with vision
enum class A5V {
    Idle,
    SpoolShooter,
    WaitShooter,
    Shoot1,
    Pickup2,
    Pickup3,
    GoToShoot23,
    Shoot23,
    Pickup4,
    WaitPickup5,
    GoToShoot45,
    Shoot45,
};


class Autonomous {
    public:
        Autonomous(JrimmyGyro *Gyro, frc::Joystick *XboxController, SwerveDrive *SwerveDrive, BallShooter *BallShooter, Collector *Collector);

        void DecidePath();
        const char *GetCurrentPath();

        void StartAuto();
        void PeriodicAuto();

        void Start0Ball();
        void Periodic0Ball();

        void StartLeft1Ball();
        void StartMiddle1Ball();
        void StartRight1Ball();
        void Periodic1Ball();

        void Start2Ball();
        void Periodic2Ball();

        void Start35Ball();
        void Periodic3Ball();
        void Periodic5Ball();
        void Periodic5BallVision();

        // ------------------Sub-Routines-------------------------//

        void IDontLikeExercise(); // IDLE

        // Timer System
        // Note: you MUST have a separate case to start the timer, though WaitForTime handles stopping & resetting
        void StartTimer();
        bool WaitForTime(double time); // Wait for specified time in seconds

        void SpoolShooter(float speed); // Spools up shooter ahead of time to improve efficiency

        // deploy collector and spool motoer
        void CollectorDown();
        // raise collector and stop motor
        void CollectorUp();

        bool IndexAndShoot(float speed); // Shooting a ball when the shooter is spinning fast enough

        // Drives in direction at speed for distance. If going straight backwards, set angle to 180, not dist as a negative
        bool DriveDirection(double dist, double angle, double speed, bool fieldOriented);

        bool TurnToAngle(float angle); // turns to a specific angle


    private:
        JrimmyGyro *a_Gyro;
        SwerveDrive *a_SwerveDrive;
        frc::Joystick *a_Xbox;
        BallShooter *a_BallShooter;
        Collector *a_Collector;

        AutoState0 a_AutoState0;
        AutoState1 a_AutoState1;
        AutoState2 a_AutoState2;
        A3 a_AutoState3 { A3::Idle };
        A5 a_AutoState5 { A5::Idle };
        A5V a_AutoState5Vision { A5V::Idle };

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