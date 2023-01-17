#pragma once

#include "misc.h"
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

/** Class for the shooter */
class BallShooter {
    public:
        /** Constructor for the shooter.
         *  @param leftId can id of left shooter motor.
         *  @param rightId can id of right shooter motor.
         */
        BallShooter(int leftId, int rightId);

        /** Set shooter speed in rpm.
         *  @param rpm desired shooter speed in rpm.
         */
        void setSpeed(double rpm);

        /** Stops the shooter.
         *  Prefer this to setSpeed(0) because that causes the shooter to try and maintain 0 rpm,
         *  which causes the gears to grind.
         *  This one sets the percent output to 0.
         */
        void stop();

        /** Gets the current shooter speed in rpm.
         *  @return shooter speed in rpm.
         */
        double getSpeed();

    private:
        TalonFX a_shooterLeft;
        TalonFX a_shooterRight;
};