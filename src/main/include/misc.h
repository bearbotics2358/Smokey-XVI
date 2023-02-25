#pragma once

namespace misc {
/** Converts rpm to the unit the talon motor uses for velocity control (ticks per 100 ms).
 *  @param rpm input rpm.
 *  @return the talon velocity units.
 */
double rpmToTalonVel(double rpm);

/** Converts the tolon motor velocity units (ticks per 100 ms) to rpm.
 *  @param units input talon velocity units.
 *  @return converted rpm.
 */
double talonVelToRpm(double units);

double degToRad(double degrees);
double radToDeg(double radians);

// clamps degrees to be in the range of 0-360
// if they are out of this range they are converted to an equivalent degrees in this range
double clampDegrees(double degrees);

double clampRotations(double rotations);

// returns the physical difference in degrees between angle a and angle b
// always returns positive value
double degreesDiff(double a, double b);

// gets seconds elapsed since the epoch of the system clock
double getSeconds();


//Provide IDs for Swerve Modules, making for easier module switching at comp
int GetFLDrive();
int GetFLSteer();
int GetFLCANCoder();
int GetFLCANCoderOffset();
int GetFRDrive();
int GetFRSteer();
int GetFRCANCoder();
int GetFRCANCoderOffset();
int GetBLDrive();
int GetBLSteer();
int GetBLCANCoder();
int GetBLCANCoderOffset();
int GetBRDrive();
int GetBRSteer();
int GetBRCANCoder();
int GetBRCANCoderOffset();
}