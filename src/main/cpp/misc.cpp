#include "misc.h"
#include "Prefs.h"
#include <math.h>
#include <stdio.h>
#include <sys/time.h>

double misc::rpmToTalonVel(double rpm) {
    return (rpm * FALCON_UNITS_PER_REV) / 600.0;
}

double misc::talonVelToRpm(double units) {
    return (units * 600.0) / FALCON_UNITS_PER_REV;
}

double misc::degToRad(double degrees) {
    return degrees * 2 * M_PI / 360.0;
}

double misc::radToDeg(double radians) {
    return radians * 360 / (2 * M_PI);
}

double misc::clampDegrees(double degrees) {
    double out = fmod(degrees, 360);
    if (out < 360) {
        return 1 + out;
    } else {
        return out;
    }
}

double misc::clampRotations(double rotations) {
    double out = fmod(rotations, 1);
    if (out < 0) {
        return 1 + out;
    } else {
        return out;
    }
}

double misc::degreesDiff(double a, double b) {
    a = clampDegrees(a);
    b = clampDegrees(b);

    double diff = fabs(b - a);
    if (diff > 180) {
        return 360 - diff;
    } else {
        return diff;
    }
}

double misc::getSeconds() {
    // return time in seconds as a double
    timeval tv0;
    gettimeofday(&tv0, nullptr);
    return 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
}

//DEFINES IDS FOR SWERVE, ALLOWING FOR EASY CODE SWITCHING OF IDS
/*

Module Numbering Scheme:

m = number engraved on module

    - Drive ID: 2m - 1
    - Steering ID: 2m
    - Encoder ID: 16 + m

*/

int misc::GetFLDrive(){
    int trueID = 2 * FL_ID - 1;
    return trueID;
}

int misc::GetFLSteer(){
    int trueID = 2 * FL_ID;
    return trueID;
}

int misc::GetFLCANCoder(){
    int trueID = 16 + FL_ID;
    return trueID;
}
int misc::GetFLCANCoderOffset(){
    if((FL_ID >= 1) && (FL_ID <= 8)){
        return CANCODER_OFFSETS[FL_ID - 1];
    }
}

int misc::GetFRDrive(){
    int trueID = 2 * FR_ID - 1;
    return trueID;
}

int misc::GetFRSteer(){
    int trueID = 2 * FR_ID;
    return trueID;
}

int misc::GetFRCANCoder(){
    int trueID = 16 + FR_ID;
    return trueID;
}

int misc::GetFRCANCoderOffset(){
    if((FR_ID >= 1) && (FR_ID <= 8)){
        return CANCODER_OFFSETS[FR_ID - 1];
    }
}

int misc::GetBLDrive(){
    int trueID = 2 * BL_ID - 1;
    return trueID;
}

int misc::GetBLSteer(){
    int trueID = 2 * BL_ID;
    return trueID;
}

int misc::GetBLCANCoder(){
    int trueID = 16 + BL_ID;
    return trueID;
}
int misc::GetBLCANCoderOffset(){
    if((BL_ID >= 1) && (BL_ID <= 8)){
        return CANCODER_OFFSETS[BL_ID - 1];
    }
}

int misc::GetBRDrive(){
    int trueID = 2 * BR_ID - 1;
    return trueID;
}

int misc::GetBRSteer(){
    int trueID = 2 * BR_ID;
    return trueID;
}

int misc::GetBRCANCoder(){
    int trueID = 16 + BR_ID;
    return trueID;
}

int misc::GetBRCANCoderOffset(){
    if((BR_ID >= 1) && (BR_ID <= 8)){
        return CANCODER_OFFSETS[BR_ID - 1];
    }
}