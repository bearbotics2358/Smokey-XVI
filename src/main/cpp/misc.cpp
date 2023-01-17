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