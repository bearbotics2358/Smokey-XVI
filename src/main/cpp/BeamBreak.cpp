#include "BeamBreak.h"

BeamBreak::BeamBreak(int port):
a_Input(port) {}

bool BeamBreak::beamBroken() {
    if (a_Input.Get() == 1) {
        isBroken = true;
    } else {
        isBroken = false;
    }
    return isBroken;
}

/*
    1 = the beam is broken
    0 = the beam is not broken
*/