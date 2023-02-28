#pragma once

#include <frc/Compressor.h>

class CompressorController {

    public:
        CompressorController();
        void update();
        double getTankPressure();
        void turnOff();

    private:
        frc::Compressor a_Compressor;

};
