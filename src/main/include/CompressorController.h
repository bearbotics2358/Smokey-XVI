#pragma once

#include <frc/Compressor.h>

class CompressorController {

    public:
        CompressorController();
        void update();
        double getTankPressure();
        void turnOff();

    private:
#ifdef COMP_BOT  // Not available on the practice bot
        frc::Compressor a_Compressor;
#endif

};
