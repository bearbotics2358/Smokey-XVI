#include "CompressorController.h"
#include <frc/smartdashboard/SmartDashboard.h>

CompressorController::CompressorController()
#ifdef COMP_BOT  // Not available on the practice bot
:
a_Compressor(frc::PneumaticsModuleType::REVPH)
#endif
{
    
}

void CompressorController::update(){
#ifdef COMP_BOT  // Not available on the practice bot
    if (a_Compressor.GetPressureSwitchValue() == false) {
        a_Compressor.Disable();
    } else {
        a_Compressor.EnableDigital();
    }
    frc::SmartDashboard::PutNumber("compressor pressure: ", getTankPressure());
#endif
}

double CompressorController::getTankPressure(){
#ifdef COMP_BOT  // Not available on the practice bot
    return a_Compressor.GetPressure().value();
#else
    return 0.0;
#endif
}

/*  CompressorController Ideas
    - give operator the ability to turn compressor on and off, an override
*/

void CompressorController::turnOff() {
#ifdef COMP_BOT  // Not available on the practice bot
    a_Compressor.Disable();
#endif
}
