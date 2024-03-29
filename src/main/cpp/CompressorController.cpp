#include "CompressorController.h"
#include <frc/smartdashboard/SmartDashboard.h>

CompressorController::CompressorController():
a_Compressor(frc::PneumaticsModuleType::REVPH)
{
    
}

void CompressorController::update(){
    if (a_Compressor.GetPressureSwitchValue() == false) {
        a_Compressor.Disable();
    } else {
        a_Compressor.EnableDigital();
    }
    frc::SmartDashboard::PutNumber("compressor pressure: ", getTankPressure());
}

double CompressorController::getTankPressure(){
    return a_Compressor.GetPressure().value();
}

/*  CompressorController Ideas
    - give operator the ability to turn compressor on and off, an override
*/

void CompressorController::turnOff() {
    a_Compressor.Disable();
}
