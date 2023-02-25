#pragma once
#include <ctre/Phoenix.h>
#include "Gyro.h"
#include "Prefs.h"


class Balancer {
    public:
        Balancer();
        float getPercentTilted();
    
    private:
        Gyro a_Gyro;
};