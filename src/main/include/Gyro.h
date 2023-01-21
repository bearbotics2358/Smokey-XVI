#pragma once
#include <ctre/phoenix/sensors/BasePigeon.h>
#include <hal/HAL.h>

class Gyro {
    
    public:
        explicit BasePigeon(int port, std::string const & canbus = "");
        virtual ~BasePigeon();
        void Cal();
        double GetAngle();
        double GetAngleClamped();
        std::string GetSmartDashboardType();
        void Zero(double offset = 0);
        virtual void Update();
};