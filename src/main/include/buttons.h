#pragma once

/** Buttons on the Xbox controller. */
class OperatorButton {
    public:
        constexpr static int A = 1;
        constexpr static int B = 2;
        constexpr static int X = 3;
        constexpr static int Y = 4;
        constexpr static int LeftBumper = 5;
        constexpr static int RightBumper = 6;
        constexpr static int Back = 7;
        constexpr static int Start = 8;
        constexpr static int LeftJoystick = 9;
        constexpr static int RightJoystick = 10;
};

/** Analog input joysticks on the Xbox controller. */
class OperatorJoystick {
    public:
        constexpr static int LeftXAxis = 0;
        constexpr static int LeftYAxis = 1;
        constexpr static int LeftTrigger = 2;
        constexpr static int RightTrigger = 3;
        constexpr static int RightXAxis = 4;
        constexpr static int RightYAxis = 5;
};

/** Buttons on the driver joystick. */
class DriverButton {
    public:
        constexpr static int Trigger = 1;
        constexpr static int ThumbButton = 2;
        constexpr static int Button3 = 3;
        constexpr static int Button4 = 4;
        constexpr static int Button5 = 5;
        constexpr static int Button6 = 6;
        constexpr static int Button7 = 7;
        constexpr static int Button8 = 8;
        constexpr static int Button9 = 9;
        constexpr static int Button10 = 10;
        constexpr static int Button11 = 11;
        constexpr static int Button12 = 12;
};

/** Analog input joysticks on the driver joystick. */
class DriverJoystick {
    public:
        constexpr static int XAxis = 0;
        constexpr static int YAxis = 1;
        constexpr static int ZAxis = 2;
        constexpr static int Slider = 3;
};