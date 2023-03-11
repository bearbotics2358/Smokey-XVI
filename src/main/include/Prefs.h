
#pragma once // only add this code once; saves space by removing redundancy

#include <units/angle.h>
#include <units/length.h>
#include <frc/SerialPort.h>

// Declare constants such as CAN IDs here

// uncomment to enable the new swerve
//#define NEW_SWERVE

#define GYRO_ID 35 // TODO: UPDATE TO CORRECT NUMBER
#define PISTON_PUSH_SOLENOID_MODULE 10
#define PISTON_PULL_SOLENOID_MODULE 11
#define CLAW_OPEN_SOLENOID_MODULE 12
#define CLAW_CLOSE_SOLENOID_MODULE 13
#define CONE_PRESSURE_SOLENOID 14
#define CUBE_PRESSURE_SOLENOID 15

#define EXTEND_PISTON_TIME 2
#define CLAW_PISTON_TIME 2

#define SHUTTLE_MOTOR 22
#define ARM_MOTOR 12
#define LIMIT_SWITCH 0

#define MAX_CLIMB_PERCENT 0.2
#define MAX_FREE_SPEED 16.3

#define TICKS_STEERING 18.0 // roughly 18 "position" units per steering rotation

/*====== SWERVE MOTOR CONTROLLER IDS ======*/

/*

Module Numbering Scheme:

m = number engraved on module

    - Drive ID: 2m - 1
    - Steering ID: 2m
    - Encoder ID: 16 + m

*/

#define FL_ID 7
#define FR_ID 2
#define BL_ID 1
#define BR_ID 6


/*======= ENCODER CONSTANTS =======*/

// the distance we were getting from the wheel was not quite right, so we multiply them by this constant to get the right distance
#define DISTANCE_ADJUSTMANT_FACTOR 1.09789

// falcon encoder ticks per 1 revolution
#define FALCON_UNITS_PER_REV 2048

// swerve drive absolute encoder analog ports
#define FL_SWERVE_ABS_ENC_PORT 0
#define FR_SWERVE_ABS_ENC_PORT 3
#define BL_SWERVE_ABS_ENC_PORT 1
#define BR_SWERVE_ABS_ENC_PORT 2

// min and max voltage of absolute encoders on swerve drives
#define FL_SWERVE_ABS_ENC_MIN_VOLTS 0.014076
#define FL_SWERVE_ABS_ENC_MAX_VOLTS 4.952392

#define FR_SWERVE_ABS_ENC_MIN_VOLTS 0.037842
#define FR_SWERVE_ABS_ENC_MAX_VOLTS 4.962158

#define BL_SWERVE_ABS_ENC_MIN_VOLTS 0.004883
#define BL_SWERVE_ABS_ENC_MAX_VOLTS 4.641113

#define BR_SWERVE_ABS_ENC_MIN_VOLTS 0.010986
#define BR_SWERVE_ABS_ENC_MAX_VOLTS 4.963378

#define CANCODER_OFFSET_1 129.5 - 180
#define CANCODER_OFFSET_2 115.3 - 180
#define CANCODER_OFFSET_3 0
#define CANCODER_OFFSET_4 0
#define CANCODER_OFFSET_5 0
#define CANCODER_OFFSET_6 127.4 - 180
#define CANCODER_OFFSET_7 240.7 - 180
#define CANCODER_OFFSET_8 0
#define CANCODER_OFFSET_ARM 130.34

#define CANCODER_ID_1 17
#define CANCODER_ID_2 18
#define CANCODER_ID_3 19
#define CANCODER_ID_4 20
#define CANCODER_ID_5 21
#define CANCODER_ID_6 22
#define CANCODER_ID_7 23
#define CANCODER_ID_8 24
#define CANCODER_ID_ARM 25

static double CANCODER_OFFSETS[] = {
    CANCODER_OFFSET_1, 
    CANCODER_OFFSET_2, 
    CANCODER_OFFSET_3, 
    CANCODER_OFFSET_4, 
    CANCODER_OFFSET_5, 
    CANCODER_OFFSET_6, 
    CANCODER_OFFSET_7, 
    CANCODER_OFFSET_8,
    CANCODER_OFFSET_ARM
    };


/* ========== Joystick Ports ========= */
#define JOYSTICK_PORT 1
#define JOYSTICK_DEADZONE 0.15

#define XBOX_CONTROLLER 5


/* ============ GEAR RATIOS ======== */
// I have a feeling this might be wrong, since our distance measurents are sligtly off
// ratio is drive motor rotations / wheel rotations
#define SWERVE_DRIVE_MOTOR_GEAR_RATIO (6.75 / 1.0)

#define INVERTED_MOTOR (-1.0) //used to allow for the inversion of motors in MK4i modules

// wheel diameter in meters
#define WHEEL_DIAMETER 0.1016


/* ============= MqttClient ============= */

#define SEND_BUF_LEN 2048
#define RECV_BUF_LEN 2048


/* ============= CanHandler ============= */

#define LEFT_ARDUINO_CAN_ID 1
#define RIGHT_ARDUINO_CAN_ID 1

#define LEFT_ARDUINO_API_ID 2
#define RIGHT_ARDUINO_API_ID 3

#define FL_SWERVE_DATA_ID 0
#define BL_SWERVE_DATA_ID 1
#define FR_SWERVE_DATA_ID 2
#define BR_SWERVE_DATA_ID 3


/* ============= Vision ============= */

// the height of the camare used to track the target for shooting
constexpr units::length::meter_t TARGET_CAMERA_HEIGHT = units::length::meter_t(1.0);

// pitch of vision camera that tracks the target, positive is up
constexpr units::angle::radian_t TARGET_CAMERA_PITCH = units::angle::radian_t(1.0);

// the height of the target we are shooting at
constexpr units::length::meter_t TARGET_HEIGHT = units::length::meter_t(1.0);

// pitch of target
constexpr units::angle::radian_t TARGET_PITCH = units::angle::radian_t(0.0);

// mdns name of camera
#define SHOOTER_CAMERA_NAME "photonvision"
#define BALL_CAMERA_NAME "temp" 

/* ============= Arduino ============= */

#define BAUD_RATE_TOF 115200
#define USB_PORT_TOF frc::SerialPort::kMXP
#define DATA_BITS_TOF 8
#define PARITY_TOF frc::SerialPort::kParity_None
#define STOP_BITS_TOF frc::SerialPort::kStopBits_One

#define BAUD_RATE_ARDUINO 9600
#define USB_PORT_ARDUINO frc::SerialPort::kUSB
#define DATA_BITS_ARDUINO 8
#define PARITY_ARDUINO frc::SerialPort::kParity_None
#define STOP_BITS_ARDUINO frc::SerialPort::kStopBits_One

/*
sraight up: 78.57
"0": 130.34
back: 319.13

no cone/cube front clearance: 120
no cone/cube back clearance: 330
*/