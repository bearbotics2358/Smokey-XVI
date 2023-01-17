
#pragma once // only add this code once; saves space by removing redundancy

#include <units/angle.h>
#include <units/length.h>

// Declare constants such as CAN IDs here

// uncomment to enable the new swerve
//#define NEW_SWERVE

/*======== CLIMBER CONSTANTS ========*/

#define CLIMBER_MOTOR_ID 24
#define CLIMBER_PUSH_SOLENOID_MODULE 10
#define CLIMBER_PULL_SOLENOID_MODULE 11
#define CLIMBER_MOTOR_PERCENT_OUTPUT 1.0
#define CLIMBER_MOTOR_SPEED 1000 // the desired speed for the arm to rise (mm/s)
// 12:1 gearbox between motor and thunderhex (winch)
#define CLIMBER_MM_PER_TICK (0.026 / 12.0) // how many mm the climber arm rises every motor tick
#define CLIMBER_TICKS_PER_MM (3.8347 * 12.0) // how many ticks for every mm the climber arm rises
// port of the climber limit switch
#define CLIMBER_SWITCH_PORT 0

/*======== SHOOTER CONSTANTS ========*/

#define LEFT_SHOOTER_ID 26
#define RIGHT_SHOOTER_ID 27
// desired shooter speed to shoot from the wall in rpm
#define LOW_SHOOTER_SPEED 1000
#define SHOOTER_SPEED 2000
// how close we have to be to shooter speed to start indexing in balls
#define SHOOTER_TOLERANCE 0.98

/*======== COLLECTOR CONSTANTS ========*/

#define COLLECTOR_MOTOR_ID 21
#define INDEXER_MOTOR_ID 22
#define COLLECTOR_MOTOR_PERCENT_OUTPUT 1.0
#define INDEXER_MOTOR_PERCENT_OUTPUT 0.75
#define COLLECTOR_PUSH_SOLENOID_MODULE 8
#define COLLECTOR_PULL_SOLENOID_MODULE 9


/*====== SWERVE MOTOR CONTROLLER IDS ======*/
#define FL_DRIVE_ID 1
#define FL_STEER_ID 2

#define FR_DRIVE_ID 3
#define FR_STEER_ID 8

#define BL_DRIVE_ID 5
#define BL_STEER_ID 6

#define BR_DRIVE_ID 7
#define BR_STEER_ID 4


/*======= ENCODER CONSTANTS =======*/

// the distance we were getting from the wheel was not quite right, so we multiply them by this constant to get the right distance
#define DISTANCE_ADJUSTMANT_FACTOR 1.09789

// TODO: figure out what this constant means
#define TICKS_STEERING 18.0 // roughly 18 "position" units per steering rotation

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

// offset from 0 of the absolute encders
//#define FL_SWERVE_ABS_ENC_OFFSET 16.24
//#define FR_SWERVE_ABS_ENC_OFFSET 85.76
//#define BL_SWERVE_ABS_ENC_OFFSET 0.22
//#define BR_SWERVE_ABS_ENC_OFFSET 179.77
#define FL_SWERVE_ABS_ENC_OFFSET 342.33
#define FR_SWERVE_ABS_ENC_OFFSET 276.11
#define BL_SWERVE_ABS_ENC_OFFSET 0.0
#define BR_SWERVE_ABS_ENC_OFFSET 0.0


/* ========== Joystick Ports ========= */
#define JOYSTICK_PORT 1
#define JOYSTICK_DEADZONE 0.15

#define XBOX_CONTROLLER 5


/* ============ GEAR RATIOS ======== */
// I have a feeling this might be wrong, since our distance measurents are sligtly off
// ratio is drive motor rotations / wheel rotations
#define SWERVE_DRIVE_MOTOR_GEAR_RATIO (7.04 / 1.0)

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