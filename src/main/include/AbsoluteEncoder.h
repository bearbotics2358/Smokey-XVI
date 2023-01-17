#pragma once

#include <frc/AnalogInput.h>

/** An encoder which remembers its position after power cycles.
 *  The absolute encoder must be connected to an analog port on the rio.
 *  The encoder will have a certain minimum voltage level and maximum voltage level,
 *  and the voltage outputted by the encoder will vary linearly over this range as the encoder is turned from 0 to 360 degrees.
 *  This minimum and maximum voltage level must be calibrated.
 */
class AbsoluteEncoder {
    public:
        /** Constructor for absolute encoder, sets offset to 0 by default.
         *  @param port the port number on the roborio that the encoder is plugged into.
         *  @param minVolts the minimum amount of volts the encoder will output.
         *  @param maxVolts the maximum amount of volts the encoder will output.
         */
        AbsoluteEncoder(int port, double minVolts, double maxVolts);

        /** Constructor for absolute encoder.
         *  Has a configurable offset, which is the offset angle that the encoder is mounted at on the robot.
         *  This is useful if the encoder is rotated away from the desired 0 point.
         *  @param port the port number on the roborio that the encoder is plugged into.
         *  @param minVolts the minimum amount of volts the encoder will output.
         *  @param maxVolts the maximum amount of volts the encoder will output.
         *  @param offset the offset angle from 0 in rotations.
         */
        AbsoluteEncoder(int port, double minVolts, double maxVolts, double offset);

        /** Get the angle from the encoder in degrees.
         *  @return angle of encoder in degrees.
         */
        double getAngle() const;

        /** Get the angle from the encoder in rotations.
         *  @return angle of encoder in rotations.
         */
        double getRotations() const;

        // TEMP
        // TODO: remove
        double getVolts() const;

    private:
        frc::AnalogInput m_encoder;

        // starting voltage value where encoder is at 0 degrees
        double m_minVolts;
        // how many volts until the encoder will reach 360 degrees, the range of the voltage, not max
        double m_voltRange;

        // offset angle in rotations (0 - 1)
        double m_offset;
};