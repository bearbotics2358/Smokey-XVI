#include "AbsoluteEncoder.h"

#include "misc.h"
#include <math.h>

AbsoluteEncoder::AbsoluteEncoder(int port, double minVolts, double maxVolts):
m_encoder(port),
m_minVolts(minVolts),
m_voltRange(maxVolts - minVolts),
m_offset(0) {}

AbsoluteEncoder::AbsoluteEncoder(int port, double minVolts, double maxVolts, double offset):
m_encoder(port),
m_minVolts(minVolts),
m_voltRange(maxVolts - minVolts),
m_offset(offset) {}

double AbsoluteEncoder::getAngle() const {
    return 2 * M_PI * getRotations();
}

double AbsoluteEncoder::getRotations() const {
    // the encoder outputs volts in a range of 0 to 5, scaling linearly with how rotated it is
    double volts = m_encoder.GetVoltage();

    // convert volts to rotations
    double out = ((volts - m_minVolts) / m_voltRange) - m_offset;
    return misc::clampRotations(out);
}

double AbsoluteEncoder::getVolts() const {
    return m_encoder.GetVoltage();
}