#include "CanHandler.h"

#include <stdio.h>
#include <string>
#include <utility>

#include "Prefs.h"

CanHandler::CanHandler(const std::vector<Arduino>& in):
m_endpoints() {
    for (const auto& arduino : in) {
        std::vector<Field> fields;

        int bitsum = 0;

        for (const auto& field : arduino.fields) {
            u8 desired_bits = field.desired_bits;

            bitsum += desired_bits;

            if (bitsum > 64 || desired_bits > 32) {
                break;
            }

            u32 bitnum = 0;
            for (u8 k = 0; k < desired_bits; k++) {
                bitnum |= 1 << k;
            }

            Field temp_d {
                field.id,
                desired_bits,
                bitnum,
                field.multiplier,
                0
            };

            fields.push_back(temp_d);
        }

        auto endpoint = Endpoint {
            frc::CAN(arduino.can_id),
            arduino.api_id,
            fields
        };

        m_endpoints.push_back(std::move(endpoint));
    }
}

CanHandler CanHandler::layout2022() {
    // clang-format off
    std::vector<Arduino> layout {
        Arduino {
            .can_id = LEFT_ARDUINO_CAN_ID,
            .api_id = LEFT_ARDUINO_API_ID,
            .fields = std::vector<DataField> {
                DataField {
                    .id = FL_SWERVE_DATA_ID,
                    .desired_bits = 16,
                    .multiplier = 10.0f
                },
                DataField {
                    .id = BL_SWERVE_DATA_ID,
                    .desired_bits = 16,
                    .multiplier = 10.0f
                }
            }
        },
        Arduino {
            .can_id = RIGHT_ARDUINO_CAN_ID,
            .api_id = RIGHT_ARDUINO_API_ID,
            .fields = std::vector<DataField> {
                DataField {
                    .id = FR_SWERVE_DATA_ID,
                    .desired_bits = 16,
                    .multiplier = 10.0f
                },
                DataField {
                    .id = BR_SWERVE_DATA_ID,
                    .desired_bits = 16,
                    .multiplier = 10.0f
                }
            }
        }
    };
    // clang-format on

    return CanHandler(layout);
}

std::optional<float> CanHandler::getData(int which) const {
    for (const auto& endpoint : m_endpoints) {
        for (const auto& field : endpoint.data) {
            if (field.id == which) {
                return field.data / field.multiplier;
            }
        }
    }
    return {};
}

void CanHandler::update() {
    frc::CANData data;
    for (auto& endpoint : m_endpoints) {
        if (endpoint.can.ReadPacketNew(endpoint.api_id, &data)) {
            for (usize j = 0; j / 2 < endpoint.data.size(); j += 2) {
                Field& datas = endpoint.data[j];
                datas.data = (data.data[j] << 8) | data.data[j + 1];
            }
        }
    }
}