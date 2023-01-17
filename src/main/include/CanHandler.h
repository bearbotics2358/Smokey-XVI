#pragma once

#include <optional>
#include <vector>

#include <frc/CAN.h>

#include "types.h"

// TODO: comment better
class DataField {
    public:
        int id;
        u8 desired_bits;
        float multiplier;
};


class Arduino {
    public:
        u8 can_id;
        u8 api_id;
        std::vector<DataField> fields;
};

// only used internally
class Field {
    public:
        int id;
        u8 bits;
        u32 bitnum;
        float multiplier;
        i32 data;
};

class Endpoint {
    public:
        frc::CAN can;
        u8 api_id;
        std::vector<Field> data;
};

class CanHandler {
    public:
        CanHandler(const std::vector<Arduino>& in);
        static CanHandler layout2022();

        std::optional<float> getData(int field_id) const;
        void update();

    private:
        std::vector<Endpoint> m_endpoints;
};