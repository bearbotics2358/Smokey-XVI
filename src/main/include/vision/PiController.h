#pragma once

#include "error.h"
#include "mqtt/MqttClient.h"
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

enum class TargetType : int {
    None = 0x0,
    Red = 0x1,
    Blue = 0x2,
    All = 0x3,
};

std::optional<TargetType> target_type_from_int(int n);

enum class Mode {
    Vision,
    RemoteViewing,
    None,
};

// returns none on invalid string
std::optional<Mode> string_to_mode(std::string_view str);

struct Ball {
        TargetType type;
        double distance;
        double angle;
        double score;
};

class PiData {
    public:
        std::vector<Ball> balls {};
        // TODO: update this mode to what the pi will start in
        Mode mode { Mode::None };
        std::vector<Error> errors {};

    private:
        friend class PiController;

        // helper method which loggs that an invalid message was recieved and pushes an error to the errors vector
        void invalid_msg(std::string_view msg);
};

class PiController {
    public:
        PiController(PiController&& in) = default;
        PiController& operator=(PiController&& in) = default;

        ~PiController();

        // create new pi controller, returns none on failure
        static std::optional<PiController> create(const std::string& address, int port);

        void update();

        // get targets recognised by vision
        const std::vector<Ball>& balls() const;
        // get current mode of pi
        Mode mode() const;

        // has the pi sent over an error since clear_error last called
        bool has_error() const;
        // get all errors that have occured since last clear_errors called
        const std::vector<Error>& errors() const;
        // clear any errors that have occured
        void clear_errors();

    private:
        PiController(MqttClient&& client, std::unique_ptr<PiData>&& data);

        static void data_callback(std::string_view msg, PiData *data);
        static void error_callback(std::string_view msg, PiData *data);

        MqttClient m_client;
        std::unique_ptr<PiData> m_data;

        // static does not work for these
};