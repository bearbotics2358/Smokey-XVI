#include "vision/PiController.h"
#include "logging.h"
#include <sstream>

static const std::string data_topic { "pi/cv/data" };
static const std::string control_topic { "pi/cv/control" };
static const std::string error_topic { "pi/cv/error" };

std::optional<TargetType> target_type_from_int(int n) {
    if (n > (int) TargetType::All) {
        return {};
    } else {
        return (TargetType) n;
    }
}

std::optional<Mode> string_to_mode(std::string_view str) {
    if (str == "vision") {
        return Mode::Vision;
    } else if (str == "remote_viewing") {
        return Mode::RemoteViewing;
    } else if (str == "none") {
        return Mode::None;
    } else {
        return {};
    }
}

void PiData::invalid_msg(std::string_view msg) {
    lg::error("invalid error message recieved from the pi");
    errors.push_back(Error::internal(std::string("invalid message sent by raspberry pi: ") + std::string(msg)));
}

PiController::PiController(MqttClient&& client, std::unique_ptr<PiData>&& data):
m_client(std::move(client)),
m_data(std::move(data)) {}

PiController::~PiController() {
    m_client.unsubscribe(data_topic);
    m_client.unsubscribe(error_topic);
}

std::optional<PiController> PiController::create(const std::string& address, int port) {
    auto client = MqttClient::create(address, port);
    if (!client.has_value()) {
        return {};
    }

    std::unique_ptr<PiData> data(new PiData);

    auto res1 = client->subscribe(data_topic, data_callback, data.get());
    auto res2 = client->subscribe(error_topic, error_callback, data.get());

    if (res1.is_err() || res2.is_err()) {
        return {};
    }

    return std::make_optional<PiController>(PiController(std::move(*client), std::move(data)));
}

void PiController::update() {
    auto result = m_client.update();
    if (result.is_err()) {
        m_data->errors.push_back(result);
    }
}

const std::vector<Ball>& PiController::balls() const {
    return m_data->balls;
}

Mode PiController::mode() const {
    return m_data->mode;
}

bool PiController::has_error() const {
    return !m_data->errors.empty();
}

const std::vector<Error>& PiController::errors() const {
    return m_data->errors;
}

void PiController::clear_errors() {
    // this should call the destructor of error
    m_data->errors.clear();
}

void PiController::data_callback(std::string_view msg, PiData *data) {
    data->balls.clear();
    std::istringstream iss = std::istringstream(std::string(msg));
    int type;
    double distance;
    double angle;
    double score;

    while (iss >> type >> distance >> angle >> score) {
        auto target_type = target_type_from_int(type);
        if (!target_type.has_value()) {
            data->errors.push_back(Error::internal("invalid target type recieved from pi"));
            return;
        }
        data->balls.push_back(Ball {
            .type = *target_type,
            .distance = distance,
            .angle = angle,
            .score = score,
        });
    }
}

void PiController::error_callback(std::string_view msg, PiData *data) {
    // serialized error string
    std::string_view error_str;

    auto semicolon_pos = msg.find(';');
    if (semicolon_pos == std::string_view::npos) {
        // there is no semicolon, which is an error
        // this should not happen
        data->invalid_msg(msg);
    } else if (semicolon_pos == 0) {
        // semicolon is at the front, no mode change occured
        error_str = msg.substr(1);
    } else {
        // mode string
        auto mode_str = msg.substr(0, semicolon_pos);
        auto mode = string_to_mode(mode_str);
        if (mode.has_value()) {
            data->mode = *mode;
        } else {
            data->invalid_msg(msg);
        }

        error_str = msg.substr(semicolon_pos + 1);
    }

    // I don't think there is a way to avoid this allocation as long as we are using istringstream to pars
    auto error = Error::deserialize(std::string(error_str));
    if (error.has_value()) {
        data->errors.push_back(std::move(*error));
    } else {
        data->invalid_msg(msg);
    }
}