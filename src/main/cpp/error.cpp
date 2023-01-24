#include "error.h"
#include "logging.h"
#include <iterator>
#include <sstream>
#include <string>

const char *error_type_to_string(ErrorType type) {
    switch (type) {
        case ErrorType::Ok:
            return "ok";
        case ErrorType::Internal:
            return "internal error";
        case ErrorType::Library:
            return "error returned by library";
        case ErrorType::Unknown:
            return "unknown error";
        case ErrorType::InvalidOperation:
            return "invalid operation";
        case ErrorType::InvalidArgs:
            return "invalid arguments";
        case ErrorType::ResourceUnavailable:
            return "resource unavailable";
        case ErrorType::Network:
            return "network error";
        case ErrorType::OutOfMem:
            return "out of memory";
    }
    // this should be unreacheble, it just stops the compiler emmiting a warning
    return "";
}

std::optional<ErrorType> error_type_from_int(int n) {
    // have to update this everytime a new error type is added
    if (n > 8) {
        return {};
    } else {
        return (ErrorType) n;
    }
}

Error::Error(ErrorType type):
m_type(type),
m_message() {}

Error::Error(ErrorType type, const std::string& message):
m_type(type),
m_message(message) {}

Error::Error(ErrorType type, std::string&& message):
m_type(type),
m_message(std::move(message)) {}

Error::~Error() {}

std::string Error::serialize() const {
    auto err_str = std::to_string((int) m_type);
    if (m_message.length() == 0) {
        return err_str;
    } else {
        return err_str + " " + m_message;
    }
}

std::optional<Error> Error::deserialize(const std::string& string) {
    std::istringstream iss(string);
    int err_num = 0;

    iss >> err_num;
    if (!iss.good()) {
        return {};
    }

    auto err_type = error_type_from_int(err_num);
    if (!err_type.has_value()) {
        return {};
    }

    char space_char;
    iss.get(space_char);
    if (iss.eof()) {
        // no error message is present
        return Error(*err_type, std::string());
    } else if (iss.good()) {
        // ensure next character is space
        if (space_char != ' ') {
            return {};
        }
    } else if (iss.bad()) {
        return {};
    }

    std::string message(std::istreambuf_iterator<char>(iss), {});

    return Error(*err_type, std::move(message));
}

std::string Error::to_string() const {
    return std::string(error_type_to_string(m_type)) + ": " + m_message;
}

ErrorType Error::type() const {
    return m_type;
}

const std::string& Error::message() const {
    return m_message;
}

bool Error::is_ok() const {
    return m_type == ErrorType::Ok;
}

bool Error::is_err() const {
    return m_type != ErrorType::Ok;
}

bool Error::is(ErrorType type) const {
    return m_type == type;
}

void Error::ignore() const {
    if (is_err()) {
        lg::warn("ignoring error");
    }
}

void Error::assert_ok() const {
    if (is_err()) {
        lg::critical("assertion failed: error is not ok: %s", to_string().c_str());
    }
}
