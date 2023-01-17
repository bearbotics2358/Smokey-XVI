#pragma once

#include <optional>
#include <string>
#include <string_view>

/** The type of error that has occurred. */
enum class ErrorType : int {
    /** No error has occured. */
    Ok = 0,
    /** An internal error occured. */
    Internal = 1,
    /** A library returned an error and the cause is unknown. */
    Library = 2,
    /** An unknown erorr occured. */
    Unknown = 3,
    /** The requested operation was not valid. */
    InvalidOperation = 4,
    /** Invalid arguments passed to function. */
    InvalidArgs = 5,
    /** Failed to open a certain resource. */
    ResourceUnavailable = 6,
    /** Error with the network connection. */
    Network = 7,
    /** Out of memory. */
    OutOfMem = 8,
};

/** Returns the string message for the given error. */
const char *error_type_to_string(ErrorType type);

/** Converts an integer into an error type.
 *  @param n the integer to convert to an error.
 *  @return the error wrapped in an optional. If the input number was invalid, returns none.
 */
std::optional<ErrorType> error_type_from_int(int n);

#define ERROR_CONSTRUCTOR_DEFS(ctor_name, error_type)                                                \
    static inline Error ctor_name() { return Error(error_type); }                                    \
    static inline Error ctor_name(const std::string& message) { return Error(error_type, message); } \
    static inline Error ctor_name(std::string&& message) { return Error(error_type, std::move(message)); }

// TODO: add domain codes
class [[nodiscard]] Error {
    public:
        Error(ErrorType type);
        Error(ErrorType type, const std::string& message);
        Error(ErrorType type, std::string&& message);
        ~Error();

        static inline Error ok() { return Error(ErrorType::Ok); }

        // error constructors to more easily make an error of a certain type
        ERROR_CONSTRUCTOR_DEFS(internal, ErrorType::Internal)
        ERROR_CONSTRUCTOR_DEFS(library, ErrorType::Library)
        ERROR_CONSTRUCTOR_DEFS(unknown, ErrorType::Unknown)
        ERROR_CONSTRUCTOR_DEFS(invalid_operation, ErrorType::InvalidOperation)
        ERROR_CONSTRUCTOR_DEFS(invalid_args, ErrorType::InvalidArgs)
        ERROR_CONSTRUCTOR_DEFS(resource_unavailable, ErrorType::ResourceUnavailable)
        ERROR_CONSTRUCTOR_DEFS(network, ErrorType::Network)
        ERROR_CONSTRUCTOR_DEFS(memory, ErrorType::OutOfMem)

        /** Serializes the error into a string.
         *  @return the serialized string.
         */
        std::string serialize() const;

        /** Deserializes an error from the given string.
         *  @param string the input string to be deserialized.
         */
        static std::optional<Error> deserialize(const std::string& string);

        // make a string to be displayed to the user
        std::string to_string() const;

        // get error type
        ErrorType type() const;
        // get error message
        const std::string& message() const;

        bool is_ok() const;
        bool is_err() const;
        bool is(ErrorType type) const;

        void ignore() const;
        // terminates program if this error is not ok
        void assert_ok() const;

    private:
        ErrorType m_type;
        std::string m_message;
};
