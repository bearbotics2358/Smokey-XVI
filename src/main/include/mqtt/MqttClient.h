#pragma once

#include "Prefs.h"
#include "error.h"
#include "types.h"
#include <memory>
#include <mqtt/mqtt.h>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

struct CallbackEntry {
        // callback data must be void pointer because we store many different types of callbacks in the callbacks hashmap
        void (*callback)(std::string_view, void *);
        void *data;
};

// data in here is visible to message callback and reconnect callback (this is most data in MqttClient)
class CallbackData {
    public:
        CallbackData(const std::string& host, int port);

        std::string host;
        int port;
        // file descriptor of open socket for mqtt
        int sockfd { -1 };

        std::string client_name;

        std::unordered_map<std::string, CallbackEntry> callbacks {};

        u8 sendbuf[SEND_BUF_LEN];
        u8 recvbuf[RECV_BUF_LEN];
};

class MqttClient {
    public:
        MqttClient(MqttClient&& client) = default;
        MqttClient& operator=(MqttClient&& client) = default;

        // creates new mqtt client, returns none on failure
        static std::optional<MqttClient> create(const std::string& host, int port);
        ~MqttClient();

        Error update();

        Error publish(const std::string& topic, std::string_view payload);

        // calback takes in a string_view of the message and a pointer to the passed in object
        template<typename T>
        Error subscribe(const std::string& topic, void (*callback)(std::string_view, T *), T *data) {
            auto callback_entry = CallbackEntry {
                .callback = (void (*)(std::string_view, void *)) callback,
                .data = data
            };

            // insert doesn't remove old elements, so delete the previous callback is it exists
            m_callback_data->callbacks.erase(topic);
            // do this before subscribing to avoid race condition
            m_callback_data->callbacks.insert(std::pair(topic, callback_entry));

            // TODO: figure out if subscribing twice is a problem
            auto result = mqtt_subscribe(m_client.get(), topic.c_str(), 0);
            if (result != MQTT_OK) {
                m_callback_data->callbacks.erase(topic);
                return get_error(result);
            }

            return Error::ok();
        }

        void unsubscribe(const std::string& topic);

    private:
        MqttClient(std::unique_ptr<mqtt_client>&& client, std::unique_ptr<CallbackData>&& callback_data);

        // converts the mqtt error into an Error
        static Error get_error(MQTTErrors error);

        static void reconnect_callback(mqtt_client *client, void **state);
        static void message_callback(void **state, mqtt_response_publish *msg);

        // open non blocking socket, returns file desecriptor on success, or -1 on error
        static int open_nb_socket(const std::string& addr, int port);

        std::unique_ptr<mqtt_client> m_client;
        std::unique_ptr<CallbackData> m_callback_data;
};
