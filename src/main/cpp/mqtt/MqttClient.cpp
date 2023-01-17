#include "mqtt/MqttClient.h"
#include "logging.h"
#include <mqtt/mqtt.h>
#include <optional>
#include <stdexcept>
#include <unistd.h>

CallbackData::CallbackData(const std::string& host, int port):
host(host),
port(port),
client_name(std::string("rio_") + std::to_string(getpid())) {}

MqttClient::MqttClient(std::unique_ptr<mqtt_client>&& client, std::unique_ptr<CallbackData>&& callback_data):
m_client(std::move(client)),
m_callback_data(std::move(callback_data)) {}

MqttClient::~MqttClient() {
    if (m_callback_data.get() != nullptr && m_callback_data->sockfd != -1) {
        close(m_callback_data->sockfd);
    }
}

std::optional<MqttClient> MqttClient::create(const std::string& host, int port) {
    auto *callback_data = new CallbackData(host, port);

    auto out = std::make_optional<MqttClient>(
        MqttClient(std::unique_ptr<mqtt_client>(new mqtt_client),
            std::unique_ptr<CallbackData>(callback_data)));

    mqtt_init_reconnect(out->m_client.get(), reconnect_callback, out->m_callback_data.get(), message_callback);
    out->m_client->publish_response_callback_state = out->m_callback_data.get();

    if (out->m_client->error != MQTT_OK) {
        return {};
    }

    return out;
}

Error MqttClient::update() {
    return get_error(mqtt_sync(m_client.get()));
}

Error MqttClient::publish(const std::string& topic, std::string_view payload) {
    return get_error(mqtt_publish(m_client.get(), topic.c_str(), (void *) payload.data(), payload.size(), MQTT_PUBLISH_QOS_0));
}

void MqttClient::unsubscribe(const std::string& topic) {
    m_callback_data->callbacks.erase(topic);
    if (mqtt_unsubscribe(m_client.get(), topic.c_str()) != MQTT_OK) {
        // don't return an error, since this isn't an error that needs to be handled
        // MqttClient will handle it by not calling the callback anymore
        lg::warn("failed to tell mqtt broker to unsubscribe from mqtt topic %s", topic.c_str());
    }
}

Error MqttClient::get_error(MQTTErrors error) {
    if (error == MQTT_OK) {
        return Error::ok();
    } else {
        return Error::library(mqtt_error_str(error));
    }
}

void MqttClient::reconnect_callback(mqtt_client *client, void **state) {
    CallbackData *callback_data = *((CallbackData **) state);

    // clean session if this is the first time connecting
    u8 connect_flags = 0;
    if (client->error == MQTT_ERROR_INITIAL_RECONNECT) {
        connect_flags = MQTT_CONNECT_CLEAN_SESSION;
    }

    if (callback_data->sockfd != -1) {
        close(callback_data->sockfd);
        callback_data->sockfd = -1;
    }

    lg::warn("mqtt connection lost, reconnecting...");

    int sockfd = open_nb_socket(callback_data->host, callback_data->port);
    if (sockfd == -1) {
        lg::error("failed to reconnect");
        return;
    }
    callback_data->sockfd = sockfd;

    // this won't fail
    mqtt_reinit(client, sockfd, callback_data->sendbuf, SEND_BUF_LEN, callback_data->recvbuf, RECV_BUF_LEN);

    mqtt_connect(client, callback_data->client_name.c_str(), nullptr, nullptr, 0, nullptr, nullptr, connect_flags, 400);
    if (client->error != MQTT_OK) {
        lg::error("failed to reconnect");
        return;
    }

    // TODO: figure out if subscriptions are preserved
}

void MqttClient::message_callback(void **state, mqtt_response_publish *msg) {
    CallbackData *callback_data = *((CallbackData **) state);
    // TODO: remove this allocation
    std::string topic((const char *) msg->topic_name, msg->topic_name_size);

    try {
        auto callback = callback_data->callbacks.at(topic);
        callback.callback(std::string_view((char *) msg->application_message, msg->application_message_size), callback.data);
    } catch (const std::out_of_range& err) {
        lg::warn("warning: no callback for topic %s", topic.c_str());
        return;
    }
}

int MqttClient::open_nb_socket(const std::string& addr, int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0); // IPv4, byte stream, non-blocking socket
    if (sockfd == -1) {
        return -1;
    }

    struct sockaddr_in addrs;
    addrs.sin_family = AF_INET;
    addrs.sin_port = htons(port); // Convert to right endianness
    addrs.sin_addr.s_addr = inet_addr(addr.c_str());

    int ret = connect(sockfd, (struct sockaddr *) &addrs, sizeof(addrs));

    if (ret == -1) {
        // TODO: figure out if this is right
        close(sockfd);
        return -1;
    }
    return sockfd;
}