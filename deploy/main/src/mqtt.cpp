#include "mqtt.hpp"

namespace mqtt {
esp_mqtt_client_handle_t client = nullptr;
extern const char isrgrootx1_pem_start[] asm("_binary_isrgrootx1_pem_start");
void send_image(const uint8_t* jpeg_data, size_t jpeg_size)
{
    int msg_id = esp_mqtt_client_publish(client, "image", (const char *)jpeg_data, jpeg_size, 0, 0);
    ESP_LOGI("MQTT", "JPEG sent with msg_id=%d, size=%zu bytes", msg_id, jpeg_size);
}

void send_classification(const dl::cls::result_t &classification)
{
    char classification_str[256];
    snprintf(classification_str, sizeof(classification_str), "%s,%.4f", classification.cat_name, classification.score);
    int msg_id = esp_mqtt_client_publish(client, "classification", (const char *)classification_str, strlen(classification_str), 0, 0);
    ESP_LOGI("MQTT", "Classification sent with msg_id=%d, size=%zu bytes", msg_id, strlen(classification_str));
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void* event_data)
{
    ESP_LOGD("MQTT", "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI("MQTT", "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI("MQTT", "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI("MQTT", "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        // if (strncmp(event->data, "send binary please", event->data_len) == 0) {
        //     ESP_LOGI("MQTT", "Sending the binary");
        //     send_binary(client);
        // }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI("MQTT", "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI("MQTT", "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI("MQTT", "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI("MQTT", "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW("MQTT", "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
            .verification = {
                .certificate = (const char *)isrgrootx1_pem_start,
            }
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication = {
                .password = MQTT_PASSWORD,
            },
        },
    };

    ESP_LOGI("MQTT", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
}
