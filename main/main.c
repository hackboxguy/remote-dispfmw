/*
 * ESP32-C6 Fragmented I2C Packet Handler
 * Handles I2C packet fragmentation and reassembly
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "i2c_fw_update.h"

static const char* TAG = "FW_UPDATE";

// Global state
static fw_state_t g_state = STATE_IDLE;
static esp_ota_handle_t g_ota_handle = 0;
static const esp_partition_t* g_update_partition = NULL;
static fw_header_t g_fw_header;
static uint32_t g_bytes_received = 0;
static uint16_t g_expected_chunk = 0;

// I2C Configuration
#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_SDA_IO        GPIO_NUM_6
#define I2C_SLAVE_SCL_IO        GPIO_NUM_7
#define I2C_SLAVE_RX_BUF_LEN    512
#define I2C_SLAVE_TX_BUF_LEN    256

// Packet reassembly buffer
static uint8_t g_packet_buffer[512];
static size_t g_packet_buffer_len = 0;
static TickType_t g_last_fragment_time = 0;
#define FRAGMENT_TIMEOUT_MS     1000

// Initialize I2C slave
esp_err_t i2c_slave_init(void) {
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP32_I2C_ADDR,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode,
                             I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

// Send response
static void send_response(fw_status_t status, uint8_t seq, const void* data, uint16_t data_len) {
    uint8_t tx_buffer[I2C_SLAVE_TX_BUF_LEN];
    fw_response_t* resp = (fw_response_t*)tx_buffer;

    memset(tx_buffer, 0, sizeof(tx_buffer));

    resp->status = status;
    resp->seq = seq;
    resp->len = data_len;

    if (data && data_len > 0 && data_len < (I2C_SLAVE_TX_BUF_LEN - sizeof(fw_response_t))) {
        memcpy(resp->data, data, data_len);
    }

    size_t response_size = sizeof(fw_response_t) + data_len;

    ESP_LOGI(TAG, "Sending response: status=0x%02X, seq=%d, len=%d", status, seq, data_len);

    int tx_len = i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_buffer, response_size, pdMS_TO_TICKS(1000));
    if (tx_len <= 0) {
        ESP_LOGW(TAG, "Failed to send response");
    }
}

// Handle GET_INFO command
/*static void handle_get_info(uint8_t seq) {
    device_info_t info = {0};

    strncpy(info.device_name, "ESP32C6-FW", sizeof(info.device_name) - 1);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    info.device_id = 0xC6010000 | (chip_info.revision << 8);
    info.current_version = 0x010203;

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    info.flash_size = flash_size;
    info.hw_version = chip_info.revision;

    const esp_partition_t* running = esp_ota_get_running_partition();
    info.boot_partition = (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) ? 1 : 0;
    info.max_chunk_size = MAX_CHUNK_SIZE;

    send_response(STATUS_OK, seq, &info, sizeof(info));
    ESP_LOGI(TAG, "Device info requested");
}*/
static void handle_get_info(uint8_t seq) {
    device_info_enhanced_t info = {0};

    strncpy(info.device_name, "ESP32C6-FW", sizeof(info.device_name) - 1);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    info.device_id = 0xC6010000 | (chip_info.revision << 8);
    info.current_version = 0x010203;

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    info.flash_size = flash_size;
    info.hw_version = chip_info.revision;

    // Get detailed partition information
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    const esp_partition_t* ota_0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    const esp_partition_t* ota_1 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    // Determine current partition type and label
    if (running) {
        strncpy(info.current_partition_label, running->label, sizeof(info.current_partition_label) - 1);

        switch (running->subtype) {
            case ESP_PARTITION_SUBTYPE_APP_FACTORY:
                info.current_partition_type = 0;
                info.boot_partition = 0; // Factory = A
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_0:
                info.current_partition_type = 1;
                info.boot_partition = 0; // ota_0 = A
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_1:
                info.current_partition_type = 2;
                info.boot_partition = 1; // ota_1 = B
                break;
            default:
                info.current_partition_type = 0xFF;
                info.boot_partition = 0xFF;
                break;
        }
    }

    // Get OTA state
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        switch (ota_state) {
            case ESP_OTA_IMG_NEW:           info.ota_state = 1; break; // Pending
            case ESP_OTA_IMG_PENDING_VERIFY: info.ota_state = 1; break; // Pending
            case ESP_OTA_IMG_VALID:         info.ota_state = 2; break; // Valid
            case ESP_OTA_IMG_INVALID:       info.ota_state = 3; break; // Invalid
            default:                        info.ota_state = 0; break; // No OTA
        }
    } else {
        info.ota_state = 0; // No OTA or factory
    }

    // Set partition versions (simplified - you can enhance this)
    info.factory_version = 0x010200;  // Factory version
    info.ota_0_version = (ota_0 && running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) ? 0x010203 : 0;
    info.ota_1_version = (ota_1 && running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) ? 0x010203 : 0;

    info.max_chunk_size = MAX_CHUNK_SIZE;

    send_response(STATUS_OK, seq, &info, sizeof(info));

    ESP_LOGI(TAG, "Device info requested - Running: %s (type=%d, state=%d)",
             info.current_partition_label, info.current_partition_type, info.ota_state);
}

// Handle START_UPDATE command
static void handle_start_update(const fw_packet_t* packet) {
    ESP_LOGI(TAG, "Start update: packet len=%d, expected=%d", packet->len, sizeof(fw_header_t));

    if (packet->len != sizeof(fw_header_t)) {
        ESP_LOGW(TAG, "Invalid header size: got %d, expected %d", packet->len, sizeof(fw_header_t));
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    memcpy(&g_fw_header, packet->data, sizeof(fw_header_t));

    ESP_LOGI(TAG, "Firmware header: magic=0x%08X, size=%d, crc=0x%08X",
             g_fw_header.magic, g_fw_header.size, g_fw_header.crc32);

    if (g_fw_header.magic != 0xDEADBEEF) {
        ESP_LOGE(TAG, "Invalid firmware magic: 0x%08X", g_fw_header.magic);
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    if (g_fw_header.size > (1024 * 1024) || g_fw_header.size == 0) {
        ESP_LOGE(TAG, "Invalid firmware size: %d bytes", g_fw_header.size);
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    g_update_partition = esp_ota_get_next_update_partition(NULL);
    if (!g_update_partition) {
        ESP_LOGE(TAG, "No OTA partition available");
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    esp_err_t err = esp_ota_begin(g_update_partition, g_fw_header.size, &g_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        send_response(STATUS_UPDATE_FAIL, packet->seq, NULL, 0);
        return;
    }

    g_state = STATE_UPDATE_STARTED;
    g_bytes_received = 0;
    g_expected_chunk = 0;

    send_response(STATUS_OK, packet->seq, NULL, 0);
    ESP_LOGI(TAG, "Update started successfully, size: %d bytes", g_fw_header.size);
}

// Handle SEND_CHUNK command
static void handle_send_chunk(const fw_packet_t* packet) {
    if (g_state != STATE_UPDATE_STARTED && g_state != STATE_RECEIVING_CHUNKS) {
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    if (packet->len != sizeof(fw_chunk_t)) {
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    fw_chunk_t* chunk = (fw_chunk_t*)packet->data;

    if (chunk->chunk_id != g_expected_chunk) {
        ESP_LOGW(TAG, "Chunk sequence error: expected %d, got %d", g_expected_chunk, chunk->chunk_id);
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    if (chunk->chunk_size > MAX_CHUNK_SIZE || chunk->chunk_size == 0) {
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    uint32_t calc_crc = calculate_crc32(chunk->chunk_data, chunk->chunk_size);
    if (calc_crc != chunk->chunk_crc) {
        ESP_LOGW(TAG, "Chunk CRC error: expected 0x%08X, got 0x%08X", chunk->chunk_crc, calc_crc);
        send_response(STATUS_CRC_ERROR, packet->seq, NULL, 0);
        return;
    }

    esp_err_t err = esp_ota_write(g_ota_handle, chunk->chunk_data, chunk->chunk_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        send_response(STATUS_UPDATE_FAIL, packet->seq, NULL, 0);
        return;
    }

    g_bytes_received += chunk->chunk_size;
    g_expected_chunk++;
    g_state = STATE_RECEIVING_CHUNKS;

    send_response(STATUS_OK, packet->seq, NULL, 0);

    if (g_expected_chunk % 10 == 0) {
        ESP_LOGI(TAG, "Progress: %d/%d bytes (%.1f%%)",
                 g_bytes_received, g_fw_header.size,
                 (float)g_bytes_received / g_fw_header.size * 100.0f);
    }
}

// Handle other commands
static void handle_simple_command(const fw_packet_t* packet) {
    switch (packet->cmd) {
        case CMD_GET_STATUS: {
            uint8_t status_data[8] = {0};
            status_data[0] = g_state;
            *((uint32_t*)&status_data[1]) = g_bytes_received;
            *((uint16_t*)&status_data[5]) = g_expected_chunk;
            send_response(STATUS_OK, packet->seq, status_data, sizeof(status_data));
            break;
        }

        case CMD_FINISH_UPDATE:
            if (g_state != STATE_RECEIVING_CHUNKS) {
                send_response(STATUS_ERROR, packet->seq, NULL, 0);
                return;
            }
            if (g_bytes_received != g_fw_header.size) {
                ESP_LOGE(TAG, "Size mismatch: expected %d, received %d", g_fw_header.size, g_bytes_received);
                send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
                return;
            }

            esp_err_t err = esp_ota_end(g_ota_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
                send_response(STATUS_UPDATE_FAIL, packet->seq, NULL, 0);
                return;
            }

            g_ota_handle = 0;
            g_state = STATE_READY_TO_ACTIVATE;
            send_response(STATUS_OK, packet->seq, NULL, 0);
            ESP_LOGI(TAG, "Firmware update completed successfully");
            break;

        case CMD_ACTIVATE_FW:
            if (g_state != STATE_READY_TO_ACTIVATE) {
                send_response(STATUS_ERROR, packet->seq, NULL, 0);
                return;
            }

            err = esp_ota_set_boot_partition(g_update_partition);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(err));
                send_response(STATUS_UPDATE_FAIL, packet->seq, NULL, 0);
                return;
            }

            send_response(STATUS_OK, packet->seq, NULL, 0);
            ESP_LOGI(TAG, "Firmware activated, restarting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
            break;

        case CMD_ABORT_UPDATE:
            if (g_ota_handle) {
                esp_ota_abort(g_ota_handle);
                g_ota_handle = 0;
            }
            g_state = STATE_IDLE;
            send_response(STATUS_OK, packet->seq, NULL, 0);
            ESP_LOGI(TAG, "Update aborted");
            break;

        case CMD_RESET_DEVICE:
            send_response(STATUS_OK, packet->seq, NULL, 0);
            ESP_LOGI(TAG, "Reset requested");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
            break;

        default:
            send_response(STATUS_INVALID_CMD, packet->seq, NULL, 0);
            ESP_LOGW(TAG, "Unknown command: 0x%02X", packet->cmd);
            break;
    }
}

// Check if we have a complete packet
static bool is_complete_packet(const uint8_t* buffer, size_t len) {
    if (len < sizeof(fw_packet_t)) {
        return false;
    }

    const fw_packet_t* packet = (const fw_packet_t*)buffer;
    size_t expected_len = sizeof(fw_packet_t) + packet->len;

    return len >= expected_len;
}

// Process fragmented data
static void process_fragment(const uint8_t* data, size_t len) {
    TickType_t current_time = xTaskGetTickCount();

    // Check for timeout (new packet started)
    if (g_packet_buffer_len > 0 &&
        (current_time - g_last_fragment_time) > pdMS_TO_TICKS(FRAGMENT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Fragment timeout, discarding %d bytes", g_packet_buffer_len);
        g_packet_buffer_len = 0;
    }

    // Add new data to buffer
    if (g_packet_buffer_len + len <= sizeof(g_packet_buffer)) {
        memcpy(g_packet_buffer + g_packet_buffer_len, data, len);
        g_packet_buffer_len += len;
        g_last_fragment_time = current_time;

        ESP_LOGI(TAG, "Fragment: added %d bytes, total %d bytes", len, g_packet_buffer_len);

        // Check if we have a complete packet
        if (is_complete_packet(g_packet_buffer, g_packet_buffer_len)) {
            const fw_packet_t* packet = (const fw_packet_t*)g_packet_buffer;
            size_t packet_len = sizeof(fw_packet_t) + packet->len;

            ESP_LOGI(TAG, "Complete packet assembled: cmd=0x%02X, seq=%d, len=%d",
                     packet->cmd, packet->seq, packet->len);

            // Process the complete packet
            switch (packet->cmd) {
                case CMD_GET_INFO:
                    handle_get_info(packet->seq);
                    break;

                case CMD_START_UPDATE:
                    handle_start_update(packet);
                    break;

                case CMD_SEND_CHUNK:
                    handle_send_chunk(packet);
                    break;

                default:
                    handle_simple_command(packet);
                    break;
            }

            // Clear buffer
            g_packet_buffer_len = 0;
        }
    } else {
        ESP_LOGW(TAG, "Fragment buffer overflow, discarding");
        g_packet_buffer_len = 0;
    }
}

// I2C communication task
static void i2c_task(void* arg) {
    uint8_t rx_buffer[I2C_SLAVE_RX_BUF_LEN];

    ESP_LOGI(TAG, "I2C communication task started");

    // Initialize fragment buffer
    g_packet_buffer_len = 0;
    g_last_fragment_time = 0;

    while (1) {
        memset(rx_buffer, 0, sizeof(rx_buffer));

        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(100));

        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes", len);
            ESP_LOGI(TAG, "RX[0-7]: %02X %02X %02X %02X %02X %02X %02X %02X",
                     rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
                     rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]);

            // Handle as fragment
            process_fragment(rx_buffer, len);
        } else if (len < 0) {
            ESP_LOGW(TAG, "I2C read error: %d", len);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C6 Firmware Update Slave Starting (Fragmented I2C)");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "Chip: ESP32-C6 Rev %d, %d cores, Flash: %d MB",
             chip_info.revision, chip_info.cores, flash_size / (1024 * 1024));

    esp_err_t err = i2c_slave_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave init failed: %s", esp_err_to_name(err));
        return;
    }

    BaseType_t ret = xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create I2C task");
        return;
    }

    ESP_LOGI(TAG, "Firmware Update Slave Ready (Address: 0x%02X, SDA: GPIO%d, SCL: GPIO%d)",
             ESP32_I2C_ADDR, I2C_SLAVE_SDA_IO, I2C_SLAVE_SCL_IO);

    const esp_partition_t* running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition: %s", running ? running->label : "unknown");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGD(TAG, "Firmware update slave running, state: %d", g_state);
    }
}
