/*
 * ESP32-C6 Speed-Optimized I2C Firmware Update
 * Reduced logging and faster timing for improved performance
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
#include "remote-dispfmw.h"

static const char* TAG = "FW_UPDATE";

// Debug logging control - set to 0 for speed optimization
#define DEBUG_I2C_VERBOSE 0
#define DEBUG_CHUNK_DETAILS 0

// Global state
static fw_state_t g_state = STATE_IDLE;
static esp_ota_handle_t g_ota_handle = 0;
static const esp_partition_t* g_update_partition = NULL;
static fw_header_t g_fw_header;
static uint32_t g_bytes_received = 0;
static uint16_t g_expected_chunk = 0;

// I2C Configuration - optimized for speed
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

// Initialize I2C slave with optimized settings
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

// Send response - minimal logging for speed
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

    // Reduced logging - only log errors and important responses
    if (status != STATUS_OK || data_len > 0) {
        ESP_LOGI(TAG, "Response: status=0x%02X, seq=%d, len=%d", status, seq, data_len);
    }

    int tx_len = i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_buffer, response_size, pdMS_TO_TICKS(500));
    if (tx_len <= 0) {
        ESP_LOGW(TAG, "TX failed");
    }
}

// Handle GET_INFO command
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
    const esp_partition_t* ota_0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    const esp_partition_t* ota_1 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    if (running) {
        strncpy(info.current_partition_label, running->label, sizeof(info.current_partition_label) - 1);

        switch (running->subtype) {
            case ESP_PARTITION_SUBTYPE_APP_FACTORY:
                info.current_partition_type = 0;
                info.boot_partition = 0;
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_0:
                info.current_partition_type = 1;
                info.boot_partition = 0;
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_1:
                info.current_partition_type = 2;
                info.boot_partition = 1;
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
            case ESP_OTA_IMG_NEW:           info.ota_state = 1; break;
            case ESP_OTA_IMG_PENDING_VERIFY: info.ota_state = 1; break;
            case ESP_OTA_IMG_VALID:         info.ota_state = 2; break;
            case ESP_OTA_IMG_INVALID:       info.ota_state = 3; break;
            default:                        info.ota_state = 0; break;
        }
    } else {
        info.ota_state = 0;
    }

    // Set partition versions
    info.factory_version = 0x010200;
    info.ota_0_version = (ota_0 && running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) ? 0x010203 : 0;
    info.ota_1_version = (ota_1 && running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) ? 0x010203 : 0;
    info.max_chunk_size = MAX_CHUNK_SIZE;

    send_response(STATUS_OK, seq, &info, sizeof(info));
    ESP_LOGI(TAG, "Device info: %s", info.current_partition_label);
}

// Handle START_UPDATE command
static void handle_start_update(const fw_packet_t* packet) {
    if (packet->len != sizeof(fw_header_t)) {
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    memcpy(&g_fw_header, packet->data, sizeof(fw_header_t));

    if (g_fw_header.magic != 0xDEADBEEF) {
        ESP_LOGE(TAG, "Invalid magic: 0x%08X", g_fw_header.magic);
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    if (g_fw_header.size > (1024 * 1024) || g_fw_header.size == 0) {
        ESP_LOGE(TAG, "Invalid size: %d", g_fw_header.size);
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    g_update_partition = esp_ota_get_next_update_partition(NULL);
    if (!g_update_partition) {
        ESP_LOGE(TAG, "No OTA partition");
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
    ESP_LOGI(TAG, "Update started: %d bytes → %s", g_fw_header.size, g_update_partition->label);
}

// Handle SEND_CHUNK command - optimized for speed
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
        ESP_LOGW(TAG, "Chunk seq error: exp %d, got %d", g_expected_chunk, chunk->chunk_id);
        send_response(STATUS_ERROR, packet->seq, NULL, 0);
        return;
    }

    if (chunk->chunk_size > MAX_CHUNK_SIZE || chunk->chunk_size == 0) {
        send_response(STATUS_SIZE_ERROR, packet->seq, NULL, 0);
        return;
    }

    uint32_t calc_crc = calculate_crc32(chunk->chunk_data, chunk->chunk_size);
    if (calc_crc != chunk->chunk_crc) {
        ESP_LOGW(TAG, "CRC error: exp 0x%08X, got 0x%08X", chunk->chunk_crc, calc_crc);
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

    // Progress logging - only every 100 chunks for speed
    if (g_expected_chunk % 100 == 0) {
        ESP_LOGI(TAG, "Progress: %d/%d (%.1f%%) - %d chunks",
                 g_bytes_received, g_fw_header.size,
                 (float)g_bytes_received / g_fw_header.size * 100.0f,
                 g_expected_chunk);
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
                ESP_LOGE(TAG, "Size mismatch: %d/%d", g_bytes_received, g_fw_header.size);
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
            ESP_LOGI(TAG, "Update completed successfully");
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
            vTaskDelay(pdMS_TO_TICKS(500));  // Reduced delay
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
            vTaskDelay(pdMS_TO_TICKS(500));  // Reduced delay
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

// Speed-optimized fragment processing
static void process_fragment(const uint8_t* data, size_t len) {
    TickType_t current_time = xTaskGetTickCount();

    // Check for timeout
    if (g_packet_buffer_len > 0 &&
        (current_time - g_last_fragment_time) > pdMS_TO_TICKS(FRAGMENT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Fragment timeout, discarding %d bytes", g_packet_buffer_len);
        g_packet_buffer_len = 0;
    }

    // Enhanced validation for complete packets
    if (len >= sizeof(fw_packet_t)) {
        const fw_packet_t* potential_packet = (const fw_packet_t*)data;
        size_t expected_complete_len = sizeof(fw_packet_t) + potential_packet->len;

        bool is_valid_direct_packet =
            (len >= expected_complete_len) &&
            (potential_packet->cmd >= CMD_GET_INFO && potential_packet->cmd <= CMD_RESET_DEVICE) &&
            (potential_packet->len <= 200) &&
            (
                (g_state == STATE_IDLE) ||
                (potential_packet->cmd == CMD_GET_INFO) ||
                (potential_packet->cmd == CMD_GET_STATUS) ||
                (potential_packet->cmd == CMD_SEND_CHUNK && g_state >= STATE_UPDATE_STARTED) ||
                (potential_packet->cmd == CMD_FINISH_UPDATE && g_state == STATE_RECEIVING_CHUNKS) ||
                (potential_packet->cmd == CMD_ACTIVATE_FW && g_state == STATE_READY_TO_ACTIVATE) ||
                (potential_packet->cmd == CMD_ABORT_UPDATE && g_state >= STATE_UPDATE_STARTED)
            );

        if (is_valid_direct_packet) {
            #if DEBUG_I2C_VERBOSE
                ESP_LOGI(TAG, "Direct packet: cmd=0x%02X, seq=%d", potential_packet->cmd, potential_packet->seq);
            #endif

            if (g_packet_buffer_len > 0) {
                ESP_LOGW(TAG, "Discarding %d fragment bytes", g_packet_buffer_len);
                g_packet_buffer_len = 0;
            }

            switch (potential_packet->cmd) {
                case CMD_GET_INFO:
                    handle_get_info(potential_packet->seq);
                    break;
                case CMD_START_UPDATE:
                    handle_start_update(potential_packet);
                    break;
                case CMD_SEND_CHUNK:
                    handle_send_chunk(potential_packet);
                    break;
                default:
                    handle_simple_command(potential_packet);
                    break;
            }
            return;
        }
    }

    // Handle fragmented packets
    if (g_packet_buffer_len + len <= sizeof(g_packet_buffer)) {
        memcpy(g_packet_buffer + g_packet_buffer_len, data, len);
        g_packet_buffer_len += len;
        g_last_fragment_time = current_time;

        #if DEBUG_I2C_VERBOSE
            ESP_LOGI(TAG, "Fragment: +%d = %d bytes", len, g_packet_buffer_len);
        #endif

        if (is_complete_packet(g_packet_buffer, g_packet_buffer_len)) {
            const fw_packet_t* packet = (const fw_packet_t*)g_packet_buffer;

            bool is_valid_assembled_packet =
                (packet->cmd >= CMD_GET_INFO && packet->cmd <= CMD_RESET_DEVICE) &&
                (packet->len <= 200) &&
                (
                    (g_state == STATE_IDLE) ||
                    (packet->cmd == CMD_GET_INFO) ||
                    (packet->cmd == CMD_GET_STATUS) ||
                    (packet->cmd == CMD_SEND_CHUNK && g_state >= STATE_UPDATE_STARTED) ||
                    (packet->cmd == CMD_FINISH_UPDATE && g_state == STATE_RECEIVING_CHUNKS) ||
                    (packet->cmd == CMD_ACTIVATE_FW && g_state == STATE_READY_TO_ACTIVATE) ||
                    (packet->cmd == CMD_ABORT_UPDATE && g_state >= STATE_UPDATE_STARTED)
                );

            if (is_valid_assembled_packet) {
                #if DEBUG_I2C_VERBOSE
                    ESP_LOGI(TAG, "Assembled packet: cmd=0x%02X, seq=%d", packet->cmd, packet->seq);
                #endif

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
            } else {
                ESP_LOGW(TAG, "Invalid assembled packet: cmd=0x%02X, state=%d", packet->cmd, g_state);
            }

            g_packet_buffer_len = 0;
        }
    } else {
        ESP_LOGW(TAG, "Buffer overflow, discarding %d+%d bytes", g_packet_buffer_len, len);
        g_packet_buffer_len = 0;
    }
}

// Speed-optimized I2C task
static void i2c_task(void* arg) {
    uint8_t rx_buffer[I2C_SLAVE_RX_BUF_LEN];

    ESP_LOGI(TAG, "I2C task started (speed optimized)");

    g_packet_buffer_len = 0;
    g_last_fragment_time = 0;

    while (1) {
        memset(rx_buffer, 0, sizeof(rx_buffer));

        // Reduced timeout for faster polling
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(30));

        if (len > 0) {
            #if DEBUG_I2C_VERBOSE
                ESP_LOGD(TAG, "RX: %d bytes", len);
            #endif

            process_fragment(rx_buffer, len);
        } else if (len < 0) {
            ESP_LOGW(TAG, "I2C error: %d", len);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Reduced task delay for faster response
        vTaskDelay(pdMS_TO_TICKS(2));  // Much faster polling
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C6 Firmware Update Slave Starting (Speed Optimized)");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "Chip: ESP32-C6 Rev %d, %d cores, Flash: %d MB",
             chip_info.revision, chip_info.cores, flash_size / (1024 * 1024));

    esp_err_t err = i2c_slave_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    BaseType_t ret = xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 6, NULL);  // Higher priority
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create I2C task");
        return;
    }

    ESP_LOGI(TAG, "Ready (Address: 0x%02X, SDA: GPIO%d, SCL: GPIO%d)",
             ESP32_I2C_ADDR, I2C_SLAVE_SDA_IO, I2C_SLAVE_SCL_IO);

    const esp_partition_t* running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running: %s", running ? running->label : "unknown");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
