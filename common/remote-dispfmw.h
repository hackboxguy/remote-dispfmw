/*
 * I2C Firmware Update Protocol Definition
 * ESP32 Slave (0x42) <-> I2C Master
 */

#ifndef I2C_FW_UPDATE_H
#define I2C_FW_UPDATE_H

#include <stdint.h>

// I2C Configuration
#define ESP32_I2C_ADDR          0x42
#define I2C_TIMEOUT_MS          1000
#define MAX_CHUNK_SIZE          128
#define FW_HEADER_SIZE          32

// Command Definitions
typedef enum {
    CMD_GET_INFO        = 0x01,  // Get device info
    CMD_START_UPDATE    = 0x02,  // Start firmware update session
    CMD_SEND_CHUNK      = 0x03,  // Send firmware chunk
    CMD_VERIFY_CHUNK    = 0x04,  // Verify received chunk
    CMD_FINISH_UPDATE   = 0x05,  // Complete update and verify
    CMD_ACTIVATE_FW     = 0x06,  // Activate new firmware
    CMD_GET_STATUS      = 0x07,  // Get current status
    CMD_ABORT_UPDATE    = 0x08,  // Abort current update
    CMD_RESET_DEVICE    = 0x09,  // Reset device
    CMD_GET_VERSION     = 0x0A   // Get firmware version
} fw_cmd_t;

// Status Codes
typedef enum {
    STATUS_OK           = 0x00,
    STATUS_READY        = 0x01,
    STATUS_UPDATING     = 0x02,
    STATUS_VERIFYING    = 0x03,
    STATUS_ERROR        = 0x80,
    STATUS_CRC_ERROR    = 0x81,
    STATUS_SIZE_ERROR   = 0x82,
    STATUS_TIMEOUT      = 0x83,
    STATUS_INVALID_CMD  = 0x84,
    STATUS_UPDATE_FAIL  = 0x85
} fw_status_t;

// Protocol Structures
typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t seq;        // Sequence number for reliability
    uint16_t len;       // Data length
    uint8_t data[];     // Variable length data
} fw_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t seq;        // Echo sequence number
    uint16_t len;       // Response data length
    uint8_t data[];     // Response data
} fw_response_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;     // 0xDEADBEEF
    uint32_t version;   // Firmware version
    uint32_t size;      // Total firmware size
    uint32_t crc32;     // CRC32 of entire firmware
    uint32_t timestamp; // Build timestamp
    uint8_t  hw_version;// Target hardware version
    uint8_t  reserved[11];
} fw_header_t;

typedef struct __attribute__((packed)) {
    uint16_t chunk_id;  // Chunk sequence number
    uint16_t chunk_size;// This chunk size
    uint32_t chunk_crc; // CRC32 of this chunk
    uint8_t  chunk_data[MAX_CHUNK_SIZE];
} fw_chunk_t;

typedef struct __attribute__((packed)) {
    char device_name[16];
    uint32_t device_id;
    uint32_t current_version;
    uint32_t flash_size;
    uint8_t  hw_version;
    uint8_t  boot_partition; // 0=A, 1=B
    uint16_t max_chunk_size;
} device_info_t;

typedef struct __attribute__((packed)) {
    char device_name[16];
    uint32_t device_id;
    uint32_t current_version;
    uint32_t flash_size;
    uint8_t  hw_version;

    // Enhanced partition information
    uint8_t  current_partition_type;    // 0=factory, 1=ota_0, 2=ota_1
    char     current_partition_label[16]; // "factory", "ota_0", "ota_1"
    uint8_t  boot_partition;            // 0=A, 1=B (for display)
    uint8_t  ota_state;                 // 0=no_ota, 1=pending, 2=valid, 3=invalid

    uint32_t factory_version;           // Version of factory partition
    uint32_t ota_0_version;             // Version of ota_0 partition (0 if empty)
    uint32_t ota_1_version;             // Version of ota_1 partition (0 if empty)

    uint16_t max_chunk_size;
    uint8_t  reserved[6];               // Future expansion
} device_info_enhanced_t;

// Protocol Functions
static inline uint32_t calculate_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;
}

// Protocol State Machine
typedef enum {
    STATE_IDLE,
    STATE_UPDATE_STARTED,
    STATE_RECEIVING_CHUNKS,
    STATE_VERIFYING,
    STATE_READY_TO_ACTIVATE,
    STATE_ERROR
} fw_state_t;

#endif // I2C_FW_UPDATE_H
