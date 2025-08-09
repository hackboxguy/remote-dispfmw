/*
 * Raspberry Pi I2C Firmware Update Tool (Fixed)
 * Better timing, retry logic, and error handling
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <time.h>
#include "remote-dispfmw.h"

#define I2C_DEVICE "/dev/i2c-1"
#define RETRY_COUNT 5
#define RETRY_DELAY_MS 200

static int i2c_fd = -1;
static uint8_t seq_counter = 0;

// Open I2C device
static int i2c_open(void) {
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, ESP32_I2C_ADDR) < 0) {
        perror("Failed to set I2C slave address");
        close(i2c_fd);
        return -1;
    }
    
    return 0;
}

// Close I2C device
static void i2c_close(void) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

// Send command with improved timing and retry logic
static int send_command(fw_cmd_t cmd, const void* data, uint16_t data_len, 
                       uint8_t* response, size_t* resp_len) {
    uint8_t tx_buffer[512];
    uint8_t rx_buffer[512];
    
    // Prepare packet
    fw_packet_t* packet = (fw_packet_t*)tx_buffer;
    packet->cmd = cmd;
    packet->seq = ++seq_counter;
    packet->len = data_len;
    
    if (data && data_len > 0) {
        memcpy(packet->data, data, data_len);
    }
    
    size_t packet_size = sizeof(fw_packet_t) + data_len;
    
    printf("Sending command 0x%02X (seq: %d, len: %d, total: %zu bytes)...\n", 
           cmd, packet->seq, data_len, packet_size);
    
    // Send packet with improved retry logic
    for (int retry = 0; retry < RETRY_COUNT; retry++) {
        // Send the command
        ssize_t bytes_written = write(i2c_fd, tx_buffer, packet_size);
        if (bytes_written != packet_size) {
            printf("Write failed (attempt %d/%d): wrote %zd/%zu bytes: %s\n", 
                   retry + 1, RETRY_COUNT, bytes_written, packet_size, strerror(errno));
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }
        
        // Wait longer for ESP32 to process
        usleep(300 * 1000); // 300ms
        
        // Clear receive buffer
        memset(rx_buffer, 0xFF, sizeof(rx_buffer));
        
        // Read response
        ssize_t bytes_read = read(i2c_fd, rx_buffer, sizeof(rx_buffer));
        if (bytes_read < sizeof(fw_response_t)) {
            printf("Read failed or incomplete (attempt %d/%d): %zd bytes: %s\n", 
                   retry + 1, RETRY_COUNT, bytes_read, 
                   bytes_read < 0 ? strerror(errno) : "too short");
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }
        
        fw_response_t* resp = (fw_response_t*)rx_buffer;
        
        printf("Response: status=0x%02X, seq=%d, len=%d\n", resp->status, resp->seq, resp->len);
        
        // Check for valid response
        if (resp->status == 0xFF) {
            printf("Invalid response (attempt %d/%d): uninitialized data\n", retry + 1, RETRY_COUNT);
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }
        
        // Verify sequence number
        if (resp->seq != packet->seq) {
            printf("Sequence mismatch (attempt %d/%d): sent %d, got %d\n", 
                   retry + 1, RETRY_COUNT, packet->seq, resp->seq);
            
            // If we got a valid status but wrong sequence, it might be delayed response
            // Try one more time immediately
            if (retry == RETRY_COUNT - 1 && resp->status < 0x80) {
                printf("Got valid status but wrong sequence, trying once more...\n");
                continue;
            }
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }
        
        // Copy response data
        if (response && resp_len) {
            size_t copy_len = (resp->len < *resp_len) ? resp->len : *resp_len;
            if (copy_len > 0) {
                memcpy(response, resp->data, copy_len);
            }
            *resp_len = resp->len;
        }
        
        printf("Command successful, status: 0x%02X\n", resp->status);
        return resp->status;
    }
    
    printf("Command failed after %d retries\n", RETRY_COUNT);
    return -1;
}

// Get device information
static int get_device_info(device_info_t* info) {
    size_t resp_len = sizeof(device_info_t);
    int status = send_command(CMD_GET_INFO, NULL, 0, (uint8_t*)info, &resp_len);
    
    if (status == STATUS_OK) {
        printf("\nDevice Information:\n");
        printf("  Name: %.16s\n", info->device_name);
        printf("  ID: 0x%08X\n", info->device_id);
        printf("  Current Version: %d.%d.%d\n", 
               (info->current_version >> 16) & 0xFF,
               (info->current_version >> 8) & 0xFF,
               info->current_version & 0xFF);
        printf("  Flash Size: %d MB\n", info->flash_size / (1024*1024));
        printf("  HW Version: %d\n", info->hw_version);
        printf("  Boot Partition: %c\n", info->boot_partition ? 'B' : 'A');
        printf("  Max Chunk Size: %d bytes\n", info->max_chunk_size);
    }
    
    return status;
}

// Enhanced get_device_info function
static int get_device_info_enhanced(device_info_enhanced_t* info) {
    size_t resp_len = sizeof(device_info_enhanced_t);
    int status = send_command(CMD_GET_INFO, NULL, 0, (uint8_t*)info, &resp_len);
    
    if (status == STATUS_OK) {
        printf("\nDevice Information:\n");
        printf("  Name: %.16s\n", info->device_name);
        printf("  ID: 0x%08X\n", info->device_id);
        printf("  Current Version: %d.%d.%d\n", 
               (info->current_version >> 16) & 0xFF,
               (info->current_version >> 8) & 0xFF,
               info->current_version & 0xFF);
        printf("  Flash Size: %d MB\n", info->flash_size / (1024*1024));
        printf("  HW Version: %d\n", info->hw_version);
        
        // Enhanced partition information
        printf("\nPartition Information:\n");
        printf("  Current Partition: %.16s", info->current_partition_label);
        
        const char* partition_names[] = {"factory", "ota_0", "ota_1", "unknown"};
        const char* partition_letters[] = {"Factory", "App-A", "App-B", "Unknown"};
        uint8_t type_idx = info->current_partition_type < 3 ? info->current_partition_type : 3;
        
        printf(" (%s)\n", partition_letters[type_idx]);
        printf("  Boot Partition: %c\n", info->boot_partition ? 'B' : 'A');
        
        const char* ota_states[] = {"No OTA", "Pending", "Valid", "Invalid"};
        uint8_t state_idx = info->ota_state < 4 ? info->ota_state : 0;
        printf("  OTA State: %s\n", ota_states[state_idx]);
        
        printf("\nFirmware Versions:\n");
        printf("  Factory: %d.%d.%d%s\n", 
               (info->factory_version >> 16) & 0xFF,
               (info->factory_version >> 8) & 0xFF,
               info->factory_version & 0xFF,
               (info->current_partition_type == 0) ? " (CURRENT)" : "");
        
        if (info->ota_0_version > 0) {
            printf("  OTA_0 (App-A): %d.%d.%d%s\n", 
                   (info->ota_0_version >> 16) & 0xFF,
                   (info->ota_0_version >> 8) & 0xFF,
                   info->ota_0_version & 0xFF,
                   (info->current_partition_type == 1) ? " (CURRENT)" : "");
        } else {
            printf("  OTA_0 (App-A): Not programmed\n");
        }
        
        if (info->ota_1_version > 0) {
            printf("  OTA_1 (App-B): %d.%d.%d%s\n", 
                   (info->ota_1_version >> 16) & 0xFF,
                   (info->ota_1_version >> 8) & 0xFF,
                   info->ota_1_version & 0xFF,
                   (info->current_partition_type == 2) ? " (CURRENT)" : "");
        } else {
            printf("  OTA_1 (App-B): Not programmed\n");
        }
        
        printf("\nUpdate Configuration:\n");
        printf("  Max Chunk Size: %d bytes\n", info->max_chunk_size);
        
        // Summary
        printf("\nStatus Summary:\n");
        if (info->current_partition_type == 0) {
            printf("  🏭 Running FACTORY firmware\n");
            printf("  🔄 Next update will go to OTA_0 (App-A)\n");
        } else if (info->current_partition_type == 1) {
            printf("  📱 Running OTA_0 (App-A) firmware\n");
            printf("  🔄 Next update will go to OTA_1 (App-B)\n");
        } else if (info->current_partition_type == 2) {
            printf("  📱 Running OTA_1 (App-B) firmware\n");
            printf("  🔄 Next update will go to OTA_0 (App-A)\n");
        }
    }
    
    return status;
}

// Load firmware file and create header
static int load_firmware(const char* filename, uint8_t** fw_data, fw_header_t* header) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        perror("Failed to open firmware file");
        return -1;
    }
    
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (file_size <= 0 || file_size > (2 * 1024 * 1024)) {
        printf("Invalid firmware size: %ld bytes\n", file_size);
        fclose(file);
        return -1;
    }
    
    *fw_data = malloc(file_size);
    if (!*fw_data) {
        fclose(file);
        printf("Failed to allocate memory for firmware\n");
        return -1;
    }
    
    if (fread(*fw_data, 1, file_size, file) != file_size) {
        free(*fw_data);
        fclose(file);
        printf("Failed to read firmware file\n");
        return -1;
    }
    fclose(file);
    
    // Create header
    header->magic = 0xDEADBEEF;
    header->version = 0x010204; // New version
    header->size = file_size;
    header->crc32 = calculate_crc32(*fw_data, file_size);
    header->timestamp = time(NULL);
    header->hw_version = 1;
    memset(header->reserved, 0, sizeof(header->reserved));
    
    printf("Firmware loaded: %ld bytes, CRC32: 0x%08X\n", file_size, header->crc32);
    return 0;
}

// Update firmware with better error handling
static int update_firmware(const char* filename) {
    uint8_t* fw_data = NULL;
    fw_header_t header;
    int result = -1;
    
    if (load_firmware(filename, &fw_data, &header) < 0) {
        return -1;
    }
    
    printf("\nStarting firmware update...\n");
    
    // Start update
    int status = send_command(CMD_START_UPDATE, &header, sizeof(header), NULL, NULL);
    if (status != STATUS_OK) {
        printf("Failed to start update: status 0x%02X\n", status);
        goto cleanup;
    }
    
    printf("Update started successfully. Sending firmware data...\n");
    
    // Send firmware in chunks
    uint32_t bytes_sent = 0;
    uint16_t chunk_id = 0;
    
    while (bytes_sent < header.size) {
        fw_chunk_t chunk;
        chunk.chunk_id = chunk_id++;
        chunk.chunk_size = (header.size - bytes_sent > MAX_CHUNK_SIZE) ? 
                          MAX_CHUNK_SIZE : (header.size - bytes_sent);
        
        memcpy(chunk.chunk_data, fw_data + bytes_sent, chunk.chunk_size);
        chunk.chunk_crc = calculate_crc32(chunk.chunk_data, chunk.chunk_size);
        
        status = send_command(CMD_SEND_CHUNK, &chunk, sizeof(chunk), NULL, NULL);
        if (status != STATUS_OK) {
            printf("Failed to send chunk %d: status 0x%02X\n", chunk.chunk_id, status);
            goto cleanup;
        }
        
        bytes_sent += chunk.chunk_size;
        printf("Progress: %d/%d bytes (%.1f%%)\r", bytes_sent, header.size, 
               (float)bytes_sent / header.size * 100);
        fflush(stdout);
    }
    printf("\n");
    
    // Finish update
    printf("Completing firmware update...\n");
    status = send_command(CMD_FINISH_UPDATE, NULL, 0, NULL, NULL);
    if (status != STATUS_OK) {
        printf("Failed to finish update: status 0x%02X\n", status);
        goto cleanup;
    }
    
    printf("Firmware update completed successfully!\n");
    result = 0;
    
cleanup:
    if (fw_data) {
        free(fw_data);
    }
    return result;
}

// Activate firmware
static int activate_firmware(void) {
    printf("Activating new firmware...\n");
    int status = send_command(CMD_ACTIVATE_FW, NULL, 0, NULL, NULL);
    
    if (status == STATUS_OK) {
        printf("Firmware activation successful! Device will restart.\n");
        return 0;
    } else {
        printf("Failed to activate firmware: status 0x%02X\n", status);
        return -1;
    }
}

// Get status
static int get_status(void) {
    uint8_t status_data[8];
    size_t resp_len = sizeof(status_data);
    int status = send_command(CMD_GET_STATUS, NULL, 0, status_data, &resp_len);
    
    if (status == STATUS_OK && resp_len >= 7) {
        fw_state_t state = (fw_state_t)status_data[0];
        uint32_t bytes_received = *((uint32_t*)&status_data[1]);
        uint16_t chunk_count = *((uint16_t*)&status_data[5]);
        
        printf("\nUpdate Status:\n");
        printf("  State: ");
        switch (state) {
            case STATE_IDLE: printf("Idle\n"); break;
            case STATE_UPDATE_STARTED: printf("Update Started\n"); break;
            case STATE_RECEIVING_CHUNKS: printf("Receiving Chunks\n"); break;
            case STATE_VERIFYING: printf("Verifying\n"); break;
            case STATE_READY_TO_ACTIVATE: printf("Ready to Activate\n"); break;
            case STATE_ERROR: printf("Error\n"); break;
            default: printf("Unknown (%d)\n", state); break;
        }
        printf("  Bytes Received: %d\n", bytes_received);
        printf("  Chunks Received: %d\n", chunk_count);
        return 0;
    } else {
        printf("Failed to get status: 0x%02X\n", status);
        return -1;
    }
}

// Print usage
static void print_usage(const char* prog_name) {
    printf("Usage: %s <command> [options]\n", prog_name);
    printf("Commands:\n");
    printf("  info                    - Get device information\n");
    printf("  update <firmware.bin>   - Update firmware\n");
    printf("  activate               - Activate new firmware\n");
    printf("  status                 - Get update status\n");
    printf("  abort                  - Abort current update\n");
    printf("  reset                  - Reset device\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    printf("ESP32 Firmware Update Tool (Fixed)\n");
    printf("===================================\n");
    
    // Open I2C device
    if (i2c_open() < 0) {
        return 1;
    }
    
    int result = 0;
    
    if (strcmp(argv[1], "info") == 0) {
        device_info_enhanced_t info;
        if (get_device_info_enhanced(&info) != STATUS_OK) {
            result = 1;
        }
        
    } else if (strcmp(argv[1], "update") == 0) {
        if (argc < 3) {
            printf("Error: Firmware file required\n");
            print_usage(argv[0]);
            result = 1;
        } else {
            if (update_firmware(argv[2]) < 0) {
                result = 1;
            }
        }
        
    } else if (strcmp(argv[1], "activate") == 0) {
        if (activate_firmware() < 0) {
            result = 1;
        }
        
    } else if (strcmp(argv[1], "status") == 0) {
        if (get_status() < 0) {
            result = 1;
        }
        
    } else if (strcmp(argv[1], "abort") == 0) {
        printf("Aborting firmware update...\n");
        int status = send_command(CMD_ABORT_UPDATE, NULL, 0, NULL, NULL);
        if (status == STATUS_OK) {
            printf("Update aborted successfully\n");
        } else {
            printf("Failed to abort update: status 0x%02X\n", status);
            result = 1;
        }
        
    } else if (strcmp(argv[1], "reset") == 0) {
        printf("Resetting device...\n");
        int status = send_command(CMD_RESET_DEVICE, NULL, 0, NULL, NULL);
        if (status == STATUS_OK) {
            printf("Reset command sent successfully\n");
        } else {
            printf("Failed to reset device: status 0x%02X\n", status);
            result = 1;
        }
        
    } else {
        printf("Error: Unknown command '%s'\n", argv[1]);
        print_usage(argv[0]);
        result = 1;
    }
    
    i2c_close();
    return result;
}
