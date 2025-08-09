idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "../common"
    REQUIRES
        "driver"
        "esp_partition"
        "app_update"
        "esp_system"
        "freertos"
        "esp_hw_support"
        "spi_flash"
)
