#include "aht20.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCL_IO_PIN CONFIG_I2C_MASTER_SCL
#define SDA_IO_PIN CONFIG_I2C_MASTER_SDA
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define PORT_NUMBER -1

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Start");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    uint8_t n = 0, addr[2] = {0, 0};
    for (int i = 0; i < 128; i++) {
        esp_err_t ret = i2c_master_probe(bus_handle, i, 50);
        if (ret == ESP_OK) {
            if (i == AHT20_ADDRESS_0 || i == AHT20_ADDRESS_1) {
                addr[n++] = i;
                ESP_LOGI(TAG, "found aht20 address: 0x%02X", i);
            } else {
                ESP_LOGI(TAG, "found i2c device address: 0x%02X", i);
            }
        }
    }

    aht20_handle_t aht20_handle[2];
    for (int i = 0; i < n; i++) {
        if (addr[i]) {
            const aht20_config_t aht20_config = {
                .aht20_device.device_address = addr[i],
                .aht20_device.scl_speed_hz = MASTER_FREQUENCY,
            };
            ESP_ERROR_CHECK(
                aht20_init(bus_handle, &aht20_config, &(aht20_handle[i])));
        }
    }

    while (1) {
        for (int i = 0; i < n; i++) {
            if (addr[i]) {
                float temperature, humidity;
                ESP_ERROR_CHECK(aht20_get_temperature_humidity(
                    aht20_handle[i], &temperature, &humidity));
                ESP_LOGI(TAG,
                         "aht20 address: 0x%02X, temperature: %.2f C,"
                         "humidity: %.2f %%RH",
                         addr[i], temperature, humidity);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
