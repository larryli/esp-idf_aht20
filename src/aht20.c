#include "aht20.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUF_SIZE (7)
#define START_MEASURMENT_CMD (0xAC)
#define CMD_RESET (0xBA)
#define STATUS_CALIBRATION_ENABLE (3)
#define STATUS_CRC_FLAG (4)
#define STATUS_BUSY_INDICATION (7)

static const char TAG[] = "aht20";

struct aht20_t {
    i2c_master_dev_handle_t i2c_dev; /*!< I2C device handle */
};

esp_err_t aht20_init(i2c_master_bus_handle_t bus_handle,
                     const aht20_config_t *aht20_config,
                     aht20_handle_t *aht20_handle)
{
    ESP_RETURN_ON_FALSE(bus_handle, ESP_ERR_INVALID_ARG, TAG,
                        "invalid i2c master bus");
    ESP_RETURN_ON_FALSE(aht20_config, ESP_ERR_INVALID_ARG, TAG,
                        "invalid aht20 config");
    esp_err_t ret = ESP_OK;
    aht20_handle_t out_handle =
        (aht20_handle_t)calloc(1, sizeof(struct aht20_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG,
                      "no memory for i2c aht20 device");

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = aht20_config->aht20_device.scl_speed_hz,
        .device_address = aht20_config->aht20_device.device_address,
    };
    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf,
                                                    &out_handle->i2c_dev),
                          err, TAG, "i2c new bus failed");
    }

    *aht20_handle = out_handle;

    return ESP_OK;

err:
    if (out_handle && out_handle->i2c_dev) {
        i2c_master_bus_rm_device(out_handle->i2c_dev);
    }
    free(out_handle);
    return ret;
}

esp_err_t aht20_deinit(aht20_handle_t aht20_handle)
{
    ESP_RETURN_ON_FALSE(aht20_handle, ESP_ERR_NO_MEM, TAG,
                        "invalid aht20 handle");
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(aht20_handle->i2c_dev), TAG,
                        "rm i2c device failed");
    free(aht20_handle);
    return ESP_OK;
}

static uint8_t calc_crc(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t byte;
    uint8_t crc = 0xFF;

    for (byte = 0; byte < len; byte++) {
        crc ^= data[byte];
        for (i = 8; i > 0; --i) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

esp_err_t aht20_get_temperature_humidity(aht20_handle_t aht20_handle,
                                         float *temperature, float *humidity)
{
    ESP_RETURN_ON_FALSE(aht20_handle, ESP_ERR_NO_MEM, TAG,
                        "invalid aht20 handle");
    ESP_RETURN_ON_FALSE(temperature, ESP_ERR_NO_MEM, TAG,
                        "invalid temperature handle");
    ESP_RETURN_ON_FALSE(humidity, ESP_ERR_NO_MEM, TAG,
                        "invalid humidity handle");

    uint8_t buf[BUF_SIZE] = {START_MEASURMENT_CMD, 0x33, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(aht20_handle->i2c_dev, buf, 3, -1),
                        TAG, "i2c write failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t status;
    for (int i = 0; i < 10; i++) {
        ESP_RETURN_ON_ERROR(i2c_master_receive(aht20_handle->i2c_dev, &status,
                                               sizeof(status), -1),
                            TAG, "i2c read failed");
        if ((status & BIT(STATUS_BUSY_INDICATION)) == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if ((status & BIT(STATUS_CALIBRATION_ENABLE)) &&
        (status & BIT(STATUS_CRC_FLAG)) &&
        ((status & BIT(STATUS_BUSY_INDICATION)) == 0)) {
        ESP_RETURN_ON_ERROR(
            i2c_master_receive(aht20_handle->i2c_dev, buf, sizeof(buf), -1),
            TAG, "i2c read failed");
        ESP_RETURN_ON_ERROR(
            (calc_crc(buf, sizeof(buf) - 1) != buf[sizeof(buf) - 1]), TAG,
            "crc is error");

        uint32_t raw_data;
        raw_data = buf[1];
        raw_data = raw_data << 8;
        raw_data += buf[2];
        raw_data = raw_data << 8;
        raw_data += buf[3];
        raw_data = raw_data >> 4;
        if (humidity) {
            *humidity = (float)raw_data * 100 / 1048576;
        }

        raw_data = buf[3] & 0x0F;
        raw_data = raw_data << 8;
        raw_data += buf[4];
        raw_data = raw_data << 8;
        raw_data += buf[5];
        if (temperature) {
            *temperature = (float)raw_data * 200 / 1048576 - 50;
        }
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "not calibrated or busy");
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t aht20_get_status(aht20_handle_t aht20_handle, bool *calibrated,
                           bool *busy)
{
    ESP_RETURN_ON_FALSE(aht20_handle, ESP_ERR_NO_MEM, TAG,
                        "invalid aht20 handle");
    ESP_RETURN_ON_FALSE(calibrated, ESP_ERR_NO_MEM, TAG,
                        "invalid calibrated handle");
    ESP_RETURN_ON_FALSE(busy, ESP_ERR_NO_MEM, TAG, "invalid busy handle");

    uint8_t status;
    ESP_RETURN_ON_ERROR(
        i2c_master_receive(aht20_handle->i2c_dev, &status, sizeof(status), -1),
        TAG, "i2c read failed");

    if (calibrated) {
        *calibrated = (status & BIT(STATUS_CALIBRATION_ENABLE)) != 0;
    }
    if (busy) {
        *busy = (status & BIT(STATUS_BUSY_INDICATION)) != 0;
    }
    return ESP_OK;
}

esp_err_t aht20_reset(aht20_handle_t aht20_handle)
{
    ESP_RETURN_ON_FALSE(aht20_handle, ESP_ERR_NO_MEM, TAG,
                        "invalid aht20 handle");

    uint8_t cmd = 0xBA;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(aht20_handle->i2c_dev, &cmd, sizeof(cmd), -1), TAG,
        "i2c write failed");
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}
