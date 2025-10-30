#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

// AHT20 address: CE pin low - 0x38, CE pin high - 0x39
#define AHT20_ADDRESS_0 (0x38)
#define AHT20_ADDRESS_1 (0x39)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_device_config_t aht20_device; /*!< Configuration for aht20 device */
} aht20_config_t;

typedef struct aht20_t *aht20_handle_t;

/**
 * @brief Initialize a AHT20 device handle
 *
 * This function allocates and returns a device handle that encapsulates I2C
 * device information for subsequent operations.
 *
 * @param[in] bus_handle I2C master bus handle (i2c_master_bus_handle_t)
 * @param[in] aht20_config Pointer to aht20_config_t to configure device
 *                         address/speed and century handling
 * @param[out] aht20_handle Returned device handle. The caller must call
 *                          aht20_deinit when the handle is no longer needed.
 * @return
 *      - ESP_OK: Initialization succeeded
 *      - ESP_ERR_INVALID_ARG: Invalid argument (e.g. NULL bus_handle or
 * aht20_config)
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 *      - Other error codes returned by underlying I2C operations
 */
esp_err_t aht20_init(i2c_master_bus_handle_t bus_handle,
                     const aht20_config_t *aht20_config,
                     aht20_handle_t *aht20_handle);

/**
 * @brief Deinitialize the AHT20 device
 *
 * Frees resources allocated by aht20_init and removes the device from the
 * I2C bus.
 *
 * @param[in] aht20_handle Device handle to free
 * @return
 *      - ESP_OK: Deinitialization succeeded
 *      - ESP_ERR_INVALID_ARG / ESP_ERR_NO_MEM: Invalid handle
 *      - Other error codes returned by i2c_master_bus_rm_device
 */
esp_err_t aht20_deinit(aht20_handle_t aht20_handle);

/**
 * @brief Read temperature and humidity from the AHT20 sensor
 *
 * This function performs a measurement transaction with the sensor and
 * returns stemperature and humidity.
 * The values are floating point human-readable units (degrees C and
 * percent RH).
 *
 * @param[in] aht20_handle Device handle returned from aht20_init
 * @param[out] temperature Pointer to receive converted temperature in degrees
 *                         Celsius
 * @param[out] humidity Pointer to receive relative humidity in percent (0.0 -
 *                      100.0)
 * @return
 *      - ESP_OK: Measurement succeeded and outputs were written
 *      - ESP_ERR_INVALID_ARG: One or more pointer arguments are NULL
 *      - ESP_ERR_INVALID_STATE: Sensor is not calibrated or is busy
 *      - Other ESP_ERR_* codes as returned by underlying I2C operations
 */
esp_err_t aht20_get_temperature_humidity(aht20_handle_t aht20_handle,
                                         float *temperature, float *humidity);

/**
 * @brief Read status flags from the AHT20 sensor
 *
 * Reads the device status register and extracts the calibration flag and
 * busy flag.
 *
 * @param[in] aht20_handle Device handle returned from aht20_init
 * @param[out] calibrated Pointer set to true if the sensor calibration bit is
 *                        set
 * @param[out] busy Pointer set to true if the sensor is busy performing a
 *                  smeasurement
 * @return
 *      - ESP_OK: Status read successfully
 *      - ESP_ERR_INVALID_ARG: One or more pointer arguments are NULL
 *      - Other ESP_ERR_* codes as returned by underlying I2C operations
 */
esp_err_t aht20_get_status(aht20_handle_t aht20_handle, bool *calibrated,
                           bool *busy);

/**
 * @brief Issue a software reset to the AHT20 sensor
 *
 * Sends the sensor reset command over I2C and waits briefly for the device
 * to complete its internal reset sequence.
 *
 * @param[in] aht20_handle Device handle returned from aht20_init
 * @return
 *      - ESP_OK: Reset command sent successfully
 *      - ESP_ERR_INVALID_ARG: Invalid device handle
 *      - Other ESP_ERR_* codes as returned by underlying I2C operations
 */
esp_err_t aht20_reset(aht20_handle_t aht20_handle);

#ifdef __cplusplus
}
#endif
