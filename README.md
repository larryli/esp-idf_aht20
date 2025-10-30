# AHT20 temperature and humidity sensor driver component for esp-idf

## Installation

    idf.py add-dependency "larryli/aht20"

## Getting Started

### New i2c master

```c
#include "driver/i2c_master.h"

i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = PORT_NUMBER,
    .scl_io_num = SCL_IO_PIN,
    .sda_io_num = SDA_IO_PIN,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;

i2c_new_master_bus(&i2c_bus_config, &bus_handle);
```

### Init AHT20 device

```c
#include "aht20.h"

aht20_config_t aht20_config = {
    .aht20_device.scl_speed_hz = MASTER_FREQUENCY,
    .aht20_device.device_address = AHT20_ADDRESS,
};

aht20_handle_t aht20_handle;
aht20_init(bus_handle, &aht20_config, &aht20_handle);
```

### Get temperature and humidity

```c
float temperature, humidity;

aht20_get_temperature_humidity(aht20_handle, NULL, &temperature, NULL, &humidity);
```
