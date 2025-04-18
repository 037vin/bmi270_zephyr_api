# BMI270 sensor API for zephyr

This is a software project that enables configuration and operation of the BMI270 sensor from Bosch Sensortec in the Zephyr environment using SPI/I2C interface. The software is dedicated to the nRF5340 DK board from Nordic Semiconductor.

## Project compilation
Place the files bmi270_api.c, bmi270_api.h and other necessary files in the /src folder of your application.

In the CMakeLists.txt file, specify the *.c files to be compiled.

e.g., target_sources(app PRIVATE src/*.c)

Replace the bmi270 driver located in \zephyr\drivers\sensor\bmi270 with the modified driver available in the bmi270 folder of this repository.
Replace the files: "bosch,bmi270-i2c.yaml", "bosch,bmi270-spi.yaml" located in \zephyr\dts\bindings\sensor with the modified files available in the bindings folder of this repository.

## Sensor Initialization 

 - Pin configuration (.overlay file)
 
 Create a file with the .overlay extension in the main project folder and add the bmi270 device to the devicetree by connecting it to an SPI/I2C device.
 
In the example, it is connected to arduino_spi.
 - Configuration of compilation modules in the project (.conf file)
 
 In the main project folder, add the following modules to the prj.conf file:

 `CONFIG_GPIO=y`

`CONFIG_SPI=y / CONFIG_I2C=y`

`CONFIG_SENSOR=y`

 - Sensor initialization in the .c file
 
 After completing the previous configuration, we can initialize the sensor in the source code using the function:
 
bmi270_init() 

## Sensor Configuration - Accelerometer, Gyroscope

 - Basic configuration of Output Data Rate (ODR), full scale, oversampling (filter configuration)

 - FIFO configuration

 - Interrupt configuration (anymotion, nomotion, fifo_full)

 - Power mode configuration - advanced_power_save

## Examples

Usage examples (bmi270_interrupt, bmi270_fifo) can be found in the examples folder

# BMI270 API

This README provides an overview of the functions available in the BMI270 API. The BMI270 is a sensor device that supports various configurations and operations related to acceleration and gyroscope measurements.

## Initialization

### `bmi270_init(const struct device *const bmi270_dev)`

Initializes the BMI270 device.

## Basic Configuration

### Accelerometer Configuration

#### `bmi270_basic_config_acc(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)`

Configures the basic settings for the accelerometer on the BMI270 device.

- `full_scale`: The full-scale range for the accelerometer.
- `sampling_freq`: The sampling frequency for the accelerometer.
- `oversampling`: The oversampling setting for the accelerometer.

### Gyroscope Configuration

#### `bmi270_basic_config_gyr(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)`

Configures the basic settings for the gyroscope on the BMI270 device.

- `full_scale`: The full-scale range for the gyroscope.
- `sampling_freq`: The sampling frequency for the gyroscope.
- `oversampling`: The oversampling setting for the gyroscope.

## Interrupt Configuration

### Any Motion Interrupt

#### `bmi270_config_anymotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)`

Configures the any motion interrupt on the BMI270 device.

- `threshold`: The threshold value for triggering the any motion interrupt.
- `duration`: The duration of motion required to trigger the any motion interrupt.

### No Motion Interrupt

#### `bmi270_config_nomotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)`

Configures the no motion interrupt on the BMI270 device.

- `threshold`: The threshold value for triggering the no motion interrupt.
- `duration`: The duration of no motion required to trigger the no motion interrupt.

### Set Any Motion Trigger

#### `bmi270_set_anymotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)`

Sets the trigger for the any motion interrupt on the BMI270 device.

- `trigger_handler`: The handler function to be called when the any motion interrupt is triggered.
- `trig`: The trigger structure containing additional information for the any motion interrupt.

### Set No Motion Trigger

#### `bmi270_set_nomotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)`

Sets the trigger for the no motion interrupt on the BMI270 device.

- `trigger_handler`: The handler function to be called when the no motion interrupt is triggered.
- `trig`: The trigger structure containing additional information for the no motion interrupt.

## Other Configurations

### `bmi270_config_ffull_interrupt(const struct device *const bmi270_dev, uint16_t ffifo_enable)`

Configures the full FIFO interrupt on the BMI270 device.

- `ffifo_enable`: Enable or disable the full FIFO interrupt.

### 

Similar code found with 1 license type

# BMI270 API

This README provides an overview of the functions available in the BMI270 API. The BMI270 is a sensor device that supports various configurations and operations related to acceleration and gyroscope measurements.

## Initialization

### `bmi270_init(const struct device *const bmi270_dev)`

Initializes the BMI270 device.

## Basic Configuration

### Accelerometer Configuration

#### `bmi270_basic_config_acc(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)`

Configures the basic settings for the accelerometer on the BMI270 device.

- `full_scale`: The full-scale range for the accelerometer.
- `sampling_freq`: The sampling frequency for the accelerometer.
- `oversampling`: The oversampling setting for the accelerometer.

### Gyroscope Configuration

#### `bmi270_basic_config_gyr(const struct device *const bmi270_dev, uint16_t full_scale, uint16_t sampling_freq, uint16_t oversampling)`

Configures the basic settings for the gyroscope on the BMI270 device.

- `full_scale`: The full-scale range for the gyroscope.
- `sampling_freq`: The sampling frequency for the gyroscope.
- `oversampling`: The oversampling setting for the gyroscope.

## Interrupt Configuration

### Any Motion Interrupt

#### `bmi270_config_anymotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)`

Configures the any motion interrupt on the BMI270 device.

- `threshold`: The threshold value for triggering the any motion interrupt.
- `duration`: The duration of motion required to trigger the any motion interrupt.

### No Motion Interrupt

#### `bmi270_config_nomotion_interrupt(const struct device *const bmi270_dev, uint16_t threshold, uint16_t duration)`

Configures the no motion interrupt on the BMI270 device.

- `threshold`: The threshold value for triggering the no motion interrupt.
- `duration`: The duration of no motion required to trigger the no motion interrupt.

### Set Any Motion Trigger

#### `bmi270_set_anymotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)`

Sets the trigger for the any motion interrupt on the BMI270 device.

- `trigger_handler`: The handler function to be called when the any motion interrupt is triggered.
- `trig`: The trigger structure containing additional information for the any motion interrupt.

### Set No Motion Trigger

#### `bmi270_set_nomotion_trigger(const struct device *const bmi270_dev, sensor_trigger_handler_t trigger_handler, struct sensor_trigger *trig)`

Sets the trigger for the no motion interrupt on the BMI270 device.

- `trigger_handler`: The handler function to be called when the no motion interrupt is triggered.
- `trig`: The trigger structure containing additional information for the no motion interrupt.

## Other Configurations

### `bmi270_config_ffull_interrupt(const struct device *const bmi270_dev, uint16_t ffifo_enable)`

Configures the full FIFO interrupt on the BMI270 device.

- `ffifo_enable`: Enable or disable the full FIFO interrupt.

### `bmi270_config_adv_power_save_mode(const struct device *const bmi270_dev, uint8_t enable)`

Configures the advanced power save mode on the BMI270 device.

- `enable`: Enable or disable the advanced power save mode.

### `bmi270_config_fifo(const struct device *const bmi270_dev, uint8_t fifo_header_en, uint8_t fifo_aux_en, uint8_t fifo_acc_en, uint8_t fifo_gyr_en)`

Configures the FIFO settings on the BMI270 device.

- `fifo_header_en`: Enable or disable FIFO header.
- `fifo_aux_en`: Enable or disable FIFO auxiliary data.
- `fifo_acc_en`: Enable or disable FIFO accelerometer data.
- `fifo_gyr_en`: Enable or disable FIFO gyroscope data.

## GPIO Initialization and Configuration

### `gpio_init_manual(const struct gpio_dt_spec *spec)`

Initializes a GPIO pin manually.

- `spec`: GPIO specification structure.

### `gpio_callback_config_manual(struct gpio_callback *callback_data, gpio_callback_handler_t handler, const struct gpio_dt_spec *spec)`

Configures a callback for a GPIO pin manually.

- `callback_data`: GPIO callback data structure.
- `handler`: GPIO callback handler function.
- `spec`: GPIO specification structure.

## FIFO Operations

### `bmi270_get_fifo_length(uint16_t *fifo_length, const struct device *dev)`

Gets the length of the FIFO data available on the BMI270 device.

- `fifo_length`: Pointer to store the FIFO data length.
- `dev`: BMI270 device structure.

### `bmi270_read_fifo_data(struct bmi2_fifo_frame *fifo, const struct device *dev, uint64_t sens_en_stat, bool spi, bool advanced_power_save)`

Reads FIFO data from the BMI270 device.

- `fifo`: FIFO frame structure to store the data.
- `dev`: BMI270 device structure.
- `sens_en_stat`: Sensor enable status.
- `spi`: Use SPI interface.
- `advanced_power_save`: Use advanced power save mode.

### `bmi270_extract_accel(struct bmi2_sens_axes_data *accel_data, uint16_t *accel_length, struct bmi2_fifo_frame *fifo, uint64_t sens_en_stat)`

Extracts accelerometer data from the FIFO frame on the BMI270 device.

- `accel_data`: Accelerometer data structure.
- `accel_length`: Pointer to store the accelerometer data length.
- `fifo`: FIFO frame structure.
- `sens_en_stat`: Sensor enable status.

### `bmi270_extract_gyro(struct bmi2_sens_axes_data *gyro_data, uint16_t *gyro_length, struct bmi2_fifo_frame *fifo, const struct bmi2_dev *dev)`

Extracts gyroscope data from the FIFO frame on the BMI270 device.

- `gyro_data`: Gyroscope data structure.
- `gyro_length`: Pointer to store the gyroscope data length.
- `fifo`: FIFO frame structure.
- `dev`: BMI270 device structure.

## Sensor Value Conversion

### `channel_accel_convert_custom(struct sensor_value *val, int64_t raw_val, uint8_t range)`

Converts the raw accelerometer value to a custom sensor value with a specified range.

- `val`: Pointer to store the converted sensor value.
- `raw_val`: Raw accelerometer value.
- `range`: Range of the accelerometer.


