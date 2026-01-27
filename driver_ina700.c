/**
 * @file      driver_ina700.c
 * @brief     driver ina700 source file
 * @version   1.0.0
 *
 * INA700 is a 40V, 16-bit digital power monitor with integrated 2mΩ EZShunt
 *
 * Register Map:
 *   CONFIG:      0x00  - Configuration register
 *   ADC_CONFIG:  0x01  - ADC configuration
 *   SHUNT_CAL:   0x02  - Shunt calibration (not used with integrated shunt)
 *   VSHUNT:      0x04  - Shunt voltage result
 *   VBUS:        0x05  - Bus voltage result
 *   DIETEMP:     0x06  - Die temperature
 *   CURRENT:     0x07  - Current result
 *   POWER:       0x08  - Power result
 *   DIAG_ALERT:  0x0B  - Diagnostic and alert
 *   DEVICE_ID:   0x3F  - Device ID
 *
 * LSB Values (INA700 with integrated 2mΩ shunt):
 *   Bus Voltage:  3.125 mV/LSB
 *   Current:      480 µA/LSB (signed 16-bit)
 *   Power:        9.6 mW/LSB (power = current_lsb * 20)
 *   Temperature:  125 m°C/LSB (7.8125 m°C/LSB for 12-bit value)
 */

#include "driver_ina700.h"

/* Register addresses */
#define INA700_REG_CONFIG           0x00
#define INA700_REG_ADC_CONFIG       0x01
#define INA700_REG_SHUNT_CAL        0x02
#define INA700_REG_VSHUNT           0x04
#define INA700_REG_VBUS             0x05
#define INA700_REG_DIETEMP          0x06
#define INA700_REG_CURRENT          0x07
#define INA700_REG_POWER            0x08
#define INA700_REG_DIAG_ALERT       0x0B
#define INA700_REG_SOVL             0x0C
#define INA700_REG_SUVL             0x0D
#define INA700_REG_BOVL             0x0E
#define INA700_REG_BUVL             0x0F
#define INA700_REG_TEMP_LIMIT       0x10
#define INA700_REG_POWER_LIMIT      0x11
#define INA700_REG_DEVICE_ID        0x3F

/* CONFIG register bits */
#define INA700_CONFIG_RST           (1 << 15)

/* ADC_CONFIG register bits */
#define INA700_ADC_CONFIG_MODE_MASK     0x000F
#define INA700_ADC_CONFIG_VBUSCT_SHIFT  9
#define INA700_ADC_CONFIG_VBUSCT_MASK   (0x7 << INA700_ADC_CONFIG_VBUSCT_SHIFT)
#define INA700_ADC_CONFIG_VSHCT_SHIFT   6
#define INA700_ADC_CONFIG_VSHCT_MASK    (0x7 << INA700_ADC_CONFIG_VSHCT_SHIFT)
#define INA700_ADC_CONFIG_VTCT_SHIFT    3
#define INA700_ADC_CONFIG_VTCT_MASK     (0x7 << INA700_ADC_CONFIG_VTCT_SHIFT)
#define INA700_ADC_CONFIG_AVG_SHIFT     0
#define INA700_ADC_CONFIG_AVG_MASK      (0x7 << INA700_ADC_CONFIG_AVG_SHIFT)

/* LSB values */
#define INA700_BUS_VOLTAGE_LSB_UV   3125        /* 3.125 mV = 3125 µV */
#define INA700_CURRENT_LSB_UA       480         /* 480 µA */
#define INA700_POWER_FACTOR         20          /* Power LSB = Current LSB * 20 */
#define INA700_TEMP_LSB_MC          7812        /* 7.8125 m°C * 1000 = 7812.5 */

/* Chip information */
#define CHIP_NAME                   "Texas Instruments INA700"
#define MANUFACTURER_NAME           "Texas Instruments"
#define SUPPLY_VOLTAGE_MIN          2.7f
#define SUPPLY_VOLTAGE_MAX          5.5f
#define MAX_CURRENT                 0.5f        /* 500 µA typical */
#define TEMPERATURE_MIN             -40.0f
#define TEMPERATURE_MAX             105.0f
#define DRIVER_VERSION              1000        /* version 1.0.00 */

/**
 * @brief     read bytes from register
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] reg register address
 * @param[out] *buf pointer to data buffer
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 read failed
 */
static uint8_t a_ina700_iic_read(ina700_handle_t *handle, uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];

    if (handle->iic_read(handle->iic_addr, reg, buf, 2) != 0)
    {
        return 1;
    }

    /* INA700 returns big-endian data */
    *data = ((uint16_t)buf[0] << 8) | buf[1];

    return 0;
}

/**
 * @brief     write bytes to register
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] reg register address
 * @param[in] data data to write
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 */
static uint8_t a_ina700_iic_write(ina700_handle_t *handle, uint8_t reg, uint16_t data)
{
    uint8_t buf[2];

    /* INA700 expects big-endian data */
    buf[0] = (uint8_t)((data >> 8) & 0xFF);
    buf[1] = (uint8_t)(data & 0xFF);

    return handle->iic_write(handle->iic_addr, reg, buf, 2);
}

/**
 * @brief      get chip's information
 * @param[out] *info pointer to an ina700 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 */
uint8_t ina700_info(ina700_info_t *info)
{
    if (info == NULL)
    {
        return 2;
    }

    memset(info, 0, sizeof(ina700_info_t));
    strncpy(info->chip_name, CHIP_NAME, 32);
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 32);
    strncpy(info->interface, "IIC", 8);
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;
    info->max_current_ma = MAX_CURRENT;
    info->temperature_min = TEMPERATURE_MIN;
    info->temperature_max = TEMPERATURE_MAX;
    info->driver_version = DRIVER_VERSION;

    return 0;
}

/**
 * @brief     set the iic address pin
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] addr_pin address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 */
uint8_t ina700_set_addr_pin(ina700_handle_t *handle, ina700_address_t addr_pin)
{
    if (handle == NULL)
    {
        return 2;
    }

    handle->iic_addr = (uint8_t)addr_pin;

    return 0;
}

/**
 * @brief      get the iic address pin
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *addr_pin pointer to an address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 */
uint8_t ina700_get_addr_pin(ina700_handle_t *handle, ina700_address_t *addr_pin)
{
    if (handle == NULL)
    {
        return 2;
    }

    *addr_pin = (ina700_address_t)(handle->iic_addr);

    return 0;
}

/**
 * @brief     initialize the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 soft reset failed
 */
uint8_t ina700_init(ina700_handle_t *handle)
{
    uint16_t config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->iic_init == NULL || handle->iic_deinit == NULL ||
        handle->iic_read == NULL || handle->iic_write == NULL ||
        handle->delay_ms == NULL)
    {
        handle->debug_print("ina700: linked functions is NULL.\n");
        return 3;
    }

    /* Initialize I2C */
    if (handle->iic_init() != 0)
    {
        handle->debug_print("ina700: iic init failed.\n");
        return 1;
    }

    /* Perform soft reset */
    config = INA700_CONFIG_RST;
    if (a_ina700_iic_write(handle, INA700_REG_CONFIG, config) != 0)
    {
        handle->debug_print("ina700: soft reset failed.\n");
        return 4;
    }

    /* Wait for reset to complete */
    handle->delay_ms(1);

    handle->inited = 1;

    return 0;
}

/**
 * @brief     close the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 power down failed
 */
uint8_t ina700_deinit(ina700_handle_t *handle)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    /* Read current ADC_CONFIG */
    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 4;
    }

    /* Set to shutdown mode */
    adc_config = (adc_config & 0xFFF0) | INA700_MODE_SHUTDOWN;
    if (a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, adc_config) != 0)
    {
        handle->debug_print("ina700: power down failed.\n");
        return 4;
    }

    /* Deinit I2C */
    if (handle->iic_deinit() != 0)
    {
        handle->debug_print("ina700: iic deinit failed.\n");
        return 1;
    }

    handle->inited = 0;

    return 0;
}

/**
 * @brief      read the bus voltage
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *raw pointer to raw data buffer
 * @param[out] *mV pointer to converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read bus voltage failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_read_bus_voltage(ina700_handle_t *handle, int16_t *raw, float *mV)
{
    uint16_t data;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, INA700_REG_VBUS, &data) != 0)
    {
        handle->debug_print("ina700: read bus voltage failed.\n");
        return 1;
    }

    *raw = (int16_t)data;
    /* Convert to mV: raw * 3.125 mV/LSB */
    *mV = (float)(*raw) * 3.125f;

    return 0;
}

/**
 * @brief      read the current
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *raw pointer to raw data buffer
 * @param[out] *mA pointer to converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read current failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_read_current(ina700_handle_t *handle, int16_t *raw, float *mA)
{
    uint16_t data;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, INA700_REG_CURRENT, &data) != 0)
    {
        handle->debug_print("ina700: read current failed.\n");
        return 1;
    }

    *raw = (int16_t)data;
    /* Convert to mA: raw * 480 µA/LSB = raw * 0.48 mA/LSB */
    *mA = (float)(*raw) * 0.48f;

    return 0;
}

/**
 * @brief      read the power
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *raw pointer to raw data buffer
 * @param[out] *mW pointer to converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read power failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_read_power(ina700_handle_t *handle, uint32_t *raw, float *mW)
{
    uint16_t data;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, INA700_REG_POWER, &data) != 0)
    {
        handle->debug_print("ina700: read power failed.\n");
        return 1;
    }

    *raw = (uint32_t)data;
    /* Convert to mW: raw * (480 µA * 20) = raw * 9.6 mW/LSB */
    *mW = (float)(*raw) * 9.6f;

    return 0;
}

/**
 * @brief      read the die temperature
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *raw pointer to raw data buffer
 * @param[out] *deg_c pointer to converted data buffer in degrees Celsius
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_read_temperature(ina700_handle_t *handle, int16_t *raw, float *deg_c)
{
    uint16_t data;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, INA700_REG_DIETEMP, &data) != 0)
    {
        handle->debug_print("ina700: read temperature failed.\n");
        return 1;
    }

    /* Temperature is stored in bits [15:4], so shift right by 4 */
    *raw = (int16_t)(data >> 4);
    /* Convert to °C: raw * 125 m°C/LSB = raw * 0.125 °C/LSB */
    *deg_c = (float)(*raw) * 0.125f;

    return 0;
}

/**
 * @brief     soft reset the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 soft reset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_soft_reset(ina700_handle_t *handle)
{
    uint16_t config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    config = INA700_CONFIG_RST;
    if (a_ina700_iic_write(handle, INA700_REG_CONFIG, config) != 0)
    {
        handle->debug_print("ina700: soft reset failed.\n");
        return 1;
    }

    /* Wait for reset to complete */
    handle->delay_ms(1);

    return 0;
}

/**
 * @brief     set the operating mode
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] mode operating mode
 * @return    status code
 *            - 0 success
 *            - 1 set mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_set_mode(ina700_handle_t *handle, ina700_mode_t mode)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    /* Read current ADC_CONFIG */
    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 1;
    }

    /* Update mode bits [3:0] in high byte (bits [15:12] of full register) */
    adc_config = (adc_config & 0x0FFF) | ((uint16_t)mode << 12);

    if (a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, adc_config) != 0)
    {
        handle->debug_print("ina700: set mode failed.\n");
        return 1;
    }

    return 0;
}

/**
 * @brief      get the operating mode
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *mode pointer to a mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_get_mode(ina700_handle_t *handle, ina700_mode_t *mode)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 1;
    }

    *mode = (ina700_mode_t)((adc_config >> 12) & 0x0F);

    return 0;
}

/**
 * @brief     set the bus voltage conversion time
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] conv_time conversion time
 * @return    status code
 *            - 0 success
 *            - 1 set conversion time failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_set_bus_voltage_conv_time(ina700_handle_t *handle, ina700_conv_time_t conv_time)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    /* Read current ADC_CONFIG */
    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 1;
    }

    /* Update VBUSCT bits [11:9] */
    adc_config = (adc_config & ~(0x7 << 9)) | ((uint16_t)conv_time << 9);

    if (a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, adc_config) != 0)
    {
        handle->debug_print("ina700: set bus voltage conv time failed.\n");
        return 1;
    }

    return 0;
}

/**
 * @brief     set the shunt voltage conversion time
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] conv_time conversion time
 * @return    status code
 *            - 0 success
 *            - 1 set conversion time failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_set_shunt_voltage_conv_time(ina700_handle_t *handle, ina700_conv_time_t conv_time)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    /* Read current ADC_CONFIG */
    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 1;
    }

    /* Update VSHCT bits [8:6] */
    adc_config = (adc_config & ~(0x7 << 6)) | ((uint16_t)conv_time << 6);

    if (a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, adc_config) != 0)
    {
        handle->debug_print("ina700: set shunt voltage conv time failed.\n");
        return 1;
    }

    return 0;
}

/**
 * @brief     set the averaging mode
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] avg_mode averaging mode
 * @return    status code
 *            - 0 success
 *            - 1 set averaging mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_set_avg_mode(ina700_handle_t *handle, ina700_avg_mode_t avg_mode)
{
    uint16_t adc_config;

    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    /* Read current ADC_CONFIG */
    if (a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, &adc_config) != 0)
    {
        handle->debug_print("ina700: read adc config failed.\n");
        return 1;
    }

    /* Update AVG bits [2:0] */
    adc_config = (adc_config & ~0x7) | ((uint16_t)avg_mode & 0x7);

    if (a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, adc_config) != 0)
    {
        handle->debug_print("ina700: set avg mode failed.\n");
        return 1;
    }

    return 0;
}

/**
 * @brief     set the chip register
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] reg register address
 * @param[in] data written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_set_reg(ina700_handle_t *handle, uint8_t reg, uint16_t data)
{
    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_write(handle, reg, data) != 0)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief      get the chip register
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[in]  reg register address
 * @param[out] *data pointer to a data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 */
uint8_t ina700_get_reg(ina700_handle_t *handle, uint8_t reg, uint16_t *data)
{
    if (handle == NULL)
    {
        return 2;
    }

    if (handle->inited != 1)
    {
        return 3;
    }

    if (a_ina700_iic_read(handle, reg, data) != 0)
    {
        return 1;
    }

    return 0;
}
