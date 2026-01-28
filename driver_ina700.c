/**
 * Copyright (c) 2024
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_ina700.c
 * @brief     driver ina700 source file
 * @version   1.0.0
 * @author    Generated
 * @date      2024-03-07
 */

#include "driver_ina700.h" 
#include <math.h>

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "Texas Instruments INA700"        /**< chip name */
#define MANUFACTURER_NAME         "Texas Instruments"               /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.7f                              /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        5.5f                              /**< chip max supply voltage */
#define MAX_CURRENT               15.0f                             /**< chip max current */
#define TEMPERATURE_MIN           -40.0f                            /**< chip min operating temperature */
#define TEMPERATURE_MAX           105.0f                            /**< chip max operating temperature */
#define DRIVER_VERSION            1000                              /**< driver version */

/**
 * @brief chip register definition
 */
#define INA700_REG_CONFIG          0x00        /**< configuration register */
#define INA700_REG_ADC_CONFIG      0x01        /**< ADC configuration register */
#define INA700_REG_VBUS            0x05        /**< bus voltage register */
#define INA700_REG_DIETEMP         0x06        /**< temperature register */
#define INA700_REG_CURRENT         0x07        /**< current register */
#define INA700_REG_POWER           0x08        /**< power register */
#define INA700_REG_ENERGY          0x09        /**< energy register */
#define INA700_REG_CHARGE          0x0A        /**< charge register */
#define INA700_REG_ALERT_DIAG      0x0B        /**< diagnostic flags and alert register */
#define INA700_REG_MANUFACTURER_ID 0x3E        /**< manufacturer ID register */

/**
 * @brief conversion factors 
 */
#define INA700_CURRENT_LSB     0.000480f       /**< 480μA/LSB */
#define INA700_VOLTAGE_LSB     0.003125f       /**< 3.125mV/LSB */
#define INA700_POWER_LSB       0.000096f       /**< 96μW/LSB */
#define INA700_TEMP_LSB        0.125f          /**< 125m°C/LSB */

/**
 * @brief      iic interface read bytes
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[in]  reg iic register address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_ina700_iic_read(ina700_handle_t *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (handle->iic_read(handle->iic_addr, reg, data, len) != 0)
    {
        return 1;                                                                /* return error */
    }
    else
    {
        return 0;                                                                /* success return 0 */
    }
}

/**
 * @brief     iic interface write bytes
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] reg iic register address
 * @param[in] data written data
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
static uint8_t a_ina700_iic_write(ina700_handle_t *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (handle->iic_write(handle->iic_addr, reg, data, len) != 0)
    {
        return 1;                                                                /* return error */
    }
    else
    {
        return 0;                                                                /* success return 0 */
    }
}

/**
 * @brief     set the iic address pin
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] addr_pin address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t ina700_set_addr_pin(ina700_handle_t *handle, ina700_address_t addr_pin)
{
    if (handle == NULL)                          /* check handle */
    {
        return 2;                                /* return error */
    }
    
    handle->iic_addr = (uint8_t)addr_pin;        /* set pin */
    
    return 0;                                    /* success return 0 */
}

/**
 * @brief      get the iic address pin
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *addr_pin pointer to an address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t ina700_get_addr_pin(ina700_handle_t *handle, ina700_address_t *addr_pin)
{
    if (handle == NULL)                                      /* check handle */
    {
        return 2;                                            /* return error */
    }
    
    *addr_pin = (ina700_address_t)(handle->iic_addr);        /* get pin */
    
    return 0;                                                /* success return 0 */
}

/**
 * @brief     soft reset the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 soft reset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t ina700_soft_reset(ina700_handle_t *handle)
{
    uint8_t res;
    uint8_t data[2] = {0x80, 0x00};  /* Set bit 15 for reset (PDF第23页表7-5) */
   
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    res = a_ina700_iic_write(handle, INA700_REG_CONFIG, data, 2);
    if (res != 0)                                                              /* check result */
    {
        handle->debug_print("ina700: write config register failed.\n");
        return 1;
    }
    
    handle->delay_ms(10);  /* Wait for reset to complete */
    
    return 0;
}

/**
 * @brief     set the mode
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] mode chip mode
 * @return    status code
 *            - 0 success
 *            - 1 set mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t ina700_set_mode(ina700_handle_t *handle, ina700_mode_t mode)
{
    uint8_t res;
    uint8_t data[2];
    uint8_t read_buf[2];
   
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    /* Read current ADC_CONFIG value */
    res = a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, read_buf, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read ADC_CONFIG register failed.\n");
        return 1;
    }
    
    /* Update mode bits (bits 15-12) */
    uint16_t adc_config = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    adc_config &= ~(0xF000);  /* Clear mode bits */
    adc_config |= ((uint16_t)mode << 12);
    
    data[0] = (uint8_t)((adc_config >> 8) & 0xFF);
    data[1] = (uint8_t)(adc_config & 0xFF);
    
    res = a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: write ADC_CONFIG register failed.\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief      get the mode
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *mode pointer to a chip mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t ina700_get_mode(ina700_handle_t *handle, ina700_mode_t *mode)
{
    uint8_t res;
    uint8_t data[2];
   
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    res = a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read ADC_CONFIG register failed.\n");
        return 1;
    }
    
    uint16_t adc_config = ((uint16_t)data[0] << 8) | data[1];
    *mode = (ina700_mode_t)((adc_config >> 12) & 0xF);
    
    return 0;
}

/**
 * @brief     set the conversion time
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] bus_time bus voltage conversion time
 * @param[in] shunt_time shunt voltage conversion time
 * @param[in] temp_time temperature conversion time
 * @return    status code
 *            - 0 success
 *            - 1 set conversion time failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t ina700_set_conversion_time(ina700_handle_t *handle, 
                                   ina700_conv_time_t bus_time,
                                   ina700_conv_time_t shunt_time,
                                   ina700_conv_time_t temp_time)
{
    uint8_t res;
    uint8_t data[2];
    uint8_t read_buf[2];
   
    if (handle == NULL)
    {
        return 2;
    }
    if (handle->inited != 1)
    {
        return 3;
    }
    
    /* Read current ADC_CONFIG value */
    res = a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, read_buf, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read ADC_CONFIG register failed.\n");
        return 1;
    }
    
    /* Update conversion time bits */
    uint16_t adc_config = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    
    /* Clear conversion time bits */
    adc_config &= ~(0x0E00);  /* Clear VBUSCT bits (11-9) */
    adc_config &= ~(0x01C0);  /* Clear VSENTCT bits (8-6) */
    adc_config &= ~(0x0038);  /* Clear TCT bits (5-3) */
    
    /* Set new conversion times */
    adc_config |= ((uint16_t)bus_time << 9);      /* Bits 11-9 */
    adc_config |= ((uint16_t)shunt_time << 6);    /* Bits 8-6 */
    adc_config |= ((uint16_t)temp_time << 3);     /* Bits 5-3 */
    
    data[0] = (uint8_t)((adc_config >> 8) & 0xFF);
    data[1] = (uint8_t)(adc_config & 0xFF);
    
    res = a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: write ADC_CONFIG register failed.\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief     set the averaging
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] avg averaging samples
 * @return    status code
 *            - 0 success
 *            - 1 set averaging failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t ina700_set_averaging(ina700_handle_t *handle, ina700_avg_t avg)
{
    uint8_t res;
    uint8_t data[2];
    uint8_t read_buf[2];
   
    if (handle == NULL)
    {
        return 2;
    }
    if (handle->inited != 1)
    {
        return 3;
    }
    
    /* Read current ADC_CONFIG value */
    res = a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, read_buf, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read ADC_CONFIG register failed.\n");
        return 1;
    }
    
    /* Update averaging bits (bits 2-0) */
    uint16_t adc_config = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    adc_config &= ~(0x0007);  /* Clear AVG bits */
    adc_config |= (uint16_t)avg;
    
    data[0] = (uint8_t)((adc_config >> 8) & 0xFF);
    data[1] = (uint8_t)(adc_config & 0xFF);
    
    res = a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: write ADC_CONFIG register failed.\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief      read the bus voltage
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *voltage pointer to voltage value in volts
 * @return     status code
 *             - 0 success
 *             - 1 read bus voltage failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t ina700_read_bus_voltage(ina700_handle_t *handle, float *voltage)
{
    uint8_t res;
    uint8_t data[2];
    
    if (handle == NULL)                                                         /* check handle */
    {
        return 2;                                                               /* return error */
    }
    if (handle->inited != 1)                                                    /* check handle initialization */
    {
        return 3;                                                               /* return error */
    }
    
    res = a_ina700_iic_read(handle, INA700_REG_VBUS, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read bus voltage register failed.\n");
        return 1;
    }
    
    /* Convert to voltage (16-bit signed, but always positive) */
    int16_t raw = ((int16_t)data[0] << 8) | data[1];
    *voltage = raw * INA700_VOLTAGE_LSB;
    
    return 0;
}

/**
 * @brief      read the current
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *current pointer to current value in amperes
 * @return     status code
 *             - 0 success
 *             - 1 read current failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t ina700_read_current(ina700_handle_t *handle, float *current)
{
    uint8_t res;
    uint8_t data[2];
    
    if (handle == NULL)                                                         /* check handle */
    {
        return 2;                                                               /* return error */
    }
    if (handle->inited != 1)                                                    /* check handle initialization */
    {
        return 3;                                                               /* return error */
    }
    
    res = a_ina700_iic_read(handle, INA700_REG_CURRENT, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read current register failed.\n");
        return 1;
    }
    
    /* Convert to current (16-bit signed, 2's complement) */
    int16_t raw = ((int16_t)data[0] << 8) | data[1];
    *current = raw * INA700_CURRENT_LSB;
    
    return 0;
}

/**
 * @brief      read the power
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *power pointer to power value in watts
 * @return     status code
 *             - 0 success
 *             - 1 read power failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t ina700_read_power(ina700_handle_t *handle, float *power)
{
    uint8_t res;
    uint8_t data[3];  /* 24-bit register */
    
    if (handle == NULL)                                                         /* check handle */
    {
        return 2;                                                               /* return error */
    }
    if (handle->inited != 1)                                                    /* check handle initialization */
    {
        return 3;                                                               /* return error */
    }
    
    /* Read 3 bytes for 24-bit power register */
    res = a_ina700_iic_read(handle, INA700_REG_POWER, data, 3);
    if (res != 0)
    {
        handle->debug_print("ina700: read power register failed.\n");
        return 1;
    }
    
    /* Convert to power (24-bit unsigned) */
    uint32_t raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *power = raw * INA700_POWER_LSB;
    
    return 0;
}

/**
 * @brief      read the temperature
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *temperature pointer to temperature value in degrees Celsius
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t ina700_read_temperature(ina700_handle_t *handle, float *temperature)
{
    uint8_t res;
    uint8_t data[2];
    
    if (handle == NULL)                                                         /* check handle */
    {
        return 2;                                                               /* return error */
    }
    if (handle->inited != 1)                                                    /* check handle initialization */
    {
        return 3;                                                               /* return error */
    }
    
    res = a_ina700_iic_read(handle, INA700_REG_DIETEMP, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: read temperature register failed.\n");
        return 1;
    }
    
    /* Convert to temperature (16-bit signed, 2's complement) */
    int16_t raw = ((int16_t)data[0] << 8) | data[1];
    *temperature = raw * INA700_TEMP_LSB;
    
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
 * @note      none
 */
uint8_t ina700_init(ina700_handle_t *handle)
{
    uint8_t res;
    uint8_t data[2];
    
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->debug_print == NULL)                                           /* check debug_print */
    {
        return 3;                                                              /* return error */
    }
    if (handle->iic_init == NULL)                                              /* check iic_init */
    {
        handle->debug_print("ina700: iic_init is null.\n");                    /* iic_init is null */
        
        return 3;                                                              /* return error */
    }
    if (handle->iic_deinit == NULL)                                            /* check iic_deinit */
    {
        handle->debug_print("ina700: iic_deinit is null.\n");                  /* iic_deinit is null */
        
        return 3;                                                              /* return error */
    }
    if (handle->iic_read == NULL)                                              /* check iic_read */
    {
        handle->debug_print("ina700: iic_read is null.\n");                    /* iic_read is null */
        
        return 3;                                                              /* return error */
    }
    if (handle->iic_write == NULL)                                             /* check iic_write */
    {
        handle->debug_print("ina700: iic_write is null.\n");                   /* iic_write is null */
        
        return 3;                                                              /* return error */
    }
    if (handle->delay_ms == NULL)                                              /* check delay_ms */
    {
        handle->debug_print("ina700: delay_ms is null.\n");                    /* delay_ms is null */
        
        return 3;                                                              /* return error */
    }
    
    if (handle->iic_init() != 0)                                               /* iic init */
    {
        handle->debug_print("ina700: iic init failed.\n");                     /* iic init failed */
        
        return 1;                                                              /* return error */
    }
    
    // /* Soft reset the chip */
    // res = ina700_soft_reset(handle);
    // if (res != 0)
    // {
    //     handle->debug_print("ina700: soft reset failed.\n");
    //     (void)handle->iic_deinit();
    //     return 4;
    // }
    
    /* Set default configuration for ADC_CONFIG */
    /* Default value from datasheet: 0xFB68 (PDF第24页表7-6) */
    data[0] = 0xFB;  /* MODE = 0xF (连续转换温度、电流和总线电压) */
    data[1] = 0x68;  /* Conversion times = 0x5 (1052μs), AVG = 0x0 (1 sample) */
    
    res = a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, data, 2);
    if (res != 0)
    {
        handle->debug_print("ina700: write ADC_CONFIG register failed.\n");
        (void)handle->iic_deinit();
        return 4;
    }
    
    handle->inited = 1;                                                        /* flag inited */
    
    return 0;                                                                  /* success return 0 */
}

/**
 * @brief     close the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t ina700_deinit(ina700_handle_t *handle)
{
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    /* Set shutdown mode */
    uint8_t data[2] = {0x00, 0x00};  /* MODE = 0x0 (shutdown) */
    uint8_t read_buf[2];
    
    /* Read current ADC_CONFIG */
    uint8_t res = a_ina700_iic_read(handle, INA700_REG_ADC_CONFIG, read_buf, 2);
    if (res == 0)
    {
        uint16_t adc_config = ((uint16_t)read_buf[0] << 8) | read_buf[1];
        adc_config &= ~(0xF000);  /* Clear mode bits */
        data[0] = (uint8_t)((adc_config >> 8) & 0xFF);
        data[1] = (uint8_t)(adc_config & 0xFF);
        a_ina700_iic_write(handle, INA700_REG_ADC_CONFIG, data, 2);
    }
    
    /* Deinit I2C */
    res = handle->iic_deinit();
    if (res != 0)
    {
        handle->debug_print("ina700: iic deinit failed.\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief      get chip's information
 * @param[out] *info pointer to an ina700 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t ina700_info(ina700_info_t *info)
{
    if (info == NULL)                                               /* check handle */
    {
        return 2;                                                   /* return error */
    }
    
    memset(info, 0, sizeof(ina700_info_t));                         /* initialize ina700 info structure */
    strncpy(info->chip_name, CHIP_NAME, 32);                        /* copy chip name */
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 32);        /* copy manufacturer name */
    strncpy(info->interface, "IIC", 8);                             /* copy interface name */
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;                /* set minimal supply voltage */
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;                /* set maximum supply voltage */
    info->max_current_ma = MAX_CURRENT;                             /* set maximum current */
    info->temperature_max = TEMPERATURE_MAX;                        /* set minimal temperature */
    info->temperature_min = TEMPERATURE_MIN;                        /* set maximum temperature */
    info->driver_version = DRIVER_VERSION;                          /* set driver version */
    
    return 0;                                                       /* success return 0 */
}