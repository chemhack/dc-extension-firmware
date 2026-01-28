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
 * @file      driver_ina700.h
 * @brief     driver ina700 header file
 * @version   1.0.0
 * @author    Generated
 * @date      2024-03-07
 */

#ifndef DRIVER_INA700_H
#define DRIVER_INA700_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup ina700_driver ina700 driver function
 * @brief    ina700 driver modules
 * @{
 */

/**
 * @addtogroup ina700_basic_driver
 * @{
 */

/**
 * @brief ina700 address enumeration definition
 */
typedef enum
{
    INA700_ADDRESS_0 = 0x44,        /**< A0 = GND */
    INA700_ADDRESS_1 = 0x45,        /**< A0 = VS+ */
    INA700_ADDRESS_2 = 0x46,        /**< A0 = SDA */
    INA700_ADDRESS_3 = 0x47,        /**< A0 = SCL */
} ina700_address_t;

/**
 * @brief ina700 mode enumeration definition
 */
typedef enum
{
    INA700_MODE_SHUTDOWN = 0x0,                    /**< shutdown */
    INA700_MODE_TRIGGER_BUS_VOLTAGE = 0x1,        /**< triggered bus voltage, single-shot */
    INA700_MODE_TRIGGER_TEMP = 0x4,               /**< triggered temperature, single-shot */
    INA700_MODE_TRIGGER_TEMP_BUS = 0x5,           /**< triggered temperature and bus voltage, single-shot */
    INA700_MODE_TRIGGER_TEMP_CURRENT = 0x6,       /**< triggered temperature and current, single-shot */
    INA700_MODE_TRIGGER_ALL = 0x7,                /**< triggered temperature, current and bus voltage, single-shot */
    INA700_MODE_CONTINUOUS_BUS_VOLTAGE = 0x9,     /**< continuous bus voltage */
    INA700_MODE_CONTINUOUS_TEMP = 0xC,            /**< continuous temperature */
    INA700_MODE_CONTINUOUS_TEMP_BUS = 0xD,        /**< continuous bus voltage and temperature */
    INA700_MODE_CONTINUOUS_TEMP_CURRENT = 0xE,    /**< continuous temperature and current */
    INA700_MODE_CONTINUOUS_ALL = 0xF,             /**< continuous temperature, current and bus voltage */
} ina700_mode_t;

/**
 * @brief ina700 conversion time enumeration definition
 */
typedef enum
{
    INA700_CONV_TIME_50US   = 0x0,   /**< 50μs */
    INA700_CONV_TIME_84US   = 0x1,   /**< 84μs */
    INA700_CONV_TIME_150US  = 0x2,   /**< 150μs */
    INA700_CONV_TIME_280US  = 0x3,   /**< 280μs */
    INA700_CONV_TIME_540US  = 0x4,   /**< 540μs */
    INA700_CONV_TIME_1052US = 0x5,   /**< 1052μs (default) */
    INA700_CONV_TIME_2074US = 0x6,   /**< 2074μs */
    INA700_CONV_TIME_4120US = 0x7,   /**< 4120μs */
} ina700_conv_time_t;

/**
 * @brief ina700 averaging enumeration definition
 */
typedef enum
{
    INA700_AVG_1    = 0x0,   /**< 1 sample */
    INA700_AVG_4    = 0x1,   /**< 4 samples */
    INA700_AVG_16   = 0x2,   /**< 16 samples */
    INA700_AVG_64   = 0x3,   /**< 64 samples */
    INA700_AVG_128  = 0x4,   /**< 128 samples */
    INA700_AVG_256  = 0x5,   /**< 256 samples */
    INA700_AVG_512  = 0x6,   /**< 512 samples */
    INA700_AVG_1024 = 0x7,   /**< 1024 samples */
} ina700_avg_t;

/**
 * @brief ina700 handle structure definition
 */
typedef struct ina700_handle_s
{
    uint8_t iic_addr;                                                                   /**< iic device address */
    uint8_t (*iic_init)(void);                                                          /**< point to an iic_init function address */
    uint8_t (*iic_deinit)(void);                                                        /**< point to an iic_deinit function address */
    uint8_t (*iic_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);         /**< point to an iic_read function address */
    uint8_t (*iic_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);        /**< point to an iic_write function address */
    void (*delay_ms)(uint32_t ms);                                                      /**< point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);                                    /**< point to a debug_print function address */
    uint8_t inited;                                                                     /**< inited flag */
} ina700_handle_t;

/**
 * @brief ina700 information structure definition
 */
typedef struct ina700_info_s
{
    char chip_name[32];                /**< chip name */
    char manufacturer_name[32];        /**< manufacturer name */
    char interface[8];                 /**< chip interface name */
    float supply_voltage_min_v;        /**< chip min supply voltage */
    float supply_voltage_max_v;        /**< chip max supply voltage */
    float max_current_ma;              /**< chip max current */
    float temperature_min;             /**< chip min operating temperature */
    float temperature_max;             /**< chip max operating temperature */
    uint32_t driver_version;           /**< driver version */
} ina700_info_t;

/**
 * @}
 */

/**
 * @defgroup ina700_link_driver ina700 link driver function
 * @brief    ina700 link driver modules
 * @ingroup  ina700_driver
 * @{
 */

/**
 * @brief     initialize ina700_handle_t structure
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] STRUCTURE ina700_handle_t
 * @note      none
 */
#define DRIVER_INA700_LINK_INIT(HANDLE, STRUCTURE)   memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link iic_init function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_init function address
 * @note      none
 */
#define DRIVER_INA700_LINK_IIC_INIT(HANDLE, FUC)    (HANDLE)->iic_init = FUC

/**
 * @brief     link iic_deinit function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_deinit function address
 * @note      none
 */
#define DRIVER_INA700_LINK_IIC_DEINIT(HANDLE, FUC)  (HANDLE)->iic_deinit = FUC

/**
 * @brief     link iic_read function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_read function address
 * @note      none
 */
#define DRIVER_INA700_LINK_IIC_READ(HANDLE, FUC)    (HANDLE)->iic_read = FUC

/**
 * @brief     link iic_write function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_write function address
 * @note      none
 */
#define DRIVER_INA700_LINK_IIC_WRITE(HANDLE, FUC)   (HANDLE)->iic_write = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to a delay_ms function address
 * @note      none
 */
#define DRIVER_INA700_LINK_DELAY_MS(HANDLE, FUC)    (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to a debug_print function address
 * @note      none
 */
#define DRIVER_INA700_LINK_DEBUG_PRINT(HANDLE, FUC)   (HANDLE)->debug_print = FUC

/**
 * @}
 */

/**
 * @defgroup ina700_basic_driver ina700 basic driver function
 * @brief    ina700 basic driver modules
 * @ingroup  ina700_driver
 * @{
 */

/**
 * @brief      get chip's information
 * @param[out] *info pointer to an ina700 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t ina700_info(ina700_info_t *info);

/**
 * @brief     set the iic address
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] addr_pin address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t ina700_set_addr_pin(ina700_handle_t *handle, ina700_address_t addr_pin);

/**
 * @brief      get the iic address
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *addr_pin pointer to an address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t ina700_get_addr_pin(ina700_handle_t *handle, ina700_address_t *addr_pin);

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
uint8_t ina700_init(ina700_handle_t *handle);

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
uint8_t ina700_deinit(ina700_handle_t *handle);

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
uint8_t ina700_set_mode(ina700_handle_t *handle, ina700_mode_t mode);

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
uint8_t ina700_get_mode(ina700_handle_t *handle, ina700_mode_t *mode);

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
                                   ina700_conv_time_t temp_time);

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
uint8_t ina700_set_averaging(ina700_handle_t *handle, ina700_avg_t avg);

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
uint8_t ina700_read_bus_voltage(ina700_handle_t *handle, float *voltage);

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
uint8_t ina700_read_current(ina700_handle_t *handle, float *current);

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
uint8_t ina700_read_power(ina700_handle_t *handle, float *power);

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
uint8_t ina700_read_temperature(ina700_handle_t *handle, float *temperature);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif