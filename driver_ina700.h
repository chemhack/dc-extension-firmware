/**
 * @file      driver_ina700.h
 * @brief     driver ina700 header file
 * @version   1.0.0
 *
 * INA700 is a 40V, 16-bit digital power monitor with integrated 2mΩ EZShunt
 * The device measures bus voltage and calculates current, power, energy and charge.
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
 * @brief ina700 address enumeration definition
 *        Address is determined by A0 pin connection
 */
typedef enum
{
    INA700_ADDRESS_GND = (0x44 << 1),        /**< A0 = GND */
    INA700_ADDRESS_VS  = (0x45 << 1),        /**< A0 = VS */
    INA700_ADDRESS_SDA = (0x46 << 1),        /**< A0 = SDA */
    INA700_ADDRESS_SCL = (0x47 << 1),        /**< A0 = SCL */
} ina700_address_t;

/**
 * @brief ina700 adc conversion time enumeration definition
 */
typedef enum
{
    INA700_CONV_TIME_50_US   = 0x0,        /**< 50 us */
    INA700_CONV_TIME_84_US   = 0x1,        /**< 84 us */
    INA700_CONV_TIME_150_US  = 0x2,        /**< 150 us */
    INA700_CONV_TIME_280_US  = 0x3,        /**< 280 us */
    INA700_CONV_TIME_540_US  = 0x4,        /**< 540 us */
    INA700_CONV_TIME_1052_US = 0x5,        /**< 1052 us */
    INA700_CONV_TIME_2074_US = 0x6,        /**< 2074 us */
    INA700_CONV_TIME_4120_US = 0x7,        /**< 4120 us */
} ina700_conv_time_t;

/**
 * @brief ina700 averaging mode enumeration definition
 */
typedef enum
{
    INA700_AVG_1    = 0x0,        /**< 1 sample */
    INA700_AVG_4    = 0x1,        /**< 4 samples */
    INA700_AVG_16   = 0x2,        /**< 16 samples */
    INA700_AVG_64   = 0x3,        /**< 64 samples */
    INA700_AVG_128  = 0x4,        /**< 128 samples */
    INA700_AVG_256  = 0x5,        /**< 256 samples */
    INA700_AVG_512  = 0x6,        /**< 512 samples */
    INA700_AVG_1024 = 0x7,        /**< 1024 samples */
} ina700_avg_mode_t;

/**
 * @brief ina700 operating mode enumeration definition
 */
typedef enum
{
    INA700_MODE_SHUTDOWN              = 0x0,        /**< Shutdown */
    INA700_MODE_SHUNT_TRIGGERED       = 0x1,        /**< Shunt voltage triggered, single shot */
    INA700_MODE_BUS_TRIGGERED         = 0x2,        /**< Bus voltage triggered, single shot */
    INA700_MODE_SHUNT_BUS_TRIGGERED   = 0x3,        /**< Shunt and bus triggered, single shot */
    INA700_MODE_TEMP_TRIGGERED        = 0x4,        /**< Temperature triggered, single shot */
    INA700_MODE_TEMP_SHUNT_TRIGGERED  = 0x5,        /**< Temp and shunt triggered, single shot */
    INA700_MODE_TEMP_BUS_TRIGGERED    = 0x6,        /**< Temp and bus triggered, single shot */
    INA700_MODE_TEMP_SHUNT_BUS_TRIGGERED = 0x7,     /**< Temp, shunt and bus triggered, single shot */
    INA700_MODE_SHUTDOWN2             = 0x8,        /**< Shutdown */
    INA700_MODE_SHUNT_CONTINUOUS      = 0x9,        /**< Shunt voltage continuous */
    INA700_MODE_BUS_CONTINUOUS        = 0xA,        /**< Bus voltage continuous */
    INA700_MODE_SHUNT_BUS_CONTINUOUS  = 0xB,        /**< Shunt and bus voltage continuous */
    INA700_MODE_TEMP_CONTINUOUS       = 0xC,        /**< Temperature continuous */
    INA700_MODE_TEMP_SHUNT_CONTINUOUS = 0xD,        /**< Temp and shunt continuous */
    INA700_MODE_TEMP_BUS_CONTINUOUS   = 0xE,        /**< Temp and bus continuous */
    INA700_MODE_TEMP_SHUNT_BUS_CONTINUOUS = 0xF,    /**< Temp, shunt and bus continuous */
} ina700_mode_t;

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
 * @brief     initialize ina700_handle_t structure
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] STRUCTURE ina700_handle_t
 */
#define DRIVER_INA700_LINK_INIT(HANDLE, STRUCTURE)   memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link iic_init function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_init function address
 */
#define DRIVER_INA700_LINK_IIC_INIT(HANDLE, FUC)    (HANDLE)->iic_init = FUC

/**
 * @brief     link iic_deinit function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_deinit function address
 */
#define DRIVER_INA700_LINK_IIC_DEINIT(HANDLE, FUC)  (HANDLE)->iic_deinit = FUC

/**
 * @brief     link iic_read function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_read function address
 */
#define DRIVER_INA700_LINK_IIC_READ(HANDLE, FUC)    (HANDLE)->iic_read = FUC

/**
 * @brief     link iic_write function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to an iic_write function address
 */
#define DRIVER_INA700_LINK_IIC_WRITE(HANDLE, FUC)   (HANDLE)->iic_write = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to a delay_ms function address
 */
#define DRIVER_INA700_LINK_DELAY_MS(HANDLE, FUC)    (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE pointer to an ina700 handle structure
 * @param[in] FUC pointer to a debug_print function address
 */
#define DRIVER_INA700_LINK_DEBUG_PRINT(HANDLE, FUC) (HANDLE)->debug_print = FUC

/**
 * @brief      get chip's information
 * @param[out] *info pointer to an ina700 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 */
uint8_t ina700_info(ina700_info_t *info);

/**
 * @brief     set the iic address pin
 * @param[in] *handle pointer to an ina700 handle structure
 * @param[in] addr_pin address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 */
uint8_t ina700_set_addr_pin(ina700_handle_t *handle, ina700_address_t addr_pin);

/**
 * @brief      get the iic address pin
 * @param[in]  *handle pointer to an ina700 handle structure
 * @param[out] *addr_pin pointer to an address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
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
 *            - 4 power down failed
 */
uint8_t ina700_deinit(ina700_handle_t *handle);

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
uint8_t ina700_read_bus_voltage(ina700_handle_t *handle, int16_t *raw, float *mV);

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
uint8_t ina700_read_current(ina700_handle_t *handle, int16_t *raw, float *mA);

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
uint8_t ina700_read_power(ina700_handle_t *handle, uint32_t *raw, float *mW);

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
uint8_t ina700_read_temperature(ina700_handle_t *handle, int16_t *raw, float *deg_c);

/**
 * @brief     soft reset the chip
 * @param[in] *handle pointer to an ina700 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 soft reset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 */
uint8_t ina700_soft_reset(ina700_handle_t *handle);

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
uint8_t ina700_set_mode(ina700_handle_t *handle, ina700_mode_t mode);

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
uint8_t ina700_get_mode(ina700_handle_t *handle, ina700_mode_t *mode);

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
uint8_t ina700_set_bus_voltage_conv_time(ina700_handle_t *handle, ina700_conv_time_t conv_time);

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
uint8_t ina700_set_shunt_voltage_conv_time(ina700_handle_t *handle, ina700_conv_time_t conv_time);

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
uint8_t ina700_set_avg_mode(ina700_handle_t *handle, ina700_avg_mode_t avg_mode);

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
uint8_t ina700_set_reg(ina700_handle_t *handle, uint8_t reg, uint16_t data);

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
uint8_t ina700_get_reg(ina700_handle_t *handle, uint8_t reg, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif
