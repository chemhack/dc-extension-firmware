#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "driver_ina219.h"
#include "driver_ina700.h"
#include "flash_store.h"

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define PWR_PIN 4

#define INA_I2C_SDA_PIN 8
#define INA_I2C_SCL_PIN 9

typedef enum {
    INA_TYPE_NONE = 0,
    INA_TYPE_INA219,
    INA_TYPE_INA700
} ina_type_t;

static ina_type_t detected_ina = INA_TYPE_NONE;

#ifdef PICO_PROGRAM_NAME
#define JETKVM_NAME PICO_PROGRAM_NAME
#else
#define JETKVM_NAME "jetkvm-dc"
#endif

#ifdef PICO_PROGRAM_VERSION_STRING
#define JETKVM_VERSION PICO_PROGRAM_VERSION_STRING
#else
#define JETKVM_VERSION "unknown"
#endif

#ifdef PICO_PROGRAM_BUILD_DATE
#define JETKVM_BUILD_DATE PICO_PROGRAM_BUILD_DATE
#else
#define JETKVM_BUILD_DATE __DATE__
#endif

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

static char event_str[128];

void gpio_callback(uint gpio, uint32_t events)
{
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}

#define UART_BUF_SIZE 128
static char uart_buf[UART_BUF_SIZE];
static int uart_buf_pos = 0;

void set_power_pin(bool value){
    gpio_put(PWR_PIN, value);
    set_power_state(value);
}

void uart_send_version() {
    char version_str[128];
    snprintf(
        version_str,
        sizeof(version_str),
        "EXTVER;%s;%s\n",
        JETKVM_NAME,
        JETKVM_VERSION,
        JETKVM_BUILD_DATE
    );
    uart_puts(UART_ID, version_str);
}

void on_uart_line(const char *line)
{
    printf("UART LINE: %s\n", line);
    if (strcmp(line, "PWR_ON\n") == 0) {
        set_power_pin(1);
        printf("Power ON\n");
    }
    else if (strcmp(line, "PWR_OFF\n") == 0) {
        set_power_pin(0);
        printf("Power OFF\n");
    }
    else if (strcmp(line, "RESTORE_MODE_OFF\n") == 0) {
        set_restore_mode(RESTORE_MODE_OFF);
    }
    else if (strcmp(line, "RESTORE_MODE_ON\n") == 0) {
        set_restore_mode(RESTORE_MODE_ON);
    }
    else if (strcmp(line, "RESTORE_MODE_LAST_STATE\n") == 0) {
        set_restore_mode(RESTORE_MODE_LAST_STATE);
    } else if (strcmp(line, "VERSION\n") == 0) {
        uart_send_version();
    }
}

void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);

        if (uart_buf_pos < UART_BUF_SIZE - 1)
        {
            uart_buf[uart_buf_pos++] = ch;
        }

        if (ch == '\n' || uart_buf_pos >= UART_BUF_SIZE - 1)
        {
            uart_buf[uart_buf_pos] = '\0';
            on_uart_line(uart_buf);
            uart_buf_pos = 0;
        }
    }
}

static ina700_handle_t ina700;
static ina219_handle_t ina219;

uint8_t ina_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    printf("ina: i2c read %d bytes from reg 0x%02x addr %d\n", len, reg, addr);
    int ret = i2c_write_blocking(i2c0, addr, &reg, 1, true);
    if (ret < 0) {
        printf("ina: i2c write failed with error %d\n", ret);
        return 1;
    }
    ret = i2c_read_blocking(i2c0, addr, buf, len, false);
    if (ret < 0) {
        printf("ina: i2c read failed with error %d\n", ret);
        return 1;
    }
    return 0;
}

uint8_t ina_i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tmp_buf[len + 1];
    tmp_buf[0] = reg;
    memcpy(&tmp_buf[1], buf, len);
    int ret = i2c_write_blocking(i2c0, addr, tmp_buf, len + 1, false);
    if (ret < 0) {
        printf("ina: i2c write failed with error %d\n", ret);
        return 1;
    }
    return 0;
}

uint8_t ina_i2c_init() {
    i2c_init(i2c0, 100*1000);
    gpio_set_function(INA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INA_I2C_SDA_PIN);
    gpio_pull_up(INA_I2C_SCL_PIN);
    printf("INAXXX I2C initialized on pins SDA=%d, SCL=%d\n", 
           INA_I2C_SDA_PIN, INA_I2C_SCL_PIN);
    return 0;
}

uint8_t ina_i2c_deinit() {
    i2c_deinit(i2c0);
    return 0;
}

uint8_t power_init() {
    uint8_t res;
    int scan_round = 0;
    const int max_scan_rounds = 3;
    ina_type_t found_type = INA_TYPE_NONE;

    // Initialize I2C first
    ina_i2c_init();

    // Scan I2C addresses from 1 to 127, up to 3 rounds
    while (scan_round < max_scan_rounds && found_type == INA_TYPE_NONE) {
        scan_round++;
        printf("I2C scan round %d/%d\n", scan_round, max_scan_rounds);
        
        for (uint8_t addr = 1; addr < 128; addr++) {
            // Skip reserved addresses if needed, but for simplicity scan all
            // Try to read a register to check if device responds
            uint8_t dummy;
            if (ina_i2c_read(addr, 0x00, &dummy, 1) == 0) {
                printf("Found device at address 0x%02x\n", addr);                
                if(addr == INA219_ADDRESS_0) {
                    found_type = INA_TYPE_INA219;
                } else if (addr == INA700_ADDRESS_0) {
                    found_type = INA_TYPE_INA700;
                } else {
                    printf("Unknown device at address 0x%02x, ignoring.\n", addr);
                }
                break;
            }
            sleep_ms(10);
        }
        
        if (found_type != INA_TYPE_NONE) {
            break;
        }
        
        printf("No compatible ina found in round %d, waiting before next scan...\n", scan_round);
        sleep_ms(500); // Wait before next scan round
    }
    
    if (found_type == INA_TYPE_INA219)
    {
        DRIVER_INA219_LINK_INIT(&ina219, ina219_handle_t);
        DRIVER_INA219_LINK_IIC_INIT(&ina219, ina_i2c_init);
        DRIVER_INA219_LINK_IIC_DEINIT(&ina219, ina_i2c_deinit); 
        DRIVER_INA219_LINK_IIC_READ(&ina219, ina_i2c_read);
        DRIVER_INA219_LINK_IIC_WRITE(&ina219, ina_i2c_write);
        DRIVER_INA219_LINK_DELAY_MS(&ina219, sleep_ms);
        DRIVER_INA219_LINK_DEBUG_PRINT(&ina219, printf);

        ina219_set_addr_pin(&ina219, INA219_ADDRESS_0);
        
        
        res = ina219_init(&ina219);
        if (res != 0) {
            printf("ina219: init failed.\n");
            return res;
        }
        
        // Set resistance (assuming 0.01 ohm shunt resistor)
        res = ina219_set_resistance(&ina219, 0.01);
        if (res != 0) {
            printf("ina219: set resistance failed.\n");
            return res;
        }
        
        // Set bus voltage range
        res = ina219_set_bus_voltage_range(&ina219, INA219_BUS_VOLTAGE_RANGE_32V);
        if (res != 0) {
            printf("ina219: set bus voltage range failed.\n");
            return res;
        }
        
        // Set bus voltage ADC mode
        res = ina219_set_bus_voltage_adc_mode(&ina219, INA219_ADC_MODE_12_BIT_1_SAMPLES);
        if (res != 0) {
            printf("ina219: set bus voltage adc mode failed.\n");
            return res;
        }
        
        // Set shunt voltage ADC mode
        res = ina219_set_shunt_voltage_adc_mode(&ina219, INA219_ADC_MODE_12_BIT_1_SAMPLES);
        if (res != 0) {
            printf("ina219: set shunt voltage adc mode failed.\n");
            return res;
        }
        
        // Set shunt bus voltage continuous
        res = ina219_set_mode(&ina219, INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);
        if (res != 0) {
            printf("ina219: set mode failed.\n");
            return res;
        }
        
        // Set PGA
        res = ina219_set_pga(&ina219, INA219_PGA_160_MV);
        if (res != 0) {
            printf("ina219: set pga failed.\n");
            return res;
        }
        
        // Calculate calibration
        uint16_t calibration;
        res = ina219_calculate_calibration(&ina219, (uint16_t *)&calibration);
        if (res != 0) {
            printf("ina219: calculate calibration failed.\n");
            return res;
        }
        
        // Set calibration
        res = ina219_set_calibration(&ina219, calibration);
        if (res != 0) {
            printf("ina219: set calibration failed.\n");
            return res;
        }
    }
    else {
        // Default to INA700
        printf("No ina detected after %d rounds, defaulting to INA700\n", max_scan_rounds);
        found_type = INA_TYPE_INA700;       
        DRIVER_INA700_LINK_INIT(&ina700, ina700_handle_t);
        DRIVER_INA700_LINK_IIC_INIT(&ina700, ina_i2c_init);
        DRIVER_INA700_LINK_IIC_DEINIT(&ina700, ina_i2c_deinit); 
        DRIVER_INA700_LINK_IIC_READ(&ina700, ina_i2c_read);
        DRIVER_INA700_LINK_IIC_WRITE(&ina700, ina_i2c_write);
        DRIVER_INA700_LINK_DELAY_MS(&ina700, sleep_ms);
        DRIVER_INA700_LINK_DEBUG_PRINT(&ina700, printf);

        ina700_set_addr_pin(&ina700, INA700_ADDRESS_0); // Ensure address pin is set to GND for default address
        
        res = ina700_init(&ina700);
        if (res != 0) {
            printf("Default INA700 init failed.\n");
            return res;
        }
		
		uint8_t data[2] = {};
	    /* code */
	    ina_i2c_read(INA700_ADDRESS_0, 0x3E, data, 2);
	    sleep_ms(100);
	    printf("ina700: loop.  data[0]:0x%02x data[1]:0x%02x\n", data[0], data[1]);
	        
        // Configure INA700
        res = ina700_set_mode(&ina700, INA700_MODE_CONTINUOUS_ALL);
        if (res != 0) {
            printf("ina700: set mode failed.\n");
            return res;
        }
        
        res = ina700_set_conversion_time(&ina700, 
                                         INA700_CONV_TIME_1052US,
                                         INA700_CONV_TIME_1052US,
                                         INA700_CONV_TIME_1052US);
        if (res != 0) {
            printf("ina700: set conversion time failed.\n");
            return res;
        }
        
        res = ina700_set_averaging(&ina700, INA700_AVG_1);
        if (res != 0) {
            printf("ina700: set averaging failed.\n");
            return res;
        }
    }
    
    detected_ina = found_type;
    return 0;
}

uint8_t ina_basic_read(float *voltage, float *current, float *power)
{
    uint8_t res;
    
    if (detected_ina == INA_TYPE_INA700) {
        /* read bus voltage */
        res = ina700_read_bus_voltage(&ina700, voltage);
        if (res != 0)
        {
            return 1;
        }
        
        /* read current */
        res = ina700_read_current(&ina700, current);
        if (res != 0)
        {
            return 1;
        }
        
        /* read power */
        res = ina700_read_power(&ina700, power);
        if (res != 0)
        {
            return 1;
        }
    } else if (detected_ina == INA_TYPE_INA219) {
        uint16_t raw;
        
        /* read bus voltage */
        res = ina219_read_bus_voltage(&ina219, &raw, voltage);
        if (res != 0)
        {
            return 1;
        }
        
        /* read current */
        res = ina219_read_current(&ina219, &raw, current);
        if (res != 0)
        {
            return 1;
        }
        
        /* read power */
        res = ina219_read_power(&ina219, &raw, power);
        if (res != 0)
        {
            return 1;
        }
    } else {
        return 1; // No ina detected
    }
    
    return 0;
}

int main()
{
    sleep_ms(1000);

    stdio_init_all();

    if (watchdog_caused_reboot())
    {
        printf("Rebooted by Watchdog!\n");
    }

    // watchdog_enable(8388, true);

    printf("Initing!\n");

    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    gpio_init(PWR_PIN);
    gpio_set_dir(PWR_PIN, GPIO_OUT);
    
    flash_store_init();
    uint8_t restore_mode = get_restore_mode();
    if (restore_mode == RESTORE_MODE_LAST_STATE){
        uint8_t last_power_state = get_power_state();
        gpio_put(PWR_PIN, last_power_state);
    } else {
        gpio_put(PWR_PIN, restore_mode);
    }
    

    power_init();

    float voltage;
    float current;
    float power;

    while (true)
    {
        watchdog_update();

        printf("Uptime: %llu s\n", time_us_64() / 1000000);
        if (ina_basic_read(&voltage, &current, &power) == 0) {
            int power_state = gpio_get(PWR_PIN);
            uint8_t restore_mode = get_restore_mode();

            printf("Power state: %d\n", power_state);
            printf("Voltage: %.2f V\n", voltage);
            printf("Current: %.2f A\n", current);
            printf("Power: %.2f W\n", power);
            char uart_msg[128];
            snprintf(uart_msg, sizeof(uart_msg), "%d;%.2f;%.2f;%.2f;%d\n", power_state, voltage, current, power, restore_mode);
            uart_puts(UART_ID, uart_msg);
        } else {
            printf("Error reading ina\n");
            uart_puts(UART_ID, "Error reading ina\n");
        }

        sleep_ms(1000);
    }
}
