#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"//cant seem to find #include "esp_adc_cal.h"
#include "driver/timer.h"
#include "mcp320x.h"

#define TIMER_DIVIDER         (80)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define DEFAULT_VREF    1100        

#define TIMERG1    1
#define HW_TIMER_1    1
        
#define SAMPLE_INTERNAL_ADC_TYPE_ID  0x01
#define SAMPLE_INTERNAL_GPIO_TYPE_ID 0x01

/*
#define GPIO_CS GPIO_NUM_15
#define GPIO_SCLK GPIO_NUM_14
#define GPIO_MISO GPIO_NUM_12
#define GPIO_MOSI GPIO_NUM_13

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        //.flags = SPICOMMON_BUSFLAG_MASTER
        };

    mcp320x_config_t mcp320x_cfg = {
        .host = SPI2_HOST,
        .device_model = MCP3204_MODEL,
        .clock_speed_hz = 1 * 1000 * 1000, // 1 Mhz.
        .reference_voltage = 5000,         // 5V
        .cs_io_num = GPIO_CS};

    mcp320x_handle_t mcp320x_handle;
*/


static esp_adc_cal_characteristics_t adc_chars;

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

/* ADC Definitions */
uint16_t vt_adc_id_sensor_1 = SAMPLE_INTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_2 = SAMPLE_INTERNAL_ADC_TYPE_ID;

adc_unit_t vt_adc_controller_sensor_1 = ADC_UNIT_1;
adc_unit_t vt_adc_controller_sensor_2 = ADC_UNIT_1;

uint32_t vt_adc_channel_sensor_1 = ADC1_CHANNEL_4;
uint32_t vt_adc_channel_sensor_2 = ADC1_CHANNEL_5;

/* GPIO Definitions */
//uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
//uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

uint16_t* vt_gpio_port_sensor_1;
uint16_t* vt_gpio_port_sensor_2;

uint16_t vt_gpio_pin_sensor_1 = GPIO_NUM_18;
uint16_t vt_gpio_pin_sensor_2 = GPIO_NUM_19;

gpio_num_t ADC_GPIO_NUM ;

static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

uint16_t vt_adc_single_read_init(
    uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{
    printf("TIMER_BASE_CLK - %d",TIMER_BASE_CLK);
    adc_unit_t unit = *((adc_unit_t*)adc_controller);
    adc_channel_t channel = *((adc_channel_t*)adc_channel);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc_gpio_init(ADC_UNIT_1,channel);
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
        adc1_pad_get_io_num(channel,&ADC_GPIO_NUM);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
    print_char_val_type(val_type);

        *adc_resolution = 12;
        *adc_ref_volt   = 3.6f;
    //ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));
    //ESP_ERROR_CHECK(spi_bus_add_device(mcp320x_cfg.host,));
    //ESP_ERROR_CHECK(spi_bus_initialize(mcp320x_cfg.host, &bus_cfg, 0));
    //ESP_ERROR_CHECK(mcp320x_initialize(&mcp320x_cfg, &mcp320x_handle));


    return 0;

}

uint16_t vt_adc_single_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
    int adc_raw = 0;
    //int adc_raw_abs = 0;

    adc_unit_t unit = *((adc_unit_t*)adc_controller);
    adc_channel_t channel = *((adc_channel_t*)adc_channel);
    
    if (unit == ADC_UNIT_1)
    {
        adc_raw=adc1_get_raw(channel);
    }
    else
    {
        adc2_get_raw(channel, width, &adc_raw);
    }
    //adc_raw_abs=abs(adc_raw-4095);

    return (uint16_t)adc_raw;
}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_pull_mode(ADC_GPIO_NUM,GPIO_FLOATING);

    gpio_set_level(*((uint16_t*)gpio_pin), 1);

        //unsigned short raw;
        //unsigned short voltage;

        //ESP_ERROR_CHECK(mcp320x_read_raw(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &raw));
        //ESP_ERROR_CHECK(mcp320x_read_voltage(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &voltage));

        //ESP_LOGI("mcp320x", "Raw: %d", raw);
        //ESP_LOGI("mcp320x", "Voltage: %d mV", voltage);
        //printf("Raw: %d \n", raw);
        //printf("Voltage: %d mV \n", voltage);

    return 0;

}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_pull_mode(ADC_GPIO_NUM,GPIO_PULLDOWN_ONLY);

    gpio_set_level(*((uint16_t*)gpio_pin), 0);
    return 0;

}

uint16_t vt_tick_init(uint16_t* max_value, uint16_t* resolution_usec)
{
    uint16_t default_max_tick        = 65535;
    uint16_t default_tick_resolution = 1;
    if (*max_value)
    {
        default_max_tick = *max_value;
    }
    else
    {
        *max_value = default_max_tick;
    }
    if (*resolution_usec)
    {
        default_tick_resolution = *resolution_usec;
    }
    else
    {
        *resolution_usec = default_tick_resolution;
    }

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMERG1, HW_TIMER_1, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMERG1, HW_TIMER_1, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMERG1, HW_TIMER_1, 65535);

    timer_start(TIMERG1, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick_deinit()
{
    timer_pause(TIMERG1, HW_TIMER_1);
    timer_deinit(TIMERG1, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick()
{
    uint64_t task_counter_value;
    timer_get_counter_value(TIMERG1, HW_TIMER_1, &task_counter_value);

    return task_counter_value;
}


void vt_interrupt_enable()
{
    esp_intr_noniram_enable();
}

void vt_interrupt_disable()
{
    esp_intr_noniram_disable();
}
