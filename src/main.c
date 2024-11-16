// standad library
#include <stdio.h>
#include <math.h>
#include "esp_log.h"

// freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// I2C
#include "driver/i2c.h"
#define I2C_MASTER_SCL_IO           22    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0     /*!< I2C master doesn't need buffer */
#define PCA9685_ADDR                0x40  /*!< PCA9685 I2C address */

// Servos
//#include "pca9685.h"

// ADC
#include "esp_adc/adc_oneshot.h"
#define ADC_SAMPLES_TO_AVG 32
#define ADC_SAMPLING_PERIOD_MS 100
static const adc_channel_t adc_pins[] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_3,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};
#define ADC_PINS_COUNT (sizeof(adc_pins) / sizeof(adc_pins[0]))
static adc_oneshot_unit_handle_t adc1_handle;
static int adc_vals[ADC_PINS_COUNT] = {-1};
static SemaphoreHandle_t adc_mutex;

// Event Groups
//static EventGroupHandle_t main_event_group;
//const int ADC_READY_FLAG = BIT0;
//const int I2C_READY_FLAG = BIT1;

/* -------------------------------------------------
Functions
---------------------------------------------------*/
static void _init_adc(void)
{
    ESP_LOGI("INIT_ADC", "Initializing ADC...");

    // ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("INIT_ADC", "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI("INIT_ADC", "ADC unit initialized successfully");

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_10,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < ADC_PINS_COUNT; i++) {
        ret = adc_oneshot_config_channel(adc1_handle, adc_pins[i], &config);
        if (ret != ESP_OK) {
            ESP_LOGE("INIT_ADC", "Failed to configure ADC channel %d: %s", adc_pins[i], esp_err_to_name(ret));
            return;
        }
        ESP_LOGI("INIT_ADC", "ADC channel %d configured successfully", adc_pins[i]);
    }

    ESP_LOGI("INIT_ADC", "ADC initialization complete");
}

static void _init_i2c_master(void)
{
    ESP_LOGI("INIT_I2C_MASTER", "Initializing I2C master...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("INIT_I2C_MASTER", "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI("INIT_I2C_MASTER", "I2C parameters configured successfully");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("INIT_I2C_MASTER", "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI("INIT_I2C_MASTER", "I2C driver installed successfully");

    ESP_LOGI("INIT_I2C_MASTER", "I2C master initialization complete");
}

/*
static void pca9685_init(pca9685_t *dev)
{
    memset(dev, 0, sizeof(pca9685_t));

    ESP_ERROR_CHECK(pca9685_init_desc(dev, PCA9685_ADDR, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(pca9685_init(dev));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(dev, 50)); // Set frequency to 50 Hz for servos
}
*/

int read_adc_value(adc_channel_t channel)
{
    int adc_val = 0;
    for (int i = 0; i < ADC_SAMPLES_TO_AVG; ++i)
    {
        int raw_val;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &raw_val));
        adc_val += raw_val;
    }
    adc_val /= ADC_SAMPLES_TO_AVG;
    return adc_val;
}

/* -------------------------------------------------
Tasks
---------------------------------------------------*/
static void _adc_sampling_task(void *pvParameters)
{
    while (1)
    {
        for (int j = 0; j < ADC_PINS_COUNT; ++j)
        {
            int new_val = read_adc_value(adc_pins[j]);

            if (abs(new_val - adc_vals[j]) > 5) //~0.5% change for 12 bit ADC
            {
                // Take the mutex before updating the ADC values
                if (xSemaphoreTake(adc_mutex, portMAX_DELAY) == pdTRUE) {
                    adc_vals[j] = new_val;
                    xSemaphoreGive(adc_mutex); // Release the mutex after updating
                }

                char name[10];
                snprintf(name, sizeof(name), "ADC-%d", j);
                ESP_LOGI("ADC", "Channel %d: %d", j, new_val);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLING_PERIOD_MS));
    }
}

static void _another_task(void *pvParameters)
{
    while (1)
    {
        // Take the mutex before accessing the ADC values
        if (xSemaphoreTake(adc_mutex, portMAX_DELAY) == pdTRUE) {
            for (int j = 0; j < ADC_PINS_COUNT; ++j)
            {
                ESP_LOGI("ANOTHER_TASK", "ADC-%d: %d", j, adc_vals[j]);
            }
            xSemaphoreGive(adc_mutex); // Release the mutex after accessing
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // Create the event group
    /*
    main_event_group = xEventGroupCreate();
    if (main_event_group == NULL) {
        ESP_LOGE("APP_MAIN", "Failed to create event group");
        return;
    }
    */

    // Initialize hardware and other components
    _init_adc();
    _init_i2c_master();

    // Initialize PCA9685
//    pca9685_t dev;
//    pca9685_init(&dev);

    // Set servo angle to 90 degrees on channel 0
//    set_servo_angle(&dev, 0, 90.0);

    // Create the adc mutex
    adc_mutex = xSemaphoreCreateMutex();
    if (adc_mutex == NULL) {
        ESP_LOGE("APP_MAIN", "Failed to create mutex");
        return;
    }

    // Create the ADC sampling task
    if (xTaskCreate(_adc_sampling_task, "TaskADCSampling", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE("APP_MAIN", "Failed to create ADC sampling task");
    }

    // Create the additional task
    /*
    if (xTaskCreate(_another_task, "AnotherTask", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE("APP_MAIN", "Failed to create another task");
    }
    */

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}