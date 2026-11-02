#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "shared_i2c.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "VL53L1X_api.h"

#define SHT31_ADDR        0x44
#define MLX90640_ADDR     0x33
#define VL53L1X_DEV       0x52

#define BUZZER_GPIO       4

#define MOISTURE_CHANNEL  ADC_CHANNEL_2
#define MOISTURE_THRESHOLD 1000

#define DISTANCE_MIN_MM   50
#define DISTANCE_MAX_MM   2000

#define EMISSIVITY        0.95f
#define TA_SHIFT          8.0f

static const char *TAG = "WALLSIGHT";

static i2c_master_dev_handle_t sht31_dev;
static adc_oneshot_unit_handle_t adc_handle;

static uint16_t eeData[832];
static uint16_t frameData[834];
static float thermal[768];
static paramsMLX90640 mlxParams;


static uint8_t sht31_crc(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 0x80)
                    ? (crc << 1) ^ 0x31
                    : (crc << 1);
        }
    }

    return crc;
}


static esp_err_t sht31_read(float *temperature, float *humidity)
{
    const uint8_t cmd[2] = {0x24, 0x00};

    esp_err_t err = i2c_master_transmit(
        sht31_dev,
        cmd,
        sizeof(cmd),
        100
    );

    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t rx[6];

    err = i2c_master_receive(
        sht31_dev,
        rx,
        sizeof(rx),
        100
    );

    if (err != ESP_OK)
        return err;

    if (sht31_crc(&rx[0], 2) != rx[2] ||
        sht31_crc(&rx[3], 2) != rx[5])
    {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_t = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t raw_h = ((uint16_t)rx[3] << 8) | rx[4];

    *temperature =
        -45.0f + 175.0f * raw_t / 65535.0f;

    *humidity =
        100.0f * raw_h / 65535.0f;

    return ESP_OK;
}


static int moisture_read(void)
{
    int total = 0;

    for (int i = 0; i < 5; i++)
    {
        int raw = 0;

        adc_oneshot_read(
            adc_handle,
            MOISTURE_CHANNEL,
            &raw
        );

        total += raw;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return total / 5;
}


static bool distance_read(uint16_t *distance_mm)
{
    uint8_t ready = 0;

    for (int attempts = 0; attempts < 100; attempts++)
    {
        if (VL53L1X_CheckForDataReady(
                VL53L1X_DEV,
                &ready
            ) != VL53L1X_ERROR_NONE)
        {
            return false;
        }

        if (ready)
            break;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!ready)
        return false;

    if (VL53L1X_GetDistance(
            VL53L1X_DEV,
            distance_mm
        ) != VL53L1X_ERROR_NONE)
    {
        return false;
    }

    VL53L1X_ClearInterrupt(VL53L1X_DEV);

    return true;
}


static bool thermal_read(
    float *minimum,
    float *maximum,
    float *average)
{
    float sum = 0.0f;

    for (int page = 0; page < 2; page++)
    {
        int status = MLX90640_GetFrameData(
            MLX90640_ADDR,
            frameData
        );

        if (status < 0)
            return false;

        float ta = MLX90640_GetTa(
            frameData,
            &mlxParams
        );

        float tr = ta - TA_SHIFT;

        MLX90640_CalculateTo(
            frameData,
            &mlxParams,
            EMISSIVITY,
            tr,
            thermal
        );
    }

    *minimum = thermal[0];
    *maximum = thermal[0];

    for (int i = 0; i < 768; i++)
    {
        if (thermal[i] < *minimum)
            *minimum = thermal[i];

        if (thermal[i] > *maximum)
            *maximum = thermal[i];

        sum += thermal[i];
    }

    *average = sum / 768.0f;

    return true;
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting WallSight demonstration");

    /* ---------------- BUZZER ---------------- */

    gpio_config_t buzzer_cfg = {
        .pin_bit_mask = 1ULL << BUZZER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&buzzer_cfg));

    /*
     * Low-level trigger buzzer:
     * HIGH = OFF
     * LOW  = ON
     */
    gpio_set_level(BUZZER_GPIO, 1);


    /* ---------------- SHARED I2C ---------------- */

    i2c_master_bus_handle_t bus =
        wallsight_i2c_get_bus();


    /* ---------------- SHT31 ---------------- */

    i2c_device_config_t sht_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT31_ADDR,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus,
            &sht_cfg,
            &sht31_dev
        )
    );


    /* ---------------- MOISTURE ADC ---------------- */

    adc_oneshot_unit_init_cfg_t adc_init = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_init,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t adc_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            MOISTURE_CHANNEL,
            &adc_cfg
        )
    );


    /* ---------------- VL53L1X ---------------- */

    ESP_LOGI(TAG, "Initializing VL53L1X...");

    if (VL53L1X_SensorInit(VL53L1X_DEV)
        != VL53L1X_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L1X initialization failed");
        return;
    }

    if (VL53L1X_StartRanging(VL53L1X_DEV)
        != VL53L1X_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L1X ranging failed");
        return;
    }


    /* ---------------- MLX90640 ---------------- */

    ESP_LOGI(TAG, "Initializing MLX90640...");

    MLX90640_I2CInit();

    if (MLX90640_DumpEE(
            MLX90640_ADDR,
            eeData
        ) != 0)
    {
        ESP_LOGE(TAG, "MLX90640 EEPROM read failed");
        return;
    }

    if (MLX90640_ExtractParameters(
            eeData,
            &mlxParams
        ) != 0)
    {
        ESP_LOGE(TAG, "MLX90640 calibration failed");
        return;
    }

    MLX90640_SetRefreshRate(
        MLX90640_ADDR,
        3
    );

    ESP_LOGI(TAG, "ALL WALLSIGHT SENSORS READY");


    while (1)
    {
        float ambient_temp = 0.0f;
        float humidity = 0.0f;

        float thermal_min = 0.0f;
        float thermal_max = 0.0f;
        float thermal_avg = 0.0f;

        uint16_t distance = 0;

        int moisture = 0;


        /* ----------- Read all sensors ----------- */

        bool distance_ok =
            distance_read(&distance);

        bool surface_detected =
            distance_ok &&
            distance >= DISTANCE_MIN_MM &&
            distance <= DISTANCE_MAX_MM;


        bool sht_ok =
            sht31_read(
                &ambient_temp,
                &humidity
            ) == ESP_OK;


        bool thermal_ok =
            thermal_read(
                &thermal_min,
                &thermal_max,
                &thermal_avg
            );


        moisture = moisture_read();


        /* ----------- Sensor interaction ----------- */

        bool environment_ready =
            sht_ok;

        bool thermal_ready =
            thermal_ok;

        float thermal_difference =
            thermal_avg - thermal_min;


        /*
         * These three sensors must all be working
         * before WallSight considers the surface
         * ready for moisture confirmation.
         */
        bool scan_ready =
            surface_detected &&
            environment_ready &&
            thermal_ready;


        /*
         * Moisture probe is the final confirmation.
         */
        bool moisture_detected =
            moisture >= MOISTURE_THRESHOLD;


        bool moisture_confirmed =
            scan_ready &&
            moisture_detected;


        /* ----------- Buzzer output ----------- */

        if (moisture_confirmed)
        {
            gpio_set_level(
                BUZZER_GPIO,
                0
            );
        }
        else
        {
            gpio_set_level(
                BUZZER_GPIO,
                1
            );
        }


        /* ----------- Serial display ----------- */

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "          WALLSIGHT SCAN");
        ESP_LOGI(TAG, "========================================");


        if (distance_ok)
        {
            ESP_LOGI(
                TAG,
                "VL53L1X Distance : %u mm",
                distance
            );
        }
        else
        {
            ESP_LOGW(
                TAG,
                "VL53L1X Distance : READ ERROR"
            );
        }


        ESP_LOGI(
            TAG,
            "SHT31 Temperature : %.2f C",
            ambient_temp
        );

        ESP_LOGI(
            TAG,
            "SHT31 Humidity    : %.2f %%",
            humidity
        );

        ESP_LOGI(
            TAG,
            "Thermal Minimum   : %.2f C",
            thermal_min
        );

        ESP_LOGI(
            TAG,
            "Thermal Maximum   : %.2f C",
            thermal_max
        );

        ESP_LOGI(
            TAG,
            "Thermal Average   : %.2f C",
            thermal_avg
        );

        ESP_LOGI(
            TAG,
            "Thermal Variation : %.2f C",
            thermal_difference
        );

        ESP_LOGI(
            TAG,
            "Moisture ADC      : %d",
            moisture
        );

        ESP_LOGI(TAG, "----------------------------------------");


        /* ----------- Status ----------- */

        if (!surface_detected)
        {
            ESP_LOGW(
                TAG,
                "SCAN STATUS: Waiting for surface"
            );
        }
        else if (!environment_ready)
        {
            ESP_LOGW(
                TAG,
                "SCAN STATUS: SHT31 unavailable"
            );
        }
        else if (!thermal_ready)
        {
            ESP_LOGW(
                TAG,
                "SCAN STATUS: Thermal camera unavailable"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "SCAN STATUS: Surface analysis ACTIVE"
            );
        }


        if (scan_ready && !moisture_detected)
        {
            ESP_LOGI(
                TAG,
                "RESULT: Surface scanned - NO MOISTURE CONFIRMED"
            );

            ESP_LOGI(
                TAG,
                "BUZZER: OFF"
            );
        }
        else if (scan_ready && moisture_detected)
        {
            ESP_LOGW(
                TAG,
                "RESULT: MOISTURE CONFIRMED!"
            );

            ESP_LOGW(
                TAG,
                "BUZZER: ON"
            );
        }
        else
        {
            ESP_LOGI(
                TAG,
                "BUZZER: OFF"
            );
        }

        ESP_LOGI(TAG, "========================================");


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
