#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define BUZZER_GPIO GPIO_NUM_4

#define WET_THRESHOLD 1000
#define DRY_THRESHOLD 300

#define REQUIRED_WET_READINGS 3
#define SAMPLES_PER_READING 5

static const char *TAG = "WALLSIGHT";

int read_moisture_average(adc_oneshot_unit_handle_t adc_handle)
{
    int total = 0;

    for (int i = 0; i < SAMPLES_PER_READING; i++)
    {
        int raw = 0;

        adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL_2,
            &raw
        );

        total += raw;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return total / SAMPLES_PER_READING;
}

void app_main(void)
{
    // -------------------------
    // Buzzer setup
    // GPIO4 -> 1k resistor -> S8050 base
    // -------------------------
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);

    // -------------------------
    // Moisture sensor setup
    // GPIO3 = ADC1 Channel 2
    // -------------------------
    adc_oneshot_unit_handle_t adc1_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &adc1_handle
        )
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc1_handle,
            ADC_CHANNEL_2,
            &channel_config
        )
    );

    int wet_count = 0;
    bool moisture_detected = false;

    while (1)
    {
        int moisture_raw =
            read_moisture_average(adc1_handle);

        ESP_LOGI(
            TAG,
            "Moisture value: %d",
            moisture_raw
        );

        if (!moisture_detected)
        {
            if (moisture_raw >= WET_THRESHOLD)
            {
                wet_count++;

                ESP_LOGI(
                    TAG,
                    "Possible moisture: %d/%d",
                    wet_count,
                    REQUIRED_WET_READINGS
                );

                if (wet_count >= REQUIRED_WET_READINGS)
                {
                    moisture_detected = true;
                    wet_count = 0;

                    ESP_LOGW(
                        TAG,
                        "MOISTURE CONFIRMED - BUZZER ON"
                    );
                }
            }
            else
            {
                wet_count = 0;
                gpio_set_level(BUZZER_GPIO, 0);

                ESP_LOGI(
                    TAG,
                    "Wall condition: DRY"
                );
            }
        }

        if (moisture_detected)
        {
            if (moisture_raw <= DRY_THRESHOLD)
            {
                moisture_detected = false;

                gpio_set_level(BUZZER_GPIO, 0);

                ESP_LOGI(
                    TAG,
                    "Wall returned to DRY - BUZZER OFF"
                );
            }
            else
            {
                gpio_set_level(BUZZER_GPIO, 1);

                ESP_LOGW(
                    TAG,
                    "MOISTURE PRESENT - BUZZER ON"
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
