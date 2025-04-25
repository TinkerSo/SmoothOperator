#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"

const static char *TAG = "example";

#define HC_SR04_TRIG_GPIO_FRONT  13
#define HC_SR04_ECHO_GPIO_FRONT  12
#define HC_SR04_TRIG_GPIO_LEFT   21
#define HC_SR04_ECHO_GPIO_LEFT   14

TaskHandle_t distance_measurement_task_handle = NULL;

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        cap_val_begin_of_sample = edata->cap_value;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

static void gen_trig(uint32_t trig_gpio) {
    gpio_set_level(trig_gpio, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_gpio, 0);
}

void distance_measurement_task(void *pvParameters) {
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channels");

    // Front sensor channel
    mcpwm_cap_channel_handle_t cap_chan_front = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_front = {
        .gpio_num = HC_SR04_ECHO_GPIO_FRONT,
        .prescale = 1,
        .flags = { .neg_edge = true, .pos_edge = true, .pull_up = true },
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf_front, &cap_chan_front));

    // Left sensor channel
    mcpwm_cap_channel_handle_t cap_chan_left = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_left = {
        .gpio_num = HC_SR04_ECHO_GPIO_LEFT,
        .prescale = 1,
        .flags = { .neg_edge = true, .pos_edge = true, .pull_up = true },
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf_left, &cap_chan_left));

    ESP_LOGI(TAG, "Register capture callbacks");
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_front, &cbs, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_left, &cbs, xTaskGetCurrentTaskHandle()));

    ESP_LOGI(TAG, "Enable capture channels");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_front));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_left));

    ESP_LOGI(TAG, "Configure Trig pins");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << HC_SR04_TRIG_GPIO_FRONT) | (1ULL << HC_SR04_TRIG_GPIO_LEFT),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(HC_SR04_TRIG_GPIO_FRONT, 0);
    gpio_set_level(HC_SR04_TRIG_GPIO_LEFT, 0);

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks_front, tof_ticks_left;
    float distance_front, distance_left;

    while (1) {
        gen_trig(HC_SR04_TRIG_GPIO_FRONT);
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks_front, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks_front * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us <= 35000) {
                distance_front = pulse_width_us / 58;
                ESP_LOGI(TAG, "Front sensor distance: %.2f cm", distance_front);
            }
        }

        gen_trig(HC_SR04_TRIG_GPIO_LEFT);
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks_left, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks_left * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us <= 35000) {
                distance_left = pulse_width_us / 58;
                ESP_LOGI(TAG, "Left sensor distance: %.2f cm", distance_left);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay before next measurement cycle
    }
}

void app_main(void) {
    xTaskCreate(distance_measurement_task, "distance_measurement_task", 4096, NULL, 5, &distance_measurement_task_handle);
}
