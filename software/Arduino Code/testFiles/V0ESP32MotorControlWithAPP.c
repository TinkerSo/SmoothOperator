#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "string.h"

static const char *TAG = "motor_control";

#define SERVO_MIN_PULSEWIDTH_US 1000
#define SERVO_MAX_PULSEWIDTH_US 2000
#define SERVO_MIN_DEGREE        -90
#define SERVO_MAX_DEGREE        90

#define SERVO_PULSE_GPIO_1       33
#define SERVO_PULSE_GPIO_2       15
#define TXD_PIN                  17
#define RXD_PIN                  16

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD        20000
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024

static mcpwm_cmpr_handle_t comparator1 = NULL;
static mcpwm_cmpr_handle_t comparator2 = NULL;

static inline uint32_t angle_to_compare(int angle, bool invert) {
    if (invert) angle = -angle;
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void setup_motors() {
    ESP_LOGI(TAG, "Initializing motor controllers");

    mcpwm_timer_handle_t timer1, timer2;
    mcpwm_oper_handle_t oper1, oper2;
    mcpwm_gen_handle_t generator1, generator2;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer1));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer2));

    mcpwm_operator_config_t operator_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper1));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper2));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config, &comparator1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config, &comparator2));

    mcpwm_generator_config_t generator_config = { .gen_gpio_num = SERVO_PULSE_GPIO_1 };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config, &generator1));
    generator_config.gen_gpio_num = SERVO_PULSE_GPIO_2;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config, &generator2));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(0, false)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(0, true)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));
}

void setup_uart() {
    ESP_LOGI(TAG, "Initializing UART communication");

    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_task(void *arg) {
    uint8_t data[BUF_SIZE];
    int angle = 0, turn_angle = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0;
            ESP_LOGI(TAG, "Received command: %s", (char *)data);

            switch (data[0]) {
                case 's':
                    angle = 90;
                    ESP_LOGI(TAG, "Motor A and B moving forward, Angle: %d", angle);
                    break;
                case 'w':
                    angle = -90;
                    ESP_LOGI(TAG, "Motor A and B moving backward, Angle: %d", angle);
                    break;
                case 'd':
                    turn_angle = 90;
                    ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(-turn_angle, true)));
                    continue;
                case 'a':
                    turn_angle = -90;
                    ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(-turn_angle, true)));
                    continue;
                case 'x':
                    angle = 0;
                    ESP_LOGI(TAG, "Motor A and B moving backward, Angle: %d", angle);
                    break;
            }
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(angle, false)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(angle, true)));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void app_main() {
    ESP_LOGI(TAG, "Starting Motor Control System");

    setup_motors();
    setup_uart();
    xTaskCreate(uart_task, "uart_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
}
