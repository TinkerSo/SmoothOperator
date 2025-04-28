#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

static const char *TAG = "example";

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2000 // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90  // Minimum angle
#define SERVO_MAX_DEGREE        90   // Maximum angle

// GPIO pins for each servo
#define SERVO_PULSE_GPIO_1       33 // Motor A
#define SERVO_PULSE_GPIO_2       15 // Motor B

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle, bool invert) {
    if (invert) {
        angle = -angle; // Invert the angle for specific servos
    }
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void configure_servo(int gpio_pin, bool invert_angle, mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator) {
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, *timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(*oper, &comparator_config, comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(*oper, &generator_config, generator));

    // Set initial position to center
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*comparator, example_angle_to_compare(0, invert_angle)));

    // Set generator actions
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(*generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(*generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(*timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP));
}

void app_main(void) {
    ESP_LOGI(TAG, "Setting up servos");

    // Timer, operator, comparator, and generator handles for each wheel
    mcpwm_timer_handle_t timer1 = NULL, timer2 = NULL;
    mcpwm_oper_handle_t oper1 = NULL, oper2 = NULL;
    mcpwm_cmpr_handle_t comparator1 = NULL, comparator2 = NULL;
    mcpwm_gen_handle_t generator1 = NULL, generator2 = NULL;

    // Configure each servo
    configure_servo(SERVO_PULSE_GPIO_1, false, &timer1, &oper1, &comparator1, &generator1);
    configure_servo(SERVO_PULSE_GPIO_2, true, &timer2, &oper2, &comparator2, &generator2); // Inverted for backward movement

    int angle = 0;
    int turn_angle = 0;
    const int max_angle = 90;
    const int min_angle = -90;
    const int angle_step = 5;

    // UART setup for input
    const int uart_num = UART_NUM_0;
    uart_driver_install(uart_num, 256, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(uart_num);

    char inputChar;

    while (1) {
        if (read(uart_num, &inputChar, 1) > 0) {
            switch (inputChar) {
                case 's':
                    // angle = (angle + angle_step) <= max_angle ? angle + angle_step : max_angle;
                    // ESP_LOGI(TAG, "Motor A and B moving forward, Angle: %d", angle);
                    angle = 90;
                    ESP_LOGI(TAG, "Motor A and B moving forward, Angle: %d", angle);
                    break;
                case 'w':
                    // angle = (angle - angle_step) >= min_angle ? angle - angle_step : min_angle;
                    // ESP_LOGI(TAG, "Motor A and B moving backward, Angle: %d", angle);
                    angle = -90;
                    ESP_LOGI(TAG, "Motor A and B moving backward, Angle: %d", angle);
                    break;
                case 'd':
                    // turn_angle = (turn_angle + angle_step) <= max_angle ? turn_angle + angle_step : max_angle;
                    // ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(turn_angle, false)));
                    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(-turn_angle, true)));
                    turn_angle = 90;
                    ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(-turn_angle, true)));
                    continue;  // Skip further processing to maintain current forward/backward angle
                case 'a':
                    // turn_angle = (turn_angle - angle_step) >= -max_angle ? turn_angle - angle_step : min_angle;
                    // ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-turn_angle, false)));
                    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(turn_angle, true)));
                    turn_angle = -90;
                    ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(-turn_angle, true)));
                    continue;  // Skip further processing to maintain current forward/backward angle
                case 'x':
                    angle = 0;
                    ESP_LOGI(TAG, "Motor A and B moving backward, Angle: %d", angle);
                    break;
            }
            // Update both motors to new forward or backward position
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(angle, false)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(angle, true)));
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay for debounce handling
    }
}
