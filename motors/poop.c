#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "example";

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90  // Minimum angle
#define SERVO_MAX_DEGREE        90   // Maximum angle

// GPIO pins for each servo
#define SERVO_PULSE_GPIO_1       33
#define SERVO_PULSE_GPIO_2       15
#define SERVO_PULSE_GPIO_3       32

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle, bool invert)
{
    if (invert) {
        angle = -angle; // Invert the angle for specific servos
    }
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void configure_servo(int gpio_pin, bool invert_angle, mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group as the timer
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

void app_main(void)
{
    ESP_LOGI(TAG, "Setting up servos");

    // Timer, operator, comparator, and generator handles for each wheel
    mcpwm_timer_handle_t timer1 = NULL, timer2 = NULL, timer3 = NULL;
    mcpwm_oper_handle_t oper1 = NULL, oper2 = NULL, oper3 = NULL;
    mcpwm_cmpr_handle_t comparator1 = NULL, comparator2 = NULL, comparator3 = NULL;
    mcpwm_gen_handle_t generator1 = NULL, generator2 = NULL, generator3 = NULL;

    // Configure each servo
    configure_servo(SERVO_PULSE_GPIO_1, true, &timer1, &oper1, &comparator1, &generator1); // Inverted for GPIO 33
    configure_servo(SERVO_PULSE_GPIO_2, false, &timer2, &oper2, &comparator2, &generator2);
    configure_servo(SERVO_PULSE_GPIO_3, false, &timer3, &oper3, &comparator3, &generator3);

    int angle = 0;
    int step = 2;

    while (1) {
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);

        // Update each comparator to adjust the angle of each wheel
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(angle, true))); // Inverted angle for GPIO 33
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(angle, false)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator3, example_angle_to_compare(angle, false)));

        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Reverse direction when limits are reached
        if ((angle + step) > 60 || (angle + step) < -60) {
            step *= -1;
        }
        angle += step;
    }
}
