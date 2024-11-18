#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_private/esp_clk.h"

static const char *TAG = "motor_control";
TaskHandle_t distance_measurement_task_handle = NULL;

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 1000
#define SERVO_MAX_PULSEWIDTH_US 2000
#define SERVO_MIN_DEGREE        -90
#define SERVO_MAX_DEGREE        90

// GPIO pins for each servo
#define SERVO_PULSE_GPIO_1       33 // Motor A
#define SERVO_PULSE_GPIO_2       15 // Motor B

#define HC_SR04_TRIG_GPIO_FRONT  13
#define HC_SR04_ECHO_GPIO_FRONT  12
#define HC_SR04_TRIG_GPIO_LEFT   21
#define HC_SR04_ECHO_GPIO_LEFT   14

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD        20000

static inline uint32_t example_angle_to_compare(int angle, bool invert) {
    if (invert) {
        angle = -angle;
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

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*comparator, example_angle_to_compare(0, invert_angle)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(*generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(*generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(*timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP));
}

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
            distance_front = pulse_width_us / 58.0;  // Conversion to centimeters
            // if (distance_front <= 350.0) {           // Ignore very high values
            //     ESP_LOGI(TAG, "Front sensor distance: %.2f cm", distance_front);
            // } else {
            //     ESP_LOGW(TAG, "Front sensor out of range");
            // }
        }

        gen_trig(HC_SR04_TRIG_GPIO_LEFT);
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks_left, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks_left * (1000000.0 / esp_clk_apb_freq());
            distance_left = pulse_width_us / 58.0;   // Conversion to centimeters
            // if (distance_left <= 350.0) {            // Ignore very high values
            //     ESP_LOGI(TAG, "Left sensor distance: %.2f cm", distance_left);
            // } else {
            //     ESP_LOGW(TAG, "Left sensor out of range");
            // }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay before the next measurement cycle
    }
}



void motor_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Setting up servos");

    mcpwm_timer_handle_t timer1 = NULL, timer2 = NULL;
    mcpwm_oper_handle_t oper1 = NULL, oper2 = NULL;
    mcpwm_cmpr_handle_t comparator1 = NULL, comparator2 = NULL;
    mcpwm_gen_handle_t generator1 = NULL, generator2 = NULL;

    configure_servo(SERVO_PULSE_GPIO_1, false, &timer1, &oper1, &comparator1, &generator1);
    configure_servo(SERVO_PULSE_GPIO_2, true, &timer2, &oper2, &comparator2, &generator2);

    int angle = 0;
    int turn_angle = 0;
    const int max_angle = 90;
    const int min_angle = -90;

    const int uart_num = UART_NUM_0;
    uart_driver_install(uart_num, 256, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(uart_num);

    char inputChar;

    while (1) {
        if (read(uart_num, &inputChar, 1) > 0) {
            switch (inputChar) {
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
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(-turn_angle, true)));
                    continue;
                case 'a':
                    turn_angle = -90;
                    ESP_LOGI(TAG, "Motor A moving forward, Motor B moving backward, Turn Angle: %d", turn_angle);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(turn_angle, false)));
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(-turn_angle, true)));
                    continue;
                case 'x':
                    angle = 0;
                    ESP_LOGI(TAG, "Motor A and B stopping, Angle: %d", angle);
                    break;
            }
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(angle, false)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(angle, true)));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
    xTaskCreate(distance_measurement_task, "distance_measurement_task", 4096, NULL, 5, &distance_measurement_task_handle);
}

Above is good
////////////////////


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "esp_attr.h"

// GPIO definitions for ultrasonic and motor control
#define GPIO_ULTRASONIC_TRIG  13
#define GPIO_ULTRASONIC_ECHO  12
#define GPIO_MOTOR_A_PWM      33
#define GPIO_MOTOR_B_PWM      15

#define DISTANCE_THRESHOLD_CM 20  // Threshold distance in centimeters
#define SPEED_OF_SOUND_CM_US  0.0343  // Speed of sound in cm/us

static const char *TAG = "SmoothOperator";

// Global flag to control motor state
volatile bool stop_motors_flag = false;

// Initialize GPIO for Ultrasonic Sensor
void ultrasonic_gpio_init() {
    gpio_set_direction(GPIO_ULTRASONIC_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ULTRASONIC_ECHO, GPIO_MODE_INPUT);
    gpio_set_level(GPIO_ULTRASONIC_TRIG, 0);
}

// Send a pulse to the Ultrasonic Sensor
void send_ultrasonic_pulse() {
    gpio_set_level(GPIO_ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(10);  // Send a 10us pulse
    gpio_set_level(GPIO_ULTRASONIC_TRIG, 0);
}

// MCPWM configuration to stop motors
void stop_motors() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_MOTOR_A_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_MOTOR_B_PWM);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,  // frequency = 1kHz
        .cmpr_a = 0,        // duty cycle of PWMxA = 0
        .cmpr_b = 0,        // duty cycle of PWMxB = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

// Task to monitor distance and control motor state
void ultrasonic_task(void *pvParameters) {
    uint32_t echo_start_time, echo_end_time, pulse_time_us;
    float distance_cm;

    while (1) {
        send_ultrasonic_pulse();
        // Wait for echo high
        while (gpio_get_level(GPIO_ULTRASONIC_ECHO) == 0);
        echo_start_time = esp_timer_get_time();

        // Wait for echo low
        while (gpio_get_level(GPIO_ULTRASONIC_ECHO) == 1);
        echo_end_time = esp_timer_get_time();

        // Calculate pulse duration and distance
        pulse_time_us = echo_end_time - echo_start_time;
        distance_cm = (pulse_time_us * SPEED_OF_SOUND_CM_US) / 2;

        if (distance_cm < DISTANCE_THRESHOLD_CM) {
            ESP_LOGI(TAG, "Object detected at %.2f cm, stopping motors", distance_cm);
            stop_motors_flag = true;
        } else {
            ESP_LOGI(TAG, "Distance: %.2f cm", distance_cm);
            stop_motors_flag = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay to prevent continuous triggering
    }
}

// Task to control the motors based on sensor inputs
void motor_control_task(void *pvParameters) {
    // Set up MCPWM for motor control
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_MOTOR_A_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_MOTOR_B_PWM);

    // Configuration for the PWM operation
    mcpwm_config_t pwm_config = {
        .frequency = 1000,  // PWM frequency at 1kHz
        .cmpr_a = 0,        // Initial duty cycle for motor A at 0%
        .cmpr_b = 0,        // Initial duty cycle for motor B at 0%
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    // Initialize PWM with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Main loop to control motor based on distance measurements
    while (1) {
        if (stop_motors_flag) {
            // Stop motors if the flag is set
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);  // Set duty cycle to 0% for Motor A
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);  // Set duty cycle to 0% for Motor B
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // Set duty type to 0 which stops the motor
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        } else {
            // Run motors if no object is detected
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 75);  // Set duty cycle to 75% for Motor A
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 75);  // Set duty cycle to 75% for Motor B
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // Apply the duty cycle setting
            mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms before the next loop iteration
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "Initializing...");
    ultrasonic_gpio_init();
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
}
