#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"

#define TXD_PIN 17
#define RXD_PIN 16

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, 
UART_PIN_NO_CHANGE);
}

static void tx_task(void *arg) {
    static const char *test_str = "Hello Jetson\n";
    while (1) {
        uart_write_bytes(UART_NUM_1, test_str, strlen(test_str));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    init();
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, 
configMAX_PRIORITIES - 1, NULL);
}

