/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_random.h"
/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (0)
#define ECHO_UART_BAUD_RATE (115200)
#define ECHO_TASK_STACK_SIZE (8192)
#define MYID 1

static const char *TAG = "UART TEST";

#define BUF_SIZE (512)

// uint8_t checksum(char *arr, uint8_t len) {
//     uint16_t cs = 0;
//     for (int i = 0; i < len; i++) {
//         cs += arr[i];
//         if (cs > 255) {
//             cs -= 255;
//         }
//     }
//     return (uint8_t)(~cs & 0xFF);
// }

uint8_t checksum(uint8_t *arr, uint8_t len)
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
    {
        checksum += arr[i];
    }
    checksum = 0xFF & ~checksum;

    return (checksum);
}

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *reply = (uint8_t *)malloc(BUF_SIZE);
    uint8_t index = 0;
    uint8_t start = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5;

    uint16_t cell_voltage[] = {3640, 3501, 3618, 3687,
                               3522, 3668, 3659, 3557,
                               3550, 3641, 3507, 3525,
                               3668, 3649, 3584, 3700};

    const uint16_t cell_voltage_copy[] = {3640, 3501, 3618, 3687,
                                          3522, 3668, 3659, 3557,
                                          3550, 3641, 3507, 3525,
                                          3668, 3649, 3584, 3700};
    int16_t temperature[5];

    uint8_t dsg_fet = 0; // discharge FET state
    uint8_t chg_fet = 0; // charge FET state
    uint16_t alarm_bits = 0;
    uint16_t cb_active_cells = 0; // Cell Balancing Active Cells
    uint8_t uv_fault = 0;         // under-voltage fault state
    uint8_t ov_fault = 0;         // over-voltage fault state
    uint8_t scd_fault = 0;        // short-circuit fault state
    uint8_t ocd_fault = 0;        // over-current fault state
    uint8_t occ_fault = 0;
    uint8_t ocd2_fault = 0;
    uint8_t pchg_fet = 0; // pre-charge FET state
    uint8_t pdsg_fet = 0; // pre-discharge FET state
    uint8_t ddsg_pin = 0;
    uint8_t dchg_pin = 0;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        int count = xTaskGetTickCount();
        uint16_t pack_voltage = 1500 * sin(2 * 3.141 * count) + 7000;
        int16_t pack_current = 10000 * sin(2 * 3.141 * count + 3.141 / 2) + 15000;
        temperature[0] = 200 * sin(2 * 3.141 * count + 3.141) + 3000;
        temperature[1] = 200 * sin(2 * 3.141 * count + 3.141) + 3000;
        temperature[2] = 200 * sin(2 * 3.141 * count + 3.141) + 3000;
        temperature[3] = 200 * sin(2 * 3.141 * count + 3.141) + 3000;
        temperature[4] = 200 * sin(2 * 3.141 * count + 3.141) + 3000;
        uint16_t stack_voltage = 500 * sin(2 * 3.141 * count + 3.141);
        uint32_t accumulated_charge_int = 20 * sin(2 * 3.141 * count + 3.141 / 2) + 40;
        uint32_t accumulated_charge_frac = 1000 * sin(2 * 3.141 * count + 3.141) + 1000;
        uint32_t accumulated_charge_time = 100 * sin(2 * 3.141 * count + 3.141) + 100;
        // Wait for the next cycle.
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // ESP_LOGI(TAG, "Tick!");

        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data + index, (BUF_SIZE - 1), 2);
        index = len;

        if (data[start] > len + index)
        {
            continue;
        }

        // check if from ui, if not, ignore message
        if (data[start + 1] != 'u' || data[start + 2] != 'i')
        {
            start = data[start];
            continue;
        }

        int reply_len = 0;

        uint8_t recipient = data[start + 3];

        if (recipient != 0xFF && recipient != MYID)
        {
            start = data[start];
            continue;
        }

        // switch (data[start + 4]) {
        //     case 0:
        //         reply[0] = 7;
        //         reply[1] = 'b';
        //         reply[2] = 'p';
        //         reply[3] = 0;
        //         reply[4] = voltage & 0xFF;
        //         reply[5] = voltage >> 8;
        //         reply[6] = checksum((char *)reply, 6);
        //         reply_len = 7;
        //         break;
        //     case 1:
        //         reply[0] = 7;
        //         reply[1] = 'b';
        //         reply[2] = 'p';
        //         reply[3] = 0;
        //         reply[4] = current & 0xFF;
        //         reply[5] = current >> 8;
        //         reply[6] = checksum((char *)reply, 6);
        //         reply_len = 7;
        //         break;
        //     case 2:
        //         reply[0] = 7;
        //         reply[1] = 'b';
        //         reply[2] = 'p';
        //         reply[3] = 0;
        //         reply[4] = temperature & 0xFF;
        //         reply[5] = temperature >> 8;
        //         reply[6] = checksum((char *)reply, 6);
        //         reply_len = 7;
        //         break;
        //     case 3:
        //         reply[0] = 7;
        //         reply[1] = 'b';
        //         reply[2] = 'p';
        //         reply[3] = 0;
        //         reply[4] = 16;
        //         reply[5] = checksum((char *)reply, 5);
        //         reply_len = 6;
        //         break;
        //     case 4:
        //         reply[0] = sizeof(cell_voltage) + 5;
        //         reply[1] = 'b';
        //         reply[2] = 'p';
        //         reply[3] = 0;
        //         reply_len = sizeof(cell_voltage);
        //         for (int i = 0; i < reply_len / 2; i++) {
        //             reply[2 * i + 4] = cell_voltage[i] & 0xFF;
        //             reply[2 * i + 5] = cell_voltage[i] >> 8;
        //             cell_voltage[i]--;
        //             if (cell_voltage_copy[i] - cell_voltage[i] > 100) {
        //                 cell_voltage[i] = cell_voltage_copy[i];
        //             }
        //         }
        //         reply[reply_len + 4] = checksum((char *)reply, reply_len + 4);
        //         reply_len += 5;
        //         break;
        //     default:
        //         reply[0] = 0xAA;
        //         reply[1] = 0xBB;
        //         reply[2] = checksum((char *)reply, 2);
        //         reply_len = 3;
        //         break;
        // }

        switch (data[start + 4])
        {
        case 0: // getCellCount
            reply[0] = 6;
            reply[1] = 'b';
            reply[2] = 's';
            // reply[0] = 7;
            // reply[1] = 'b';
            // reply[2] = 'p';
            reply[3] = 0;
            reply[4] = 16;
            reply[5] = checksum(reply, 5);
            reply_len = 6;
            break;
        case 1:
        {
            reply[0] = 71;
            reply[1] = 'b';
            reply[2] = 's';
            // reply[0] = 7;
            // reply[1] = 'b';
            // reply[2] = 'p';
            reply[3] = 0;
            reply[4] = pack_voltage & 0xFF;
            reply[5] = pack_voltage >> 8;
            reply[6] = pack_current & 0xFF;
            reply[7] = pack_current >> 8;
            reply[8] = temperature[0] & 0xFF;
            reply[9] = temperature[0] >> 8;
            reply[10] = temperature[1] & 0xFF;
            reply[11] = temperature[1] >> 8;
            reply[12] = temperature[2] & 0xFF;
            reply[13] = temperature[2] >> 8;
            reply[14] = temperature[3] & 0xFF;
            reply[15] = temperature[3] >> 8;
            reply[16] = temperature[4] & 0xFF;
            reply[17] = temperature[4] >> 8;
            uint8_t i;
            for (i = 0; i < 16; i++)
            {
                reply[2 * i + 18] = cell_voltage[i] & 0xFF;
                reply[2 * i + 19] = cell_voltage[i] >> 8;
                cell_voltage[i]--;
                if (cell_voltage_copy[i] - cell_voltage[i] > 100)
                {
                    cell_voltage[i] = cell_voltage_copy[i];
                }
            }
            reply[50] = stack_voltage & 0xFF;
            reply[51] = stack_voltage >> 8;
            reply[52] = accumulated_charge_int & 0xFF;
            reply[53] = accumulated_charge_int >> 8 & 0xFF;
            reply[54] = accumulated_charge_int >> 16 & 0xFF;
            reply[55] = accumulated_charge_int >> 24 & 0xFF;
            reply[56] = accumulated_charge_frac & 0xFF;
            reply[57] = accumulated_charge_frac >> 8 & 0xFF;
            reply[58] = accumulated_charge_frac >> 16 & 0xFF;
            reply[59] = accumulated_charge_frac >> 24 & 0xFF;
            reply[60] = accumulated_charge_time & 0xFF;
            reply[61] = accumulated_charge_time >> 8 & 0xFF;
            reply[62] = accumulated_charge_time >> 16 & 0xFF;
            reply[63] = accumulated_charge_time >> 24 & 0xFF;
            reply[64] = chg_fet | dsg_fet << 1;
            reply[65] = alarm_bits & 0x4 >> 2;
            reply[66] = cb_active_cells & 0xFF;
            reply[67] = cb_active_cells >> 8;
            reply[68] = uv_fault | ov_fault << 1 | scd_fault << 2 | ocd_fault << 3 | ocd2_fault << 4 | occ_fault << 5;
            reply[69] = pchg_fet | pdsg_fet << 1 | dchg_pin << 2 | ddsg_pin << 3;
            reply[70] = checksum(reply, 70);
            reply_len = 71;
            break;
        }
        case 2: // configReceive
        case 3: // configSend
        default:
            reply[0] = 0xAA;
            reply[1] = 0xBB;
            reply[2] = checksum(reply, 2);
            reply_len = 3;
            break;
        }

        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)reply, reply_len);
        index = 0;
        data[start] = 255;
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}