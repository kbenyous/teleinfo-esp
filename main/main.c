#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_task_wdt.h"

#define TELEINFO_UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_04)
#define RXD_PIN (GPIO_NUM_10)
#define BLINK_PIN (GPIO_NUM_26)

static const char *TAG = "Teleinfo";
static const int RX_BUF_SIZE = 1048;

// static const char teleinfo_frame_start = 0x02;
//  static const char teleinfo_frame_end = 0x03;

#define TIC_FRAME_START (char)0x02
#define TIC_FRAME_END (char)0x03
#define TIC_DATASET_START (char)0x0A
#define TIC_DATASET_END (char)0x0D
#define TIC_DATASET_SEP (char)0x09

#define JSON_OUTPUT_BUFFER_SIZE 2048

static QueueHandle_t uart1_queue;

static uint8_t led_blink_state = false;

static void configure_led(void)
{
    gpio_reset_pin(BLINK_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_PIN, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_PIN, led_blink_state = !led_blink_state);
}

void frame_to_json(char *input_buffer, int input_buffer_size, char *output_buffer, int output_buffer_size)
{

    /*
    Frame = set of datasets

    Dataset = 2 or 3 tokens : [key, value] or [key, timestamp, value]
    0x0A KEY 0x09 TIMESTAMP 0x09 VALUE 0x09 CHECKSUM 0x0D
    0x0A KEY 0x09 VALUE 0x09 CHECKSUM 0x0D
    */

    int read_idx = 0;
    memset(output_buffer, 0x00, output_buffer_size);

    // nb of tokens = dataset size : 1= key only, 2= key+value, 3= key+timestamp+value
    char *tokens[4];
    int tokens_count = 0;

    bool first_dataset = true; // For appending ',' in enumerations in JSON output

    u_char computed_checksum = 0x00;
    u_char parsed_checksum = 0x00;

    while (read_idx < input_buffer_size)
    {

        switch (input_buffer[read_idx])
        {

        case TIC_FRAME_START:
            strncat(output_buffer, "{", strlen(output_buffer) - 1);
            first_dataset = true;
            break;

        case TIC_FRAME_END:
            strncat(output_buffer, "}", strlen(output_buffer) - 1);
            break;

        case TIC_DATASET_START:
            computed_checksum = 0x00;
            tokens_count = 0;

            // The first data for this dataset set will start at the next char (not read yet)
            tokens[tokens_count] = input_buffer + ((read_idx + 1) * sizeof(char));
            break;

        case TIC_DATASET_SEP:
            computed_checksum += input_buffer[read_idx];

            // Closes the previous token, erases the TIC_DATA_SEP with a 0x00 string termination in the input buffer
            input_buffer[read_idx] = 0x00;

            // Prepares the next token : it will start at the next char (not read yet)
            tokens[++tokens_count] = input_buffer + ((read_idx + 1) * sizeof(char));
            break;

        case TIC_DATASET_END:
            // The checksum is the char just before the end of the dataset
            parsed_checksum = input_buffer[read_idx - 1];

            // Compute checksum
            computed_checksum = (computed_checksum & 0x3F) + 0x20;

            if (computed_checksum == parsed_checksum)
            {
                if (first_dataset)
                {
                    first_dataset = false;
                }
                else
                {
                    strncat(output_buffer, ",", output_buffer_size - strlen(output_buffer));
                }

                strncat(output_buffer, "\'", output_buffer_size - strlen(output_buffer));
                strncat(output_buffer, tokens[0], output_buffer_size - strlen(output_buffer));
                strncat(output_buffer, "\':", output_buffer_size - strlen(output_buffer));

                if (tokens_count == 2)
                {
                    // key=>Value only
                    strncat(output_buffer, "{'value':'", output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, tokens[1], output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, "'}", output_buffer_size - strlen(output_buffer));
                }
                else if (tokens_count == 3)
                {
                    // key=>Timestamp+value
                    strncat(output_buffer, "{'value':'", output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, tokens[2], output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, "','time':'", output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, tokens[1], output_buffer_size - strlen(output_buffer));
                    strncat(output_buffer, "'}", output_buffer_size - strlen(output_buffer));
                }
            }

            break;

        default:
            // If next char is the dataset end char, then current char is the checksum.
            if (input_buffer[read_idx + 1] != TIC_DATASET_END)
            {
                // Current char is not the read checksum, it must be included in the checksum computation
                computed_checksum += input_buffer[read_idx];
            }
        }
        read_idx++;
    }
}

static void uart_event_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[APP] uart_event_task");

    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RX_BUF_SIZE);
    for (;;)
    {

        // Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, 10000 / portTICK_PERIOD_MS))
        {
            bzero(dtmp, RX_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", TELEINFO_UART_NUM);

            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                //  uart_read_bytes(TELEINFO_UART_NUM, dtmp, event.size, pdMS_TO_TICKS( 1100 ));
                //  ESP_LOGI(TAG, "[DATA EVT]:");
                //  uart_write_bytes(TELEINFO_UART_NUM, (const char*) dtmp, event.size);
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(TELEINFO_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(TELEINFO_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                // ESP_LOGI(TAG, "uart UART_PATTERN_DET");
                uart_get_buffered_data_len(TELEINFO_UART_NUM, &buffered_size);
                int pos = -1;
                while (-1 != (pos = uart_pattern_pop_pos(TELEINFO_UART_NUM)))
                {

                    //  +1 to get the end of the frame
                    //  The char that triggered this pattern_det is excluded from the computed data_len.
                    //  But it exists in the buffer, since it has triggered the pattern_det !
                    uart_read_bytes(TELEINFO_UART_NUM, dtmp, pos + 1, 100 / portTICK_PERIOD_MS);

                    if ((dtmp[0] == TIC_FRAME_START) && (dtmp[pos] == TIC_FRAME_END))
                    {
                        blink_led();
                        // ESP_LOGI(TAG, "trame complete: %s", dtmp);
                        char *output_buffer = calloc(JSON_OUTPUT_BUFFER_SIZE, sizeof(char));

                        frame_to_json((char *)dtmp, pos+1, output_buffer, JSON_OUTPUT_BUFFER_SIZE);
                        ESP_LOGI(TAG, "Frame : %s", output_buffer);
                        free(output_buffer);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "trame incomplete: %s", dtmp);
                    }
                }

                break;
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "[APP] uart_event_task noevent");
        }
    }
}

void app_main(void)
{

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    configure_led();
    ESP_LOGI(TAG, "[APP] Configuring UART...");
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_7_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,

    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(TELEINFO_UART_NUM, RX_BUF_SIZE * 2, 0, 20, &uart1_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(TELEINFO_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TELEINFO_UART_NUM, UART_PIN_NO_CHANGE, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_enable_pattern_det_baud_intr(TELEINFO_UART_NUM, TIC_FRAME_END, 1, 2, 0, 0);
    ESP_LOGI(TAG, "[APP] Configuring UART... done");
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, &uart1_queue, tskIDLE_PRIORITY, NULL);
}
