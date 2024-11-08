// Copyright 2019-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "board.h"
#include "tusb_hid.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include <string.h>
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"




#define TAG "HID Example"


#define EXAMPLE_MOUSE_OFFSET_X 8
#define EXAMPLE_MOUSE_OFFSET_Y 8
#define EXAMPLE_BUTTON_NUM 4
#define CONFIG_EXAMPLE_IPV4 
#define CONFIG_EXAMPLE_IPV4_ADDR "172.20.10.10"
#define CONFIG_EXAMPLE_PORT 3000


#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT


#ifdef CONFIG_BOARD_ESP32S3_USB_OTG_EV
#define EXAMPLE_BUTTON_UP BOARD_IO_BUTTON_UP
#define EXAMPLE_BUTTON_DOWN BOARD_IO_BUTTON_DW
#define EXAMPLE_BUTTON_LEFT BOARD_IO_BUTTON_OK
#define EXAMPLE_BUTTON_RIGHT BOARD_IO_BUTTON_MENU
#else
#define EXAMPLE_BUTTON_UP 10
#define EXAMPLE_BUTTON_DOWN 11
#define EXAMPLE_BUTTON_LEFT 0
#define EXAMPLE_BUTTON_RIGHT 14
#endif


#define BOARD_UART_PORT UART_NUM_1
#define UART_TXD	        43
#define UART_RXD	        44
#define UART_RTS		    (UART_PIN_NO_CHANGE)
#define UART_CTS		    (UART_PIN_NO_CHANGE)
 
#define UART_BAUD_RATE		115200
#define UART_BUF_SIZE		512
 

const uart_port_t uart_num = UART_NUM_1;
char payload[5] = "&&&&";


static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            /*
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }
            */

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


 
void uart_init() {
	if (uart_is_driver_installed(uart_num)) {
		ESP_LOGE(TAG, "UART_NUM_1 already been uesd!");
		uart_driver_delete(uart_num);
	}
	// set uart params
	uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
	// config uart params
	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	// set uart pin
	ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
	// install driver
	ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
}


static int s_button_gpio[EXAMPLE_BUTTON_NUM] = {EXAMPLE_BUTTON_UP, EXAMPLE_BUTTON_DOWN, EXAMPLE_BUTTON_LEFT, EXAMPLE_BUTTON_RIGHT};
static button_handle_t s_button_handles[EXAMPLE_BUTTON_NUM] = {NULL};

static int get_button_gpio(button_handle_t btn_hdl)
{
    for (size_t i = 0; i < EXAMPLE_BUTTON_NUM; i++) {
        if (s_button_handles[i] == btn_hdl) {
            return s_button_gpio[i];
        }
    }
    return -1;
}

#ifdef CONFIG_SUBCLASS_KEYBOARD
static void button_keyboard_cb(void *arg)
{
    button_handle_t button_hdl = (button_handle_t)arg;
    int button_gpio = get_button_gpio(button_hdl);
    uint8_t _keycode[6] = { 0 };
    switch (button_gpio) {
        case EXAMPLE_BUTTON_UP:
            _keycode[0] = HID_KEY_U;
            break;

        case EXAMPLE_BUTTON_DOWN:
            _keycode[0] = HID_KEY_D;
            break;

        case EXAMPLE_BUTTON_LEFT:
            _keycode[0] = HID_KEY_L;
            break;

        case EXAMPLE_BUTTON_RIGHT:
            _keycode[0] = HID_KEY_R;
            break;

        default:
            break;
    }
    tinyusb_hid_keyboard_report(_keycode);
    ESP_LOGI(TAG, "Keyboard %c", _keycode[0] - HID_KEY_A + 'a');
}
#else if CONFIG_SUBCLASS_MOUSE
static void button_mouse_cb(void *arg)
{
    button_handle_t button_hdl = (button_handle_t)arg;
    int button_gpio = get_button_gpio(button_hdl);
    int mouse_offset_x = 0, mouse_offset_y = 0;
    switch (button_gpio) {
        case EXAMPLE_BUTTON_UP:
            mouse_offset_y = -EXAMPLE_MOUSE_OFFSET_Y;
            break;

        case EXAMPLE_BUTTON_DOWN:
            mouse_offset_y = EXAMPLE_MOUSE_OFFSET_Y;
            break;

        case EXAMPLE_BUTTON_LEFT:
            mouse_offset_x = -EXAMPLE_MOUSE_OFFSET_X;
            break;

        case EXAMPLE_BUTTON_RIGHT:
            mouse_offset_x = EXAMPLE_MOUSE_OFFSET_X;
            break;

        default:
            break;
    }
    tinyusb_hid_mouse_move_report(mouse_offset_x, mouse_offset_y, 0, 0);
    ESP_LOGI(TAG, "Mouse x=%d y=%d", mouse_offset_x, mouse_offset_y);
}
#endif

void app_main(void)
{
    /* board init, peripherals will be initialized based on menuconfig */
    iot_board_init();

#ifdef CONFIG_BOARD_ESP32S3_USB_OTG_EV
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(false, false);
#endif
    uart_init();

    iot_board_wifi_init();
    /* buttons init, buttons used to simulate keyboard or mouse events */
    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .active_level = 0,
        },
    };

    for (size_t i = 0; i < EXAMPLE_BUTTON_NUM; i++) {
        cfg.gpio_button_config.gpio_num = s_button_gpio[i];
        s_button_handles[i] = iot_button_create(&cfg);
        if(s_button_handles[i] == NULL) {
            ESP_LOGE(TAG, "button=%d created failed", s_button_gpio[i]);
            assert(0);
        }
    }

    /* Install tinyusb driver, Please enable `CONFIG_TINYUSB_HID_ENABLED` in menuconfig */
    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

#ifdef CONFIG_SUBCLASS_KEYBOARD
    button_cb_t button_cb = button_keyboard_cb;
    ESP_LOGI(TAG, "Keyboard demo");
#else if CONFIG_SUBCLASS_MOUSE
    button_cb_t button_cb = button_mouse_cb;
    ESP_LOGI(TAG, "Mouse demo");
#endif

    /* register button callback, send HID report when click button */
    for (size_t i = 0; i < EXAMPLE_BUTTON_NUM; i++) {
        iot_board_button_register_cb(s_button_handles[i], BUTTON_SINGLE_CLICK, button_cb);
    }

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_crerate_default());

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    while (1) {

       uint8_t _keycode[6] = { 0 };
      // _keycode[0] = HID_KEY_R;
     //  tinyusb_hid_keyboard_report(_keycode);

       
        char tx_data[512] = {0};                 // 自己赋值
        uart_write_bytes(uart_num, (const char *) tx_data, strlen(tx_data));        //发送数据
        int len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len));  // 检测接收缓存区是否有返回数据
        char rx_data[512] = {0};   
        uart_read_bytes(uart_num, rx_data, len, 100);        // 读取数据，100：此参数为sTimeout，以 RTOS 滴答计数
        int i=0;
        ESP_LOGI(TAG, "Mouse len=%d",len);
        while(i+4<len){
            //tinyusb_hid_mouse_move_report(8, 8, 0, 0);
            if(rx_data[i]==0 && rx_data[i+1]==0 && rx_data[i+2]!=0 && rx_data[i+3]==0){
                ESP_LOGI(TAG, "Mouse x=%d",rx_data[i+2]);
                uint8_t _keycode[6] = { 0 };
                _keycode[0] = rx_data[i+2];
                payload[0]=rx_data[i+2];
                tinyusb_hid_keyboard_report(_keycode);
            }
            i=i+1;
        }
        
        while(i<len){
             ESP_LOGI(TAG, "Mouse x=%d",rx_data[i]);
             i++;
        }
        //tinyusb_hid_mouse_move_report(8, 8, 0, 0);
        //ESP_LOGI(TAG, "Mouse x=8 y=8");
        vTaskDelay(100 / portTICK_RATE_MS);

    }
}