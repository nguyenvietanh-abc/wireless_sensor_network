// DHT22.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "DHT22_header.h"

// Sử dụng cùng chân GPIO với LM35 để tương thích
#define DHT_GPIO GPIO_NUM_32  
static const char *TAG = "DHT22";

// Cấu hình chân GPIO
static void configure_pin(void) {
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 1);
}

// Tạo tín hiệu bắt đầu giao tiếp với DHT22
static bool dht22_start_signal(void) {
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    // Gửi tín hiệu start bằng cách kéo xuống ít nhất 1ms
    gpio_set_level(DHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));  // Ít nhất 18ms
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(30);  // Đợi 20-40us
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    
    // Đợi DHT22 phản hồi (sẽ kéo xuống trong vòng 80us)
    int timeout = 0;
    while(gpio_get_level(DHT_GPIO) == 1) {
        if(timeout++ > 100) return false;
        esp_rom_delay_us(1);
    }
    
    // Đợi DHT22 kéo lên (khoảng 80us)
    timeout = 0;
    while(gpio_get_level(DHT_GPIO) == 0) {
        if(timeout++ > 100) return false;
        esp_rom_delay_us(1);
    }
    
    // Đợi DHT22 kéo xuống (khoảng 80us)
    timeout = 0;
    while(gpio_get_level(DHT_GPIO) == 1) {
        if(timeout++ > 100) return false;
        esp_rom_delay_us(1);
    }
    
    return true;
}

// Đọc dữ liệu từ DHT22
static esp_err_t dht22_read_bits(uint8_t *data, size_t length) {
    uint8_t current_byte = 0;
    uint8_t bit_count = 0;
    
    for(int i = 0; i < length * 8; i++) {
        // Đợi cạnh lên
        int timeout = 0;
        while(gpio_get_level(DHT_GPIO) == 0) {
            if(timeout++ > 100) return ESP_ERR_TIMEOUT;
            esp_rom_delay_us(1);
        }
        
        // Đo độ rộng xung cao (26-28us = 0, 70us = 1)
        esp_rom_delay_us(40);  // Đợi và lấy mẫu sau 40us
        current_byte <<= 1;
        if(gpio_get_level(DHT_GPIO)) {
            current_byte |= 1;
        }
        
        bit_count++;
        if(bit_count == 8) {
            data[i/8] = current_byte;
            bit_count = 0;
            current_byte = 0;
        }
        
        // Đợi cạnh xuống
        timeout = 0;
        while(gpio_get_level(DHT_GPIO) == 1) {
            if(timeout++ > 100) return ESP_ERR_TIMEOUT;
            esp_rom_delay_us(1);
        }
    }
    
    return ESP_OK;
}

// Khởi tạo DHT22
void DHT22_init(void) {
    configure_pin();
}

// Giải phóng DHT22
void DHT22_de_init(void) {
    // Không cần thao tác đặc biệt để giải phóng
}

// Đọc nhiệt độ và độ ẩm từ DHT22
esp_err_t DHT22_read_data(float *temperature, float *humidity) {
    uint8_t data[5] = {0};
    
    if (!dht22_start_signal()) {
        ESP_LOGE(TAG, "Không thể gửi tín hiệu bắt đầu");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t err = dht22_read_bits(data, 5);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi đọc dữ liệu");
        return err;
    }
    
    // Kiểm tra checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "Lỗi checksum");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Chuyển đổi dữ liệu
    *humidity = ((data[0] << 8) + data[1]) / 10.0;
    int16_t temp16 = (data[2] << 8) + data[3];
    *temperature = temp16 / 10.0;
    
    ESP_LOGI(TAG, "Nhiệt độ: %.1f°C, Độ ẩm: %.1f%%", *temperature, *humidity);
    return ESP_OK;
}