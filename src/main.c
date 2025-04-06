#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_gatt_common_api.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          100000

#define BMP180_I2C_ADDRESS          0x77
#define AHT10_I2C_ADDRESS           0x38

#define BMP180_OVERSAMPLING         3

#define I2C_WAIT_TIMEOUT_100        100 / portTICK_PERIOD_MS
#define I2C_WAIT_TIMEOUT_1000       1000 / portTICK_PERIOD_MS

#define FILTER_WINDOW_SIZE 5
#define MAX_SENSOR_VALUES 10

#define SERVICE_UUID  0x00FF
#define CHAR_UUID     0xFF01

typedef struct {
    float buffer[MAX_SENSOR_VALUES];
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} CircularBuffer;

void buffer_init(CircularBuffer* buf);
void buffer_add(CircularBuffer* buf, float value);
int buffer_get_value(CircularBuffer* buf, float* value);
float moving_median_filter(CircularBuffer* buf);

void i2c_master_init(void);
void device_read_bytes(i2c_master_dev_handle_t device, uint8_t reg, uint8_t *data, size_t len);
int16_t device_read_int16(i2c_master_dev_handle_t device, uint8_t reg);
uint16_t device_read_uint16(i2c_master_dev_handle_t device, uint8_t reg);

void update_char_value(const char *prefix);
void ble_init(void);
void prepare_ble_data(float bmp180_temp, float bmp180_press, float aht10_temp, float aht10_humidity,
    CircularBuffer* bmp180_temp_buf, CircularBuffer* bmp180_press_buf, CircularBuffer* aht10_temp_buf, CircularBuffer* aht10_humidity_buf);
void ble_periodic_task(void *pvParameters);

void aht10_init(void);
void aht10_read_data(float *temperature, float *humidity);
void aht10_task(void *pvParameters);

void bmp180_read_calibration(void);
int16_t bmp180_read_uncompensated_temperature(void);
uint32_t bmp180_read_uncompensated_pressure(uint8_t oss);
int32_t bmp180_computeB5(uint8_t oss);
float bmp180_read_temperature(uint8_t oss);
uint32_t bmp180_read_pressure(uint8_t oss);
void bmp180_task(void *pvParameters);

// Tags for esp logging
static const char *BMPTAG = "BMP180";
static const char *AHTTAG = "AHT10";
static const char *TAG = "GENERIC";

/// I2C master bus and device handles
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t aht10_handle;
i2c_master_dev_handle_t bmp180_handle;

/// Circular buffers for sensor data (not used, was for debugging)
CircularBuffer bmp180_temp_buf;
CircularBuffer bmp180_pressure_buf;
CircularBuffer aht10_temp_buf;
CircularBuffer aht10_hum_buf;

/// BMP180 calibration values
int16_t ac1;
int16_t ac2;
int16_t ac3;
uint16_t ac4;
uint16_t ac5;
uint16_t ac6;
int16_t b1;
int16_t b2;
int16_t mb;
int16_t mc;
int16_t md;

static uint8_t char_value[500] = "bruhhh 111111"; // Ensure enough space
static esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static esp_attr_value_t char_attr_value = {
    .attr_max_len = sizeof(char_value),
    .attr_len = 500,
    .attr_value = char_value
};
static uint16_t handle_table[2];


//////////     CIRCULAR BUFFER START     //////////
void buffer_init(CircularBuffer* buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->size = 0;
}

void buffer_add(CircularBuffer* buf, float value) {
    if (buf->size < MAX_SENSOR_VALUES) {
        buf->buffer[buf->head] = value;
        buf->head = (buf->head + 1) % MAX_SENSOR_VALUES;
        buf->size++;
    } else {
        buf->buffer[buf->head] = value;
        buf->head = (buf->head + 1) % MAX_SENSOR_VALUES;
        buf->tail = (buf->tail + 1) % MAX_SENSOR_VALUES;
    }
}

int buffer_get_value(CircularBuffer* buf, float* value) {
    if (buf->size == 0) {
        return -1;
    }

    *value = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % MAX_SENSOR_VALUES;
    buf->size--;
    return 0;
}

float buffer_moving_median_filter(CircularBuffer* buf) {
    float sorted[MAX_SENSOR_VALUES];
    memcpy(sorted, buf->buffer, sizeof(sorted));

    // Sort the array to calculate the median
    for (int i = 0; i < buf->size; i++) {
        for (int j = i + 1; j < buf->size; j++) {
            if (sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    int mid = buf->size / 2;
    if (buf->size % 2 == 0) {
        return (sorted[mid - 1] + sorted[mid]) / 2.0;
    } else {
        return sorted[mid];
    }
}

float buffer_standardDeviation(CircularBuffer* buf) {
    float mean = 0;
    for (int i = 0; i < buf->size; i++) {
        mean += buf->buffer[i];
    }
    mean /= buf->size;

    float variance = 0;
    for (int i = 0; i < buf->size; i++) {
        variance += (buf->buffer[i] - mean) * (buf->buffer[i] - mean);
    }
    variance /= buf->size;
    return sqrt(variance);
}

float buffer_max(CircularBuffer* buf) {
    float max = buf->buffer[0];
    for (int i = 1; i < buf->size; i++) {
        if (buf->buffer[i] > max) {
            max = buf->buffer[i];
        }
    }
    return max;
}

float buffer_min(CircularBuffer* buf) {
    float min = buf->buffer[0];
    for (int i = 1; i < buf->size; i++) {
        if (buf->buffer[i] < min) {
            min = buf->buffer[i];
        }
    }
    return min;
}

//////////     CIRCULAR BUFFER END     //////////

/// @brief 
/// Initializes the I2C master bus and adds the BMP180 and AHT10 devices to it.
void i2c_master_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t bmp180_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP180_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp180_cfg, &bmp180_handle));

    i2c_device_config_t aht10_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT10_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &aht10_cfg, &aht10_handle));
}

/// @brief 
/// Transmits to `reg` and receives `len` bytes into `data` from the device.
/// @param device device to communicate with
/// @param reg register to read from
/// @param data pointer to the buffer to store the received data
/// @param len number of bytes to read
void device_read_bytes(i2c_master_dev_handle_t device, uint8_t reg, uint8_t *data, size_t len) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(device, &reg, 1, data, len, I2C_WAIT_TIMEOUT_1000));
}

/// @brief 
/// Reads a 16-bit signed integer from the specified register of the device.
/// @param device device to communicate with
/// @param reg register to read from
/// @return 
int16_t device_read_int16(i2c_master_dev_handle_t device, uint8_t reg) {
    uint8_t data[2];
    device_read_bytes(device, reg, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}

/// @brief 
/// Reads a 16-bit unsigned integer from the specified register of the device.
/// @param device device to communicate with
/// @param reg register to read from
/// @return 
uint16_t device_read_uint16(i2c_master_dev_handle_t device, uint8_t reg) {
    uint8_t data[2];
    device_read_bytes(device, reg, data, 2);
    return (uint16_t)((data[0] << 8) | data[1]);
}

//////////     AHT10 I2C LIBRARY START     //////////
/*
AHT10 Datasheet:
https://server4.eca.ir/eshop/AHT10/Aosong_AHT10_en_draft_0c.pdf
*/
void aht10_init() {
    uint8_t reset_cmd = 0xBA;
    ESP_ERROR_CHECK(i2c_master_transmit(aht10_handle, &reset_cmd, sizeof(reset_cmd), I2C_WAIT_TIMEOUT_100)); // Reset AHT10
    vTaskDelay(pdMS_TO_TICKS(50));
}

void aht10_read_data(float *temperature, float *humidity) {
    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_master_transmit(aht10_handle, trigger_cmd, sizeof(trigger_cmd), I2C_WAIT_TIMEOUT_100)); // Trigger measurement

    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_ERROR_CHECK(i2c_master_receive(aht10_handle, data, 6, I2C_WAIT_TIMEOUT_100));

    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *humidity = ((float)raw_humidity / 1048576.0) * 100.0;
    *temperature = ((float)raw_temperature / 1048576.0) * 200.0 - 50.0;
}
//////////     AHT10 I2C LIBRARY END     //////////

//////////     BMP180 I2C LIBRARY START     //////////
/*
BMP180 Datasheet:
https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
void bmp180_read_calibration() {
    ac1 = device_read_int16(bmp180_handle, 0xAA);
    ac2 = device_read_int16(bmp180_handle, 0xAC);
    ac3 = device_read_int16(bmp180_handle, 0xAE);
    ac4 = device_read_uint16(bmp180_handle, 0xB0);
    ac5 = device_read_uint16(bmp180_handle, 0xB2);
    ac6 = device_read_uint16(bmp180_handle, 0xB4);
    b1 = device_read_int16(bmp180_handle, 0xB6);
    b2 = device_read_int16(bmp180_handle, 0xB8);
    mb = device_read_int16(bmp180_handle, 0xBA);
    mc = device_read_int16(bmp180_handle, 0xBC);
    md = device_read_int16(bmp180_handle, 0xBE);
}

int16_t bmp180_read_uncompensated_temperature() {
    uint8_t buffer[2] = { 0xF4, 0x2E };
    ESP_ERROR_CHECK(i2c_master_transmit(bmp180_handle, buffer, sizeof(buffer), I2C_WAIT_TIMEOUT_1000));

    vTaskDelay(pdMS_TO_TICKS(10)); // Data read wait

    int16_t UT = device_read_int16(bmp180_handle, 0xF6);
    return UT;
}

uint32_t bmp180_read_uncompensated_pressure(uint8_t oss) {
    uint8_t buffer[2] = { 0xF4, 0x34 + (oss << 6) };
    ESP_ERROR_CHECK(i2c_master_transmit(bmp180_handle, buffer, sizeof(buffer), I2C_WAIT_TIMEOUT_1000));

    vTaskDelay(pdMS_TO_TICKS((2 + (3 << oss))));  // Data read wait

    uint8_t data[3];
    device_read_bytes(bmp180_handle, 0xF6, data, 3);
    
    uint32_t UP = ((data[0] << 16) | (data[1] << 8) | data[2]);
    UP >>= (8 - oss);
    return UP;
}

int32_t bmp180_computeB5(uint8_t oss) {
    int16_t UT = bmp180_read_uncompensated_temperature();
    int32_t X1 = ((UT - ac6) * ac5) >> 15;
    int32_t X2 = (mc << 11) / (X1 + md);
    return X1 + X2;
}

float bmp180_read_temperature(uint8_t oss) {
    int32_t B5 = bmp180_computeB5(oss);
    float T = ((B5 + 8) >> 4) / 10;
    return T;
}

uint32_t bmp180_read_pressure(uint8_t oss) {
    int32_t b3, b5, b6, x1, x2, x3, p;
    uint32_t up, b4, b7;

    b5 = bmp180_computeB5(oss);

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)ac1) * 4 + x3) << oss) + 2) >> 2;

    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

    up = bmp180_read_uncompensated_pressure(oss);

    b7 = ((uint32_t)(up - b3) * (50000 >> oss));
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return (uint32_t)p;
}
//////////     BMP180 I2C LIBRARY END     //////////


/// @brief 
/// Implemented for debugging purposes. No use.
/// @param pvParameters 
void bmp180_task(void *pvParameters) {
    bmp180_read_calibration();

    uint8_t oss = BMP180_OVERSAMPLING; // Oversampling setting (0, 1, 2, or 3)

    buffer_init(&bmp180_temp_buf);
    buffer_init(&bmp180_pressure_buf);

    while (1) {
        float temp = bmp180_read_temperature(oss);
        uint32_t press = bmp180_read_pressure(oss);

        float sealevelPressure = 101325l;
        float altitude = 44330 * (1.0 - pow(press / sealevelPressure, 0.1903));

        buffer_add(&bmp180_temp_buf, temp);
        buffer_add(&bmp180_pressure_buf, press);
        
        float filtered_temp = moving_median_filter(&bmp180_temp_buf);
        float filtered_press = moving_median_filter(&bmp180_pressure_buf);
        
        ESP_LOGI(BMPTAG, "Temperature: %.2f C | Pressure: %.2f Pa | Altitude: %.2f m", filtered_temp, filtered_press, altitude);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/// @brief 
/// Implemented for debugging purposes. No use.
/// @param pvParameters 
void aht10_task(void *pvParameters) {
    aht10_init();

    buffer_init(&aht10_temp_buf);
    buffer_init(&aht10_hum_buf);

    while (1) {

        float temperature, humidity;
        aht10_read_data(&temperature, &humidity);

        buffer_add(&aht10_temp_buf, temperature);
        buffer_add(&aht10_hum_buf, humidity);

        float filtered_temperature = moving_median_filter(&aht10_temp_buf);
        float filtered_humidity = moving_median_filter(&aht10_hum_buf);

        ESP_LOGI(AHTTAG, "Temperature: %.2f C | Humidity: %.2f %%", filtered_temperature, filtered_humidity);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void update_char_value(const char *prefix) {
    snprintf((char *)char_value, sizeof(char_value), prefix);
    char_attr_value.attr_len = strlen((char *)char_value);

    ESP_LOGI(TAG, "Updated char value length: %d", char_attr_value.attr_len);
}

static void gatts_profile_event_handler(
    esp_gatts_cb_event_t event, 
    esp_gatt_if_t gatts_if,
    esp_ble_gatts_cb_param_t *param) 
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "Registering service...");
            esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
                .id = { .uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = SERVICE_UUID } } },
                .is_primary = true,
            }, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created!");
            handle_table[0] = param->create.service_handle;
            esp_ble_gatts_start_service(handle_table[0]);

            param->mtu.mtu = 500;
            esp_ble_gatts_add_char(
                handle_table[0],
                &(esp_bt_uuid_t){ .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = CHAR_UUID } },
                ESP_GATT_PERM_READ, // allow unauthenticated reads
                char_property,
                &char_attr_value,
                NULL
            );
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "Characteristic added!");
            param->mtu.mtu = 500;
            handle_table[1] = param->add_char.attr_handle;
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected!");
            esp_ble_gatts_send_indicate(gatts_if, param->connect.conn_id, handle_table[1], char_attr_value.attr_len, char_attr_value.attr_value, false);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU size: %d", param->mtu.mtu);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected!");
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "Characteristic Read Requested");
            param->mtu.mtu = 500;
            
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = char_attr_value.attr_len;
            memcpy(rsp.attr_value.value, char_attr_value.attr_value, char_attr_value.attr_len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            //esp_ble_gatts_send_indicate(gatts_if, param->read.conn_id, handle_table[1], char_attr_value.attr_len, char_attr_value.attr_value, false);
            
            ESP_LOGI(TAG, "Chunk size: %d", char_attr_value.attr_len);
            ESP_LOGI(TAG, "Chunk value: %.*s", char_attr_value.attr_len, char_attr_value.attr_value);


            break;

        default:
            break;
    }
}

void ble_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_ble_gatts_register_callback(gatts_profile_event_handler);
    esp_ble_gatts_app_register(0);

    esp_ble_gatt_set_local_mtu(500);

    //esp_ble_gap_set_device_name("ESP32_Sensor_device");
    esp_ble_gap_set_device_name("ESP32_BordaAssignment");
    esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    });
    esp_ble_gatt_set_local_mtu(500);
}

void prepare_ble_data(float bmp180_temp, float bmp180_press, float aht10_temp, float aht10_humidity,
    CircularBuffer* bmp180_temp_buf, CircularBuffer* bmp180_press_buf, CircularBuffer* aht10_temp_buf, CircularBuffer* aht10_humidity_buf) {
    // Calculate statistics for each sensor
    float bmp180_temp_stddev = buffer_standardDeviation(bmp180_temp_buf);
    float bmp180_temp_max = buffer_max(bmp180_temp_buf);
    float bmp180_temp_min = buffer_min(bmp180_temp_buf);
    float bmp180_temp_median = buffer_moving_median_filter(bmp180_temp_buf);

    float bmp180_press_stddev = buffer_standardDeviation(bmp180_press_buf);
    float bmp180_press_max = buffer_max(bmp180_press_buf);
    float bmp180_press_min = buffer_min(bmp180_press_buf);
    float bmp180_press_median = buffer_moving_median_filter(bmp180_press_buf);

    float aht10_temp_stddev = buffer_standardDeviation(aht10_temp_buf);
    float aht10_temp_max = buffer_max(aht10_temp_buf);
    float aht10_temp_min = buffer_min(aht10_temp_buf);
    float aht10_temp_median = buffer_moving_median_filter(aht10_temp_buf);

    float aht10_humidity_stddev = buffer_standardDeviation(aht10_humidity_buf);
    float aht10_humidity_max = buffer_max(aht10_humidity_buf);
    float aht10_humidity_min = buffer_min(aht10_humidity_buf);
    float aht10_humidity_median = buffer_moving_median_filter(aht10_humidity_buf);

    // Prepare the advertisement data
    uint8_t adv_data[500];
    snprintf((char*)adv_data, sizeof(adv_data), // size is 287 bytes in tests
    "BMP_TEMP: %.2f MAX: %.2f MIN: %.2f STDDEV: %.2f MEDIAN: %.2f | BMP_PRESS: %.2f MAX: %.2f MIN: %.2f STDDEV: %.2f MEDIAN: %.2f | AHT_TEMP: %.2f MAX: %.2f MIN: %.2f STDDEV: %.2f MEDIAN: %.2f | AHT_HUM: %.2f MAX: %.2f MIN: %.2f STDDEV: %.2f MEDIAN: %.2f",
    bmp180_temp, bmp180_temp_max, bmp180_temp_min, bmp180_temp_stddev, bmp180_temp_median,
    bmp180_press, bmp180_press_max, bmp180_press_min, bmp180_press_stddev, bmp180_press_median,
    aht10_temp, aht10_temp_max, aht10_temp_min, aht10_temp_stddev, aht10_temp_median,
    aht10_humidity, aht10_humidity_max, aht10_humidity_min, aht10_humidity_stddev, aht10_humidity_median);

    update_char_value((char*)adv_data);

}

void ble_periodic_task(void *pvParameters) {
    CircularBuffer bmp180_temp_buf;
    CircularBuffer bmp180_press_buf;
    CircularBuffer aht10_temp_buf;
    CircularBuffer aht10_humidity_buf;

    buffer_init(&bmp180_temp_buf);
    buffer_init(&bmp180_press_buf);
    buffer_init(&aht10_temp_buf);
    buffer_init(&aht10_humidity_buf);

    while (1) {
        float bmp180_temp = bmp180_read_temperature(BMP180_OVERSAMPLING);
        uint32_t bmp180_press = bmp180_read_pressure(BMP180_OVERSAMPLING);
        float aht10_temp, aht10_humidity;
        aht10_read_data(&aht10_temp, &aht10_humidity);

        // collect
        buffer_add(&bmp180_temp_buf, bmp180_temp);
        buffer_add(&bmp180_press_buf, (float)bmp180_press);
        buffer_add(&aht10_temp_buf, aht10_temp);
        buffer_add(&aht10_humidity_buf, aht10_humidity);

        //ble...
        prepare_ble_data(bmp180_temp, (float)bmp180_press, aht10_temp, aht10_humidity,
                      &bmp180_temp_buf, &bmp180_press_buf, &aht10_temp_buf, &aht10_humidity_buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void) {
    ble_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    aht10_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    bmp180_read_calibration();
    vTaskDelay(pdMS_TO_TICKS(50));
    

    ESP_LOGI(TAG, "MASTER INITALIZED");
    //xTaskCreate(bmp180_task, "bmp180_task", 8192, NULL, 5, NULL);
    //xTaskCreate(aht10_task, "aht10_task", 8192, NULL, 5, NULL);
    xTaskCreate(ble_periodic_task, "ble_periodic_task", 8192, NULL, 5, NULL);
}