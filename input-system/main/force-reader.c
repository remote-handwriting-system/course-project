#include "force-reader.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// ADC configuration
#define FSR_ADC_CHANNEL ADC_CHANNEL_0
#define ADC_MAX_VALUE   4095

// module state
static const char *TAG = "FORCE_READER";
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static bool is_initialized = false;

esp_err_t force_reader_init(void) {
    if (is_initialized) {
        ESP_LOGW(TAG, "Force reader already initialized");
        return ESP_OK;
    }

    // initialize ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // configure the ADC Channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12-bit (0-4095)
        .atten = ADC_ATTEN_DB_12,         // 0V - 3.3V input range
    };

    ret = adc_oneshot_config_channel(adc1_handle, FSR_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
        return ret;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "Force reader initialized successfully");
    return ESP_OK;
}


esp_err_t force_reader_read_raw(uint16_t *raw_value) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Force reader not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (raw_value == NULL) {
        ESP_LOGE(TAG, "Null pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    int adc_read_result = 0;
    
    esp_err_t ret = adc_oneshot_read(adc1_handle, FSR_ADC_CHANNEL, &adc_read_result);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    *raw_value = (uint16_t)adc_read_result;

    return ESP_OK;
}

esp_err_t force_reader_read_normalized(float *force) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Force reader not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (force == NULL) {
        ESP_LOGE(TAG, "Null pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_value = 0;
    esp_err_t ret = force_reader_read_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }

    // normalize to 0.0 - 1.0 range
    *force = (float)raw_value / (float)ADC_MAX_VALUE;

    return ESP_OK;
}

esp_err_t force_reader_deinit(void) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "Force reader not initialized, nothing to deinitialize");
        return ESP_OK;
    }

    esp_err_t ret = adc_oneshot_del_unit(adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc1_handle = NULL;
    is_initialized = false;
    ESP_LOGI(TAG, "Force reader deinitialized");
    return ESP_OK;
}
