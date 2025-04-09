#include "esp_log.h"
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define PULSE_TAG "ENCODER_PULSE"

pcnt_unit_handle_t setup_pcnt_encoder(gpio_num_t enc_a, gpio_num_t enc_b, uint32_t high_limit, uint32_t low_limit){
    ESP_LOGI(PULSE_TAG, "Init pcnt driver to decode rotary signal");
    
    pcnt_unit_config_t pcnt_cfg = {
        .high_limit = high_limit,
        .low_limit = low_limit,
        .flags.accum_count = true // enable counter accumulation
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_cfg, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    
    // Channel A
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = enc_a,
        .level_gpio_num = enc_b
    };
    pcnt_unit_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_cfg, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Channel B
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = enc_b,
        .level_gpio_num = enc_a
    };
    pcnt_unit_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_cfg, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return pcnt_unit;
}

int get_encoder_pulses(pcnt_unit_handle_t pcnt_unit){
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));
    return count;
}