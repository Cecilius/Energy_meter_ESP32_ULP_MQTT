// main.c

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"
#include "esp_rom_crc.h"
#include <driver/adc.h>
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp32/ulp.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "esp_sntp.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "ulp_main.h"

static const char *TAG = "Enemon";
#include "secrets.h"
#include "ha_config.h"
#include "MQTT.h"

// How many times will try to connect WiFi
#define CONECTION_RETRY 2

static struct timeval app_start;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
uint32_t pulse_count = 0;
uint32_t previous_pulse_count = 0;
uint32_t time_difference_ms = 0;
esp_reset_reason_t cause;

float battery;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static void init();

static void init_ulp_program(void);

static void update_pulse_count(void);

static void battery_measurement();

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data);

void wifi_init_sta(void);

static void enter_deep_sleep();


void app_main(void)
{
    init();

    // Turn on WiFi
    wifi_init_sta();

    start_MQTT();

    // Send MQTT
    uint32_t power_consumption = 0;
    if(time_difference_ms != 0) power_consumption = (( pulse_count - previous_pulse_count ) * 360000) / time_difference_ms; 
    uint32_t daily_energy = pulse_count / 10;
    uint32_t total_energy = pulse_count / 10;

    char message[12];
    char topic[70];
    char dev_topic[30];
    sprintf(dev_topic, "%s/%s", HA_UNIQ_ID, HA_COMPONENT);
    sprintf(message, "%.2f", battery);
    sprintf(topic, "%s/battery/state", dev_topic);
    send_MQTT(topic, message);
    sprintf(message, "%d", power_consumption);
    sprintf(topic, "%s/power_consumption/state", dev_topic);
    send_MQTT(topic, message);
    sprintf(message, "%d", daily_energy);
    sprintf(topic, "%s/daily_energy/state", dev_topic);
    send_MQTT(topic, message);
    sprintf(message, "%d", total_energy);
    sprintf(topic, "%s/total_energy/state", dev_topic);
    send_MQTT(topic, message);
    if (cause != ESP_RST_DEEPSLEEP)
    {
        char conf_topic[100];
        char device_topic[70];
        char conf_message[400];
        sprintf(device_topic, "homeassistant/%s/%s", HA_COMPONENT, HA_UNIQ_ID);
        sprintf(conf_topic, "%s/power_consumption/config", device_topic);
        sprintf(conf_message, "{\"dev_cla\":\"power\",\"unit_of_meas\":\"W\",\"stat_cla\":\"measurement\",\"name\":\"Power consumption\",\"stat_t\":\"%s/%s/power_consumption/state\",\"uniq_id\":\"%s-power\",\"dev\":{\"ids\":\"%s\",\"name\":\"%s\",\"sw\":\"%s\"}}", HA_UNIQ_ID, HA_COMPONENT, HA_UNIQ_ID, HA_DEV_ID, HA_DEV_NAME, HA_DEV_SW);
        send_MQTT(conf_topic, conf_message);
        sprintf(conf_topic, "%s/daily_energy/config", device_topic);
        sprintf(conf_message, "{\"dev_cla\":\"energy\",\"unit_of_meas\":\"Wh\",\"stat_cla\":\"total\",\"name\":\"Daily energy\",\"stat_t\":\"%s/%s/daily_energy/state\",\"uniq_id\":\"%s-denergy\",\"dev\":{\"ids\":\"%s\",\"name\":\"%s\",\"sw\":\"%s\"}}", HA_UNIQ_ID, HA_COMPONENT, HA_UNIQ_ID, HA_DEV_ID, HA_DEV_NAME, HA_DEV_SW);
        send_MQTT(conf_topic, conf_message);
        sprintf(conf_topic, "%s/total_energy/config", device_topic);
        sprintf(conf_message, "{\"dev_cla\":\"energy\",\"unit_of_meas\":\"Wh\",\"stat_cla\":\"total\",\"name\":\"Total Energy\",\"stat_t\":\"%s/%s/total_energy/state\",\"uniq_id\":\"%s-tenergy\",\"dev\":{\"ids\":\"%s\",\"name\":\"%s\",\"sw\":\"%s\"}}", HA_UNIQ_ID, HA_COMPONENT, HA_UNIQ_ID, HA_DEV_ID, HA_DEV_NAME, HA_DEV_SW);
        send_MQTT(conf_topic, conf_message);
        sprintf(conf_topic, "%s/battery/config", device_topic);
        sprintf(conf_message, "{\"dev_cla\":\"voltage\",\"unit_of_meas\":\"V\",\"stat_cla\":\"measurement\",\"name\":\"Battery level\",\"stat_t\":\"%s/%s/battery/state\",\"uniq_id\":\"%s-battery\",\"dev\":{\"ids\":\"%s\",\"name\":\"%s\",\"sw\":\"%s\"}}", HA_UNIQ_ID, HA_COMPONENT, HA_UNIQ_ID, HA_DEV_ID, HA_DEV_NAME, HA_DEV_SW);
        send_MQTT(conf_topic, conf_message);
    }

    end_MQTT();

    enter_deep_sleep();
}

void init()
{
    cause = esp_reset_reason();
    if (cause != ESP_RST_DEEPSLEEP)
    {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
    }
    else
    {
        printf("ULP wakeup, saving pulse count\n");
        update_pulse_count();
    }

    gettimeofday(&app_start, NULL);
    ESP_LOGI(TAG, "%d.%06d started at %ld.%06ld\n", 0, 0, app_start.tv_sec, app_start.tv_usec);

    battery_measurement();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

}

void battery_measurement()
{
    //configure ADC and get reading from battery
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);
    // Reading divided by full range times referent voltage * voltage divider
    int raw_adc = adc1_get_raw(ADC1_CHANNEL_5);
    ESP_LOGD(TAG, "Raw ADC = %d\n", raw_adc);
    battery = (float)raw_adc / 4096 * 1.1 / 100 * (100 + 330);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < CONECTION_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // WiFi Persistent
    if(cause != ESP_RST_DEEPSLEEP)
        esp_wifi_set_storage(WIFI_STORAGE_RAM);

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();

    esp_netif_dhcpc_stop(my_sta);
    esp_netif_ip_info_t ip_info;

    esp_netif_str_to_ip4(MY_SECRET_STATIC_IP, &ip_info.ip);
    esp_netif_str_to_ip4(MY_SECRET_STATIC_MASK, &ip_info.netmask);
    esp_netif_str_to_ip4(MY_SECRET_STATIC_GW, &ip_info.gw);

    esp_netif_set_ip_info(my_sta, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = MY_SECRET_SSID,
            .password = MY_SECRET_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 MY_SECRET_SSID, MY_SECRET_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 MY_SECRET_SSID, MY_SECRET_PASS);
        enter_deep_sleep();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* To speed-up the program you can use just RTC-GPIO 0-15 */
    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = GPIO_NUM_15;
    int rtcio_num = rtc_io_number_get(gpio_num);
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");
    /* GPIO RED LED. */
    gpio_num_t gpio_num_rl = GPIO_NUM_2;
    int rtcio_num_rl = rtc_io_number_get(gpio_num_rl);
    assert(rtc_gpio_is_valid_gpio(gpio_num_rl) && "GPIO used for pulse counting must be an RTC IO");
    /* GPIO GREEN LED. */
    gpio_num_t gpio_num_gl = GPIO_NUM_4;
    int rtcio_num_gl = rtc_io_number_get(gpio_num_gl);
    assert(rtc_gpio_is_valid_gpio(gpio_num_gl) && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_next_edge = 1;
    ulp_previous_count_l = 0;
    ulp_previous_count_h = 0;
    ulp_signal_count_l = 0;
    ulp_signal_count_h = 0;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_io_number_rl = rtcio_num_rl; /* map from GPIO# to RTC_IO# */
    ulp_io_number_gl = rtcio_num_gl; /* map from GPIO# to RTC_IO# */

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    rtc_gpio_init(gpio_num_rl);
    rtc_gpio_set_direction(gpio_num_rl, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_rl);
    rtc_gpio_pullup_dis(gpio_num_rl);
    rtc_gpio_hold_en(gpio_num_rl);

    rtc_gpio_init(gpio_num_gl);
    rtc_gpio_set_direction(gpio_num_gl, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_gl);
    rtc_gpio_pullup_dis(gpio_num_gl);
    rtc_gpio_hold_en(gpio_num_gl);

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    ulp_time1 = time_us % 65536;
    ulp_time2 = (int64_t)( time_us / 65536 ) % 65536;
    ulp_time3 = (int64_t)( time_us / 4294967296 ) % 65536;
    ulp_time4 = (int64_t)( time_us / ( 4294967296 * 65536 ) );

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void update_pulse_count(void)
{
    /* ULP program counts signal edges, convert that to the number of pulses */
    previous_pulse_count = (ulp_previous_count_l & UINT16_MAX) + (ulp_previous_count_h & UINT16_MAX) * 65536;
    pulse_count = (ulp_signal_count_l & UINT16_MAX) + (ulp_signal_count_h & UINT16_MAX) * 65536;
    /* get actual time */
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t time_old = (ulp_time1 & UINT16_MAX) + (ulp_time2 & UINT16_MAX) * 65536 + (ulp_time3 & UINT16_MAX) * 4294967296 + (ulp_time4 & UINT16_MAX) * 4294967296 * 65536;
    time_difference_ms = ( time_us - time_old ) / 1000;
    ulp_time1 = time_us % 65536;
    ulp_time2 = (int64_t)( time_us / 65536 ) % 65536;
    ulp_time3 = (int64_t)( time_us / 4294967296 ) % 65536;
    ulp_time4 = (int64_t)( time_us / ( 4294967296 * 65536 ) );

    printf("Previous pulse count from ULP: %5d\n", previous_pulse_count);
    printf("Pulse count from ULP: %5d\n", pulse_count);
    printf("Time difference: %.3f\n", (float)(time_difference_ms) / 1000);
    /* Save current pulse count as previous to ULP */
    ulp_previous_count_l = pulse_count % 65536;
    ulp_previous_count_h = pulse_count / 65536;
}

static void enter_deep_sleep()
{
    ESP_LOGW(TAG, "Turning off WiFi\n");

    ESP_ERROR_CHECK(esp_wifi_stop());

    struct timeval tv;

    gettimeofday(&tv, NULL);
    tv.tv_sec -= app_start.tv_sec;
    tv.tv_usec -= app_start.tv_usec;
    if (tv.tv_usec < 0)
    {
        tv.tv_usec += 1000000;
        --tv.tv_sec;
    }

    const int wakeup_time_sec = 58;
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup((wakeup_time_sec) * 1000000);

    ESP_LOGW(TAG, "Entering deep sleep after %ld.%06ld seconds\n\n", tv.tv_sec, tv.tv_usec);
    // Allow prints to finish
    fflush(stdout);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
//    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    esp_deep_sleep_start();
}
