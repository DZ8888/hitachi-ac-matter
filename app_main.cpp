#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_matter.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "taiseia.h"
#include "http_ota.h"

using namespace chip;
using namespace chip::DeviceLayer;

static const char *TAG = "AC_MATTER";
static uint16_t g_therm_ep = 0;
static uint16_t g_fan_ep   = 0;

// Flag to prevent OTA from running more than once per boot
static bool g_ota_started = false;

#define RESET_BUTTON_GPIO    GPIO_NUM_0
#define RESET_HOLD_MS        10000
#define CLUSTER_THERMOSTAT   0x0201
#define ATTR_LOCAL_TEMP      0x0000
#define ATTR_SYSTEM_MODE     0x001C
#define ATTR_COOL_SP         0x0011
#define ATTR_HEAT_SP         0x0012
#define CLUSTER_FAN          0x0202
#define ATTR_PCT_SETTING     0x0002
#define ATTR_PCT_CURRENT     0x0003
#define ATTR_ROCK_SETTING    0x0006

static esp_err_t apply_system_mode(uint8_t m) {
    switch (m) {
        case 0: return taiseia_set_power(false);
        case 1: return taiseia_set_mode(TAISEIA_MODE_AUTO);
        case 3: return taiseia_set_mode(TAISEIA_MODE_COOL);
        case 4: return taiseia_set_mode(TAISEIA_MODE_HEAT);
        case 7: return taiseia_set_mode(TAISEIA_MODE_FAN_ONLY);
        case 8: return taiseia_set_mode(TAISEIA_MODE_DRY);
        default: return ESP_OK;
    }
}

static uint8_t pct_to_speed(uint8_t p) {
    if (p==0)   return TAISEIA_FAN_AUTO;
    if (p<=20)  return TAISEIA_FAN_QUIET;
    if (p<=40)  return TAISEIA_FAN_LOW;
    if (p<=60)  return TAISEIA_FAN_MEDIUM;
    if (p<=80)  return TAISEIA_FAN_HIGH;
    return TAISEIA_FAN_FOCUS;
}

static uint8_t speed_to_pct(uint8_t s) {
    switch (s) {
        case TAISEIA_FAN_AUTO:    return 0;
        case TAISEIA_FAN_QUIET:   return 10;
        case TAISEIA_FAN_LOW:     return 30;
        case TAISEIA_FAN_MEDIUM:  return 50;
        case TAISEIA_FAN_HIGH:    return 70;
        case TAISEIA_FAN_FOCUS:   return 90;
        case TAISEIA_FAN_DIFFUSE: return 100;
        default:                  return 50;
    }
}

static uint8_t taiseia_to_matter_mode(uint8_t mode, bool power) {
    if (!power) return 0;
    switch (mode) {
        case TAISEIA_MODE_COOL:     return 3;
        case TAISEIA_MODE_HEAT:     return 4;
        case TAISEIA_MODE_DRY:      return 8;
        case TAISEIA_MODE_FAN_ONLY: return 7;
        default:                    return 1;
    }
}

static void update_attr(uint16_t ep, uint32_t cl, uint32_t attr, esp_matter_attr_val_t val) {
    esp_matter::attribute::update(ep, cl, attr, &val);
}

static void poll_task(void *arg) {
    ESP_LOGI(TAG, "Poll task started");
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (true) {
        taiseia_state_t s = {};
        if (taiseia_poll(&s) == ESP_OK) {
            update_attr(g_therm_ep, CLUSTER_THERMOSTAT, ATTR_LOCAL_TEMP,
                esp_matter_int16((int16_t)(s.indoor_temp * 100)));
            update_attr(g_therm_ep, CLUSTER_THERMOSTAT, ATTR_COOL_SP,
                esp_matter_int16((int16_t)(s.target_temp * 100)));
            update_attr(g_therm_ep, CLUSTER_THERMOSTAT, ATTR_HEAT_SP,
                esp_matter_int16((int16_t)(s.target_temp * 100)));
            update_attr(g_therm_ep, CLUSTER_THERMOSTAT, ATTR_SYSTEM_MODE,
                esp_matter_uint8(taiseia_to_matter_mode(s.mode, s.power)));
            uint8_t pct = speed_to_pct(s.fan_speed);
            update_attr(g_fan_ep, CLUSTER_FAN, ATTR_PCT_SETTING, esp_matter_uint8(pct));
            update_attr(g_fan_ep, CLUSTER_FAN, ATTR_PCT_CURRENT, esp_matter_uint8(pct));
            uint8_t rock = 0;
            if (s.swing_horizontal) rock |= 0x01;
            if (s.swing_vertical)   rock |= 0x02;
            update_attr(g_fan_ep, CLUSTER_FAN, ATTR_ROCK_SETTING, esp_matter_uint8(rock));
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

// OTA task — only runs after commissioning or on reboot of paired device
static void ota_task(void *arg) {
    ESP_LOGI(TAG, "OTA check task started");
    // Small delay to let Matter stack settle
    vTaskDelay(pdMS_TO_TICKS(3000));
    http_ota_check_and_update();
    vTaskDelete(NULL);
}

static void start_ota_if_needed() {
    if (g_ota_started) return;  // Only run once per boot
    g_ota_started = true;
    ESP_LOGI(TAG, "Scheduling OTA check...");
    xTaskCreate(ota_task, "ota_task", 8192, NULL, 3, NULL);
}

static void button_task(void *arg) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    ESP_LOGI(TAG, "Hold BOOT %d seconds to factory reset", RESET_HOLD_MS/1000);
    while (true) {
        if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
            int64_t t0 = esp_timer_get_time();
            while (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
                int64_t held = (esp_timer_get_time() - t0) / 1000;
                if (held % 2000 < 100)
                    ESP_LOGW(TAG, "Reset in %lld s...", (RESET_HOLD_MS-held)/1000);
                if (held >= RESET_HOLD_MS) {
                    ESP_LOGW(TAG, "Factory reset!");
                    esp_matter::factory_reset();
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t app_attribute_update_cb(
    esp_matter::attribute::callback_type_t type,
    uint16_t ep_id, uint32_t cluster_id, uint32_t attr_id,
    esp_matter_attr_val_t *val, void *priv_data)
{
    if (type != esp_matter::attribute::PRE_UPDATE) return ESP_OK;
    ESP_LOGI(TAG, "WRITE ep=%u cl=0x%04lX attr=0x%04lX", ep_id, cluster_id, attr_id);

    if (ep_id == g_therm_ep && cluster_id == CLUSTER_THERMOSTAT) {
        if (attr_id == ATTR_SYSTEM_MODE) {
            ESP_LOGI(TAG, "  SystemMode -> %u", val->val.u8);
            apply_system_mode(val->val.u8);
        } else if (attr_id == ATTR_COOL_SP || attr_id == ATTR_HEAT_SP) {
            int16_t t = val->val.i16 / 100;
            ESP_LOGI(TAG, "  SetPoint -> %d°C", t);
            taiseia_set_target_temp(t);
        }
    } else if (ep_id == g_fan_ep && cluster_id == CLUSTER_FAN) {
        if (attr_id == ATTR_PCT_SETTING) {
            uint8_t spd = pct_to_speed(val->val.u8);
            ESP_LOGI(TAG, "  Fan %u%% -> speed %u", val->val.u8, spd);
            taiseia_set_fan_speed(spd);
        } else if (attr_id == ATTR_ROCK_SETTING) {
            bool h = (val->val.u8 & 0x01) != 0;
            bool v = (val->val.u8 & 0x02) != 0;
            ESP_LOGI(TAG, "  Swing H=%s V=%s", h?"ON":"OFF", v?"ON":"OFF");
            taiseia_set_swing(h, v);
        }
    }
    return ESP_OK;
}

static esp_err_t app_identification_cb(
    esp_matter::identification::callback_type_t type, uint16_t ep_id,
    uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{ return ESP_OK; }

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        // Case 1: Fresh pairing just completed → safe to run OTA now
        ESP_LOGI(TAG, "Commissioning complete!");
        start_ota_if_needed();
        break;

    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        // Case 2: Already paired device got IP after reboot → run OTA
        // Only trigger on IPv4 (not IPv6) to avoid firing twice
        // Check fabric count > 0 to confirm device is truly commissioned
        if (event->InterfaceIpAddressChanged.Type ==
            chip::DeviceLayer::InterfaceIpChangeType::kIpV4_Assigned) {
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
                ESP_LOGI(TAG, "WiFi IPv4 ready (already commissioned) — starting OTA check");
                start_ota_if_needed();
            }
        }
        break;

    default:
        break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Hitachi AC Matter Controller v0.2 ===");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_flash_init_partition("fctry");
    if (err != ESP_OK) ESP_LOGE(TAG, "fctry init failed: %d", err);

    err = taiseia_init();
    if (err != ESP_OK) ESP_LOGE(TAG, "TaiSEIA init failed: %d", err);

    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);

    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(
        &node_config, app_attribute_update_cb, app_identification_cb);
    if (!node) { ESP_LOGE(TAG, "Failed to create node"); return; }

    {
        esp_matter::endpoint::thermostat::config_t cfg;
        cfg.thermostat.control_sequence_of_operation = 4;
        cfg.thermostat.system_mode                   = 3;
        cfg.thermostat.local_temperature             = 2600;
        cfg.thermostat.features.heating.occupied_heating_setpoint = 2000;
        cfg.thermostat.features.cooling.occupied_cooling_setpoint = 2600;
        cfg.thermostat.features.auto_mode.min_setpoint_dead_band  = 2;
        cfg.thermostat.feature_flags =
            esp_matter::cluster::thermostat::feature::heating::get_id() |
            esp_matter::cluster::thermostat::feature::cooling::get_id() |
            esp_matter::cluster::thermostat::feature::auto_mode::get_id();
        esp_matter::endpoint_t *ep = esp_matter::endpoint::thermostat::create(
            node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, NULL);
        g_therm_ep = esp_matter::endpoint::get_id(ep);
        ESP_LOGI(TAG, "Thermostat ep: %u", g_therm_ep);
    }

    {
        esp_matter::endpoint::fan::config_t cfg;
        cfg.fan_control.fan_mode_sequence = 5;
        cfg.fan_control.fan_mode          = 2;
        cfg.fan_control.percent_setting   = 50;
        cfg.fan_control.percent_current   = 50;
        esp_matter::endpoint_t *ep = esp_matter::endpoint::fan::create(
            node, &cfg, esp_matter::ENDPOINT_FLAG_NONE, NULL);
        g_fan_ep = esp_matter::endpoint::get_id(ep);
        ESP_LOGI(TAG, "Fan ep: %u", g_fan_ep);
        esp_matter::cluster_t *fan_cl = esp_matter::cluster::get(
            ep, chip::app::Clusters::FanControl::Id);
        esp_matter::cluster::fan_control::feature::rocking::config_t rock;
        rock.rock_support = 0x03;
        rock.rock_setting = 0x00;
        esp_matter::cluster::fan_control::feature::rocking::add(fan_cl, &rock);
    }

    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) ESP_LOGE(TAG, "Matter start failed: %d", err);

    // OTA is NO LONGER started here — it starts from app_event_cb instead
    xTaskCreate(poll_task, "poll_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Running — hold BOOT 10s to factory reset");
}