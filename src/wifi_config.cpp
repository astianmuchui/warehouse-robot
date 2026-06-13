#include "defines.h"

#ifdef ENABLE_NETWIZARD

#include <Arduino.h>
#include <WiFi.h>
#include <NetWizard.h>

/*
 * Triple-reset to wipe WiFi credentials. RTC_DATA_ATTR survives a warm reset but
 * not a power cycle, so the counter only climbs on quick repeated resets. Three
 * before a task clears it -> erase saved creds and open the captive portal.
 */

RTC_DATA_ATTR static uint8_t s_reset_count = 0;

static NETWIZARD_WEBSERVER s_nw_server(80);
static NetWizard NW(&s_nw_server);

/** clear_reset_count_task - zero the counter once the device has stayed up. */
static void clear_reset_count_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(RESET_WINDOW_MS));
    s_reset_count = 0;
    Serial.println("[WiFi] Stable boot — reset counter cleared");
    vTaskDelete(nullptr);
}

void wifi_begin()
{
    s_reset_count++;
    Serial.printf("[WiFi] Boot reset count: %u\n", s_reset_count);

    xTaskCreate(clear_reset_count_task, "rst_clr", 1024, nullptr, 1, nullptr);

    if (s_reset_count >= 3)
    {
        Serial.println("[WiFi] Triple-reset detected — clearing saved credentials");
        s_reset_count = 0;
        NW.reset();
    }

    Serial.printf("[WiFi] NetWizard autoConnect (AP: \"%s\")\n", NW_AP_SSID);

    /* Blocking: connect with stored creds, or run the portal until configured. */
    NW.autoConnect(NW_AP_SSID, NW_AP_PASSWORD);

    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("[WiFi] Connected — IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
        Serial.println("[WiFi] WARNING: not connected after portal");
}

#endif /* ENABLE_NETWIZARD */
