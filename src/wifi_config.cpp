#include "defines.h"

#ifdef ENABLE_NETWIZARD

#include <Arduino.h>
#include <WiFi.h>
#include <NetWizard.h>

/*
 * Triple-reset detection
 * ----------------------
 * RTC_DATA_ATTR survives warm resets (EN / reset button) but is cleared by
 * a power cycle.  On each boot we increment the counter.  A FreeRTOS task
 * clears it after RESET_WINDOW_MS of stable uptime.  If the counter reaches
 * 3 before it is cleared, we wipe the stored credentials and force the
 * captive-portal so the user can reconfigure WiFi.
 */

RTC_DATA_ATTR static uint8_t s_reset_count = 0;

static NETWIZARD_WEBSERVER s_nw_server(80);
static NetWizard NW(&s_nw_server);

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

    /* Spawn a task that zeroes the counter once the device stays up long
     * enough.  If the user resets again before it fires, the count survives
     * into the next boot via RTC memory. */
    xTaskCreate(clear_reset_count_task, "rst_clr", 1024, nullptr, 1, nullptr);

    if (s_reset_count >= 3)
    {
        Serial.println("[WiFi] Triple-reset detected — clearing saved credentials");
        s_reset_count = 0;
        NW.reset();  /* erase stored SSID / password from NVS */
    }

    Serial.printf("[WiFi] NetWizard autoConnect (AP: \"%s\")\n", NW_AP_SSID);

    /* Blocking: connects using stored credentials, or starts the captive
     * portal hotspot named NW_AP_SSID until the user configures WiFi. */
    NW.autoConnect(NW_AP_SSID, NW_AP_PASSWORD);

    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("[WiFi] Connected — IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
        Serial.println("[WiFi] WARNING: not connected after portal");
}

#endif /* ENABLE_NETWIZARD */
