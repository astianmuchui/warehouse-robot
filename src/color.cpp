#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "defines.h"

/*  TCS34725 colour sensor 
 *  Optional sensor.  The whole module is written so the firmware behaves
 *  identically whether or not the sensor is physically present:
 *
 *    - InitializeColorSensor() probes the I2C bus once at boot and latches the
 *      result in s_enabled.
 *    - is_color_sensor_enabled() lets any caller skip colour logic cleanly.
 *    - ReadColor() returns a struct with valid=false when the sensor is absent,
 *      so the publish/serial paths never read garbage.
 *
 *  Future plan: pick red objects.  The dominant-colour classifier already
 *  reports COLOR_RED, so that logic can hang off ReadColor().dominant later.
 *  */

static Adafruit_TCS34725 s_tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
static bool s_enabled = false;

void InitializeColorSensor()
{
    /* Wire.begin() is already called by the IMU init; calling it again is
       harmless but we guard the probe so a missing sensor can't hang the bus. */
    if (s_tcs.begin(TCS34725_ADDR, &Wire))
    {
        s_enabled = true;
        Serial.println("[Color] TCS34725 OK");
    }
    else
    {
        s_enabled = false;
        Serial.println("[Color] TCS34725 not found — colour features disabled");
    }
}

bool is_color_sensor_enabled()
{
    return s_enabled;
}

/* Classify the dominant colour from normalised RGB.  Deliberately simple and
   threshold-based so it stays predictable on a warehouse line; tune as needed. */
static color_name_t classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (c < 80)                       return COLOR_BLACK;   /* almost no light back */

    /* Normalise to the clear channel to cancel out ambient brightness. */
    float rn = (float)r / (float)c;
    float gn = (float)g / (float)c;
    float bn = (float)b / (float)c;

    /* Near-equal channels with high clear → white/grey. */
    if (rn > 0.28f && gn > 0.28f && bn > 0.28f &&
        fabsf(rn - gn) < 0.10f && fabsf(gn - bn) < 0.10f)
        return COLOR_WHITE;

    if (rn > gn && rn > bn)
    {
        /* Red vs yellow: yellow has a strong green component too. */
        if (gn > 0.30f && bn < 0.25f) return COLOR_YELLOW;
        return COLOR_RED;
    }
    if (gn > rn && gn > bn)            return COLOR_GREEN;
    if (bn > rn && bn > gn)            return COLOR_BLUE;

    return COLOR_UNKNOWN;
}

color_data_t ReadColor()
{
    color_data_t out = {};
    out.valid    = false;
    out.dominant = COLOR_UNKNOWN;

    if (!s_enabled)
        return out;   /* sensor absent — caller checks .valid */

    uint16_t r, g, b, c;
    s_tcs.getRawData(&r, &g, &b, &c);

    out.r = r; out.g = g; out.b = b; out.c = c;
    out.color_temp = s_tcs.calculateColorTemperature_dn40(r, g, b, c);
    out.lux        = s_tcs.calculateLux(r, g, b);
    out.dominant   = classify(r, g, b, c);
    out.valid      = true;
    return out;
}

const char *ColorName(color_name_t color)
{
    switch (color)
    {
    case COLOR_RED:    return "red";
    case COLOR_GREEN:  return "green";
    case COLOR_BLUE:   return "blue";
    case COLOR_YELLOW: return "yellow";
    case COLOR_WHITE:  return "white";
    case COLOR_BLACK:  return "black";
    default:           return "unknown";
    }
}
