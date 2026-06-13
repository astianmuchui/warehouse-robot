#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "defines.h"

/*
 * TCS34725 colour sensor. Optional: the firmware behaves the same whether or not
 * it's fitted. InitializeColorSensor probes the bus once; ReadColor returns
 * valid=false when the sensor is absent so callers never read garbage.
 *
 * 154 ms integration + 16X gain: readings off the matte surface came back very
 * dark (clear ~30), which starved the classifier. More light keeps the channels
 * large enough to tell red apart.
 */
static Adafruit_TCS34725 s_tcs(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);
static bool s_enabled = false;

/** InitializeColorSensor - probe the bus and latch whether the sensor answered. */
void InitializeColorSensor()
{
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

/**
 * classify - simple threshold classifier on RGB normalised to the clear channel.
 * The black gate is low (c < 15): the old c < 80 was higher than real readings
 * off this surface and forced every read to BLACK.
 */
static color_name_t classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (c < 15) return COLOR_BLACK;

    float rn = (float)r / (float)c;
    float gn = (float)g / (float)c;
    float bn = (float)b / (float)c;

    /* Near-equal channels with high clear -> white. */
    if (rn > 0.28f && gn > 0.28f && bn > 0.28f &&
        fabsf(rn - gn) < 0.10f && fabsf(gn - bn) < 0.10f)
        return COLOR_WHITE;

    if (rn > gn && rn > bn)
    {
        if (gn > 0.30f && bn < 0.25f) return COLOR_YELLOW; /* yellow has strong green too */
        return COLOR_RED;
    }
    if (gn > rn && gn > bn) return COLOR_GREEN;
    if (bn > rn && bn > gn) return COLOR_BLUE;

    return COLOR_UNKNOWN;
}

/** ReadColor - one reading; valid=false when the sensor is absent. */
color_data_t ReadColor()
{
    color_data_t out = {};
    out.valid    = false;
    out.dominant = COLOR_UNKNOWN;

    if (!s_enabled)
        return out;

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
