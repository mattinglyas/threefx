#include <Arduino.h>
#include <DebugLog.h>
#include <Arduino_GFX_Library.h>
#include "ThreeFX.hpp"

#define TFT_CS 8    // GPIO5
#define TFT_RESET 17 // GPIO3
#define TFT_DC 9    // GPIO4
#define TFT_MOSI 35 // GPIO10/MOSI
#define TFT_SCK 36   // GPIO8/SCK
#define TFT_LED 18   // GPIO2
#define TFT_MISO -1 // not used for TFT

#define GFX_BL TFT_LED // backlight pin
#define DRAW_REFRESH_MS 250
#define COLOR RED

// camera parameters
#define Z_NEAR (Fix16(1.))
#define Z_FAR (Fix16(50.))
#define FOV_RAD (PI / 4.)

// visualization parameters
#define CAM_DIST (-5.)
#define CAM_HEIGHT (0.)

// ThreeFX_Fixed object
Arduino_DataBus *bus; 
Arduino_GFX *gfx;
Arduino_GFX *output_display;

ThreeFX_Fixed *tfx;

Coord3d tp[4] =
{
    Coord3d(Fix16(1.), Fix16(-0.5774), Fix16(-0.4082)),
    Coord3d(Fix16(-1.), Fix16(-0.5774), Fix16(-0.4082)),
    Coord3d(Fix16(0.), Fix16(1.155), Fix16(-0.4082)),
    Coord3d(Fix16(0.), Fix16(0.), Fix16(1.2247))
};

Facet3d tetra[4] = 
{
    Facet3d(tp[3], tp[0], tp[1], RED),
    Facet3d(tp[3], tp[1], tp[2], BLUE),
    Facet3d(tp[3], tp[2], tp[0], GREEN),
    Facet3d(tp[0], tp[1], tp[2], YELLOW)
};

void setup()
{
    Serial.begin(115200);
    vTaskDelay(2500);
    LOG_ATTACH_SERIAL(Serial);

    LOG_INFO("Set up gfx...");
    bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
    gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 1, false);
    gfx->begin();
    LOG_INFO("done");

    LOG_INFO("Wipe screen...");
    // light up display and wipe screen
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);
    LOG_INFO("done");

    LOG_INFO("Init ThreeFX_Fixed...");
    tfx = new ThreeFX_Fixed(gfx, FOV_RAD, Z_NEAR, Z_FAR);
    tfx->setCameraOrigin(Coord3d(0., CAM_HEIGHT, CAM_DIST));
    LOG_INFO("done");
}

void loop()
{
    TickType_t x_last_wake_time;
    const TickType_t x_period pdMS_TO_TICKS(DRAW_REFRESH_MS);

    int16_t angle = 0;

    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        Fix16 rads = Fix16(angle) * Fix16(2. * PI * 1. / 360);
        Coord3d cam;
        cam.x = rads.sin() * CAM_DIST;
        cam.y = CAM_HEIGHT;
        cam.z = rads.cos() * CAM_DIST;
        
        tfx->setCameraOrigin(cam);
        tfx->setCameraAngle(0., rads, 0.);
        angle = (angle + 5) % 360;

        gfx->fillScreen(BLACK);
        for (int i = 0; i < 4; i++)
            tfx->queueDrawable(&tetra[i]);
        tfx->flushDrawQueue();

        vTaskDelayUntil(&x_last_wake_time, x_period);
    }
}