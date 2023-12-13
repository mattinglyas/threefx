#include <Arduino.h>
#include <DebugLog.h>
#include <Arduino_GFX_Library.h>
#include "3fx_fixed.hpp"

#define TFT_CS 5    // GPIO5
#define TFT_RESET 3 // GPIO3
#define TFT_DC 4    // GPIO4
#define TFT_MOSI 10 // GPIO10/MOSI
#define TFT_SCK 8   // GPIO8/SCK
#define TFT_LED 2   // GPIO2
#define TFT_MISO -1 // not used for TFT

#define GFX_BL TFT_LED // backlight pin
#define DRAW_REFRESH_MS 75

// camera parameters
#define Z_NEAR (Fix16(1.))
#define Z_FAR (Fix16(50.))
#define FOV_RAD (PI / 2.)

// visualization parameters
#define CAM_DIST (-2.)
#define CAM_HEIGHT (0.)

// ThreeFX_Fixed object
Arduino_DataBus *bus; 
Arduino_GFX *gfx;
ThreeFX_Fixed *tfx;

Line3d cube[12] = 
{
    Line3d(Point3d(-0.5, -0.5, -0.5), Point3d(-0.5, -0.5, 0.5)),
    Line3d(Point3d(-0.5, -0.5, -0.5), Point3d(-0.5, 0.5, -0.5)),
    Line3d(Point3d(-0.5, -0.5, -0.5), Point3d(0.5, -0.5, -0.5)),
    Line3d(Point3d(0.5, -0.5, 0.5), Point3d(0.5, -0.5, -0.5)),
    Line3d(Point3d(0.5, -0.5, 0.5), Point3d(0.5, 0.5, 0.5)),
    Line3d(Point3d(0.5, -0.5, 0.5), Point3d(-0.5, -0.5, 0.5)),
    Line3d(Point3d(-0.5, 0.5, 0.5), Point3d(-0.5, 0.5, -0.5)),
    Line3d(Point3d(-0.5, 0.5, 0.5), Point3d(-0.5, -0.5, 0.5)),
    Line3d(Point3d(-0.5, 0.5, 0.5), Point3d(0.5, 0.5, 0.5)),
    Line3d(Point3d(0.5, 0.5, -0.5), Point3d(0.5, 0.5, 0.5)),
    Line3d(Point3d(0.5, 0.5, -0.5), Point3d(0.5, -0.5, -0.5)),
    Line3d(Point3d(0.5, 0.5, -0.5), Point3d(-0.5, 0.5, -0.5)),
};

void drawCube(uint16_t color)
{
    for (int i = 0; i < 12; i++)
    {
        tfx->drawLine3d(cube[i], color);
    }
}

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
    tfx->setCameraOrigin(Point3d(0., CAM_HEIGHT, CAM_DIST));
    LOG_INFO("done");
}

void loop()
{
    TickType_t x_last_wake_time;
    const TickType_t x_period pdMS_TO_TICKS(DRAW_REFRESH_MS);

    int16_t angle = 0;

    x_last_wake_time = xTaskGetTickCount();
    drawCube(RED);

    for (;;)
    {
        // wait for next frame
        vTaskDelayUntil(&x_last_wake_time, x_period);

        Fix16 rads = Fix16(angle) * Fix16(2. * PI * 1. / 360);
        Point3d cam;
        cam.x = rads.sin() * CAM_DIST;
        cam.y = CAM_HEIGHT;
        cam.z = rads.cos() * CAM_DIST;
        
        drawCube(BLACK);
        tfx->setCameraOrigin(cam);
        tfx->setCameraAngle(0., rads, 0.);
        drawCube(RED);

        angle = (angle + 1) % 360;
    }
}