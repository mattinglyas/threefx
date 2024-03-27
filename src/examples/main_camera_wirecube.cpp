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
#define DRAW_REFRESH_MS 100
#define COLOR RED

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
Arduino_GFX *output_display;

ThreeFX_Fixed *tfx;

Line3d cube[12] = 
{
    Line3d(Coord3d(-0.5, -0.5, -0.5), Coord3d(-0.5, -0.5, 0.5), COLOR),
    Line3d(Coord3d(-0.5, -0.5, -0.5), Coord3d(-0.5, 0.5, -0.5), COLOR),
    Line3d(Coord3d(-0.5, -0.5, -0.5), Coord3d(0.5, -0.5, -0.5), COLOR),
    Line3d(Coord3d(0.5, -0.5, 0.5), Coord3d(0.5, -0.5, -0.5), COLOR),
    Line3d(Coord3d(0.5, -0.5, 0.5), Coord3d(0.5, 0.5, 0.5), COLOR),
    Line3d(Coord3d(0.5, -0.5, 0.5), Coord3d(-0.5, -0.5, 0.5), COLOR),
    Line3d(Coord3d(-0.5, 0.5, 0.5), Coord3d(-0.5, 0.5, -0.5), COLOR),
    Line3d(Coord3d(-0.5, 0.5, 0.5), Coord3d(-0.5, -0.5, 0.5), COLOR),
    Line3d(Coord3d(-0.5, 0.5, 0.5), Coord3d(0.5, 0.5, 0.5), COLOR),
    Line3d(Coord3d(0.5, 0.5, -0.5), Coord3d(0.5, 0.5, 0.5), COLOR),
    Line3d(Coord3d(0.5, 0.5, -0.5), Coord3d(0.5, -0.5, -0.5), COLOR),
    Line3d(Coord3d(0.5, 0.5, -0.5), Coord3d(-0.5, 0.5, -0.5), COLOR),
};

void drawCube(const uint16_t color)
{
    for (int i = 0; i < 12; i++)
    {
        cube[i].color = color;
        tfx->drawWorldLine3d(cube[i]);
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
    tfx->setCameraOrigin(Coord3d(0., CAM_HEIGHT, CAM_DIST));
    LOG_INFO("done");
}

void loop()
{
    TickType_t x_last_wake_time;
    const TickType_t x_period pdMS_TO_TICKS(DRAW_REFRESH_MS);

    int16_t angle = 0;

    x_last_wake_time = xTaskGetTickCount();
    drawCube(COLOR);

    for (;;)
    {
        Fix16 rads = Fix16(angle) * Fix16(2. * PI * 1. / 360);
        Coord3d cam;
        cam.x = rads.sin() * CAM_DIST;
        cam.y = CAM_HEIGHT;
        cam.z = rads.cos() * CAM_DIST;
        
        drawCube(BLACK);
        tfx->setCameraOrigin(cam);
        tfx->setCameraAngle(0., rads, 0.);
        drawCube(COLOR);

        angle = (angle + 1) % 360;
        
        // wait for next frame
        vTaskDelayUntil(&x_last_wake_time, x_period);
    }
}