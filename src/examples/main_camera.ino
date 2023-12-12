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
#define DRAW_REFRESH_MS 50

// camera parameters
#define Z_NEAR (Fix16(3.))
#define Z_FAR (Fix16(50.))
#define FOV_RAD (PI / 2.)

// ThreeFX_Fixed object
Arduino_DataBus *bus; 
Arduino_GFX *gfx;
ThreeFX_Fixed *tfx;

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
    LOG_INFO("done");
}

void loop()
{

}