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

// parameters for generating perspective grid
#define Z_PLANE_COLOR CYAN
#define NUM_Z_PLANE_VERT (17)
#define NUM_Z_PLANE_HORIZ (9)
#define XS_MIN (Fix16(-14.))
#define XS_MAX (Fix16(14.))
#define ZS_MIN (Fix16(6.))
#define ZS_MAX (Fix16(14.))
#define Y_HEIGHT (Fix16(-3.))

// camera parameters
#define Z_NEAR (Fix16(3.))
#define Z_FAR (Fix16(50.))
#define FOV_RAD (PI / 2.)

// storage
Line3d z_grid_horiz[NUM_Z_PLANE_HORIZ];
Line3d z_grid_vert[NUM_Z_PLANE_VERT];

// ThreeFX_Fixed object
Arduino_DataBus *bus; 
Arduino_GFX *gfx;
ThreeFX_Fixed *tfx;

// init grid by linspace
void initGridWorldCoordinates()
{
    int32_t delta;
    // horizontals
    delta = (ZS_MAX - ZS_MIN) / int16_t{NUM_Z_PLANE_HORIZ - 1};
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        z_grid_horiz[i].p1.x = XS_MIN;
        z_grid_horiz[i].p1.y = Y_HEIGHT;
        z_grid_horiz[i].p1.z = (ZS_MIN + delta * i);

        z_grid_horiz[i].p2.x = XS_MAX;
        z_grid_horiz[i].p2.y = Y_HEIGHT;
        z_grid_horiz[i].p2.z = (ZS_MIN + delta * i);
    }

    // vertical
    delta = (XS_MAX - XS_MIN) / int16_t{NUM_Z_PLANE_VERT - 1};
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        z_grid_vert[i].p1.x = (XS_MIN + delta * i);
        z_grid_vert[i].p1.y = Y_HEIGHT;
        z_grid_vert[i].p1.z = ZS_MIN;

        z_grid_vert[i].p2.x = (XS_MIN + delta * i);
        z_grid_vert[i].p2.y = Y_HEIGHT;
        z_grid_vert[i].p2.z = ZS_MAX;
    }
}

// moves the grid by dx and dz and scrolls the grid lines if they underflow or
// overflow (does not handle multiple overflows)
void updateGridWorldCoordinates(const Fix16 &dx, const Fix16 &dz)
{
    Fix16 temp = Fix16();
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        temp = z_grid_horiz[i].p1.z + dz;

        if (temp > ZS_MAX)
        {
            temp -= (ZS_MAX - ZS_MIN);
        }
        else if (temp < ZS_MIN)
        {
            temp += (ZS_MAX - ZS_MIN);
        }

        z_grid_horiz[i].p1.z = temp;
        z_grid_horiz[i].p2.z = temp;
    }

    // vertical
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        temp = z_grid_vert[i].p1.x + dx;

        if (temp > XS_MAX)
        {
            temp -= (XS_MAX - XS_MIN);
        }
        else if (temp < XS_MIN)
        {
            temp += (XS_MAX - XS_MIN);
        }

        z_grid_vert[i].p1.x = temp;
        z_grid_vert[i].p2.x = temp;
    }
}

void drawZPlaneVert(int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        tfx->drawLine3d(z_grid_vert[i], color);
    }
}

void drawZPlaneHoriz(int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        tfx->drawLine3d(z_grid_horiz[i], color);
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
    LOG_INFO("done");
}

void loop()
{
    TickType_t x_last_wake_time;
    const TickType_t x_period pdMS_TO_TICKS(DRAW_REFRESH_MS);

    int16_t angle = 0;
    Fix16 speed(0.1);

    x_last_wake_time = xTaskGetTickCount();

    initGridWorldCoordinates();

    // draw first frame
    drawZPlaneHoriz(Z_PLANE_COLOR);
    drawZPlaneVert(Z_PLANE_COLOR);

    for (;;)
    {
        // wait for next frame
        vTaskDelayUntil(&x_last_wake_time, x_period);
        
        Fix16 rads = Fix16(angle) * Fix16(2. * PI * 1. / 360.);
        Fix16 dx, dy;
        dx = rads.sin() * speed;
        dy = rads.cos() * speed;
        angle = (angle + 1) % 360;

        // update grid
        drawZPlaneVert(BLACK);
        drawZPlaneHoriz(BLACK);
        updateGridWorldCoordinates(dx, dy);
        drawZPlaneVert(Z_PLANE_COLOR);
        drawZPlaneHoriz(Z_PLANE_COLOR);
    }
}
