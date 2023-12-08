#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "debug.h"
#include "3d_fixed.hpp"

#define TFT_CS 5    // GPIO5
#define TFT_RESET 3 // GPIO3
#define TFT_DC 4    // GPIO4
#define TFT_MOSI 10 // GPIO10/MOSI
#define TFT_SCK 8   // GPIO8/SCK
#define TFT_LED 2   // GPIO2
#define TFT_MISO -1 // not used for TFT

#define GFX_BL TFT_LED // backlight pin
#define DRAW_REFRESH_MS 500

// parameters for generating perspective grid
#define Z_PLANE_COLOR CYAN
#define NUM_Z_PLANE_VERT (17)
#define NUM_Z_PLANE_HORIZ (9)
#define XS_MIN (Fix16(-7.))
#define XS_MAX (Fix16(7.))
#define ZS_MIN (Fix16(6.))
#define ZS_MAX (Fix16(14.))
#define Y_HEIGHT (Fix16(-2.5))

// camera parameters
#define Z_NEAR (Fix16(3.))
#define Z_FAR (Fix16(50.))
#define FOV_RAD (PI / 2.)

// storage
Line3d z_grid_horiz[NUM_Z_PLANE_HORIZ];
Line3d z_grid_vert[NUM_Z_PLANE_VERT];

// ThreeFX object
Arduino_DataBus *bus; 
Arduino_GFX *gfx;
ThreeFX *tfx;

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
    // put your setup code here, to run once:
    if (DEBUG)
    {
        Serial.begin(115200);
        vTaskDelay(2500);
    }
    debug_println("Set up gfx...")
    bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
    gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 1, false);
    gfx->begin();

    debug_println("Wipe screen...");
    // light up display and wipe screen
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);

    debug_println("Init ThreeFX...");
    tfx = new ThreeFX(gfx, FOV_RAD, Z_NEAR, Z_FAR);
}

void loop()
{
    TickType_t x_last_wake_time;
    const TickType_t x_period pdMS_TO_TICKS(DRAW_REFRESH_MS);

    x_last_wake_time = xTaskGetTickCount();

    initGridWorldCoordinates();

    // draw first frame
    drawZPlaneHoriz(Z_PLANE_COLOR);
    drawZPlaneVert(Z_PLANE_COLOR);

    for (;;)
    {
        // wait for next frame
        vTaskDelayUntil(&x_last_wake_time, x_period);
        debug_print(millis());
        debug_println(" frame");
        
        // update grid
        drawZPlaneVert(BLACK);
        drawZPlaneHoriz(BLACK);
        updateGridWorldCoordinates(Fix16(-0.1), Fix16(-0.2));
        drawZPlaneVert(Z_PLANE_COLOR);
        drawZPlaneHoriz(Z_PLANE_COLOR);
    }
}
