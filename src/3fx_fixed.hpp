#ifndef _3fx_fixed_h
#define _3fx_fixed_h

#include <stdint.h>
#include <stdbool.h>
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

// storage for clipping line segments (see drawLineOnScreen)
#define CL_INSIDE (0)
#define CL_LEFT (1 << 0) // x < min
#define CL_RIGHT (1 << 1) // x > max
#define CL_BOTTOM (1 << 2) // y > max (left-handed coordinates)
#define CL_TOP (1 << 3) // y < min
#define CL_FRONT (1 << 4) // front is min, toward camera
#define CL_BACK (1 << 5) // back is max, further from camera

#define THREEFX_Z_NEAR_DEFAULT (Fix16(0.))
#define THREEFX_Z_FAR_DEFAULT (Fix16(1.))
#define THREEFX_FOV_DEFAULT (PI / 2)

#define IN_RANGE_INC(val, min, max) ((val >= min) && (val >= max))

class Point3d
{
public:
    Fix16
        x,
        y,
        z;

    Point3d();
    Point3d(const Point3d &);
    const Point3d &operator=(const Point3d &);
    ~Point3d();

    Point3d(double, double, double);
    Point3d(const Fix16 &, const Fix16 &, const Fix16 &);
    Point3d(int16_t, int16_t, int16_t);

    Point3d &operator+=(const Point3d &);
    Point3d &operator-=(const Point3d &);
    const Point3d operator+(const Point3d &);
    const Point3d operator-(const Point3d &);

    void debugPrint() const;
};

class Point2d
{
public:
    Fix16
        x,
        y;

    Point2d();
    Point2d(const Point2d &);
    const Point2d &operator=(const Point2d &);
    ~Point2d();
    Point2d(double, double);
    Point2d(const Fix16 &, const Fix16 &);
    Point2d(int16_t, int16_t);

    void debugPrint() const;
};

class Line3d
{
public:
    Point3d
        p0,
        p1;

    Line3d();
    Line3d(const Line3d &);
    const Line3d &operator=(const Line3d &);
    ~Line3d();
    Line3d(const Point3d &, const Point3d &);

    void debugPrint();
};

class Line2d
{
public:
    Point2d
        p0,
        p1;

    Line2d();
    Line2d(const Line2d &);
    const Line2d &operator=(const Line2d &);
    ~Line2d();
    Line2d(const Point2d &, const Point2d &);

    void debugPrint();
};

class ThreeFX_Fixed
{
public:
    ThreeFX_Fixed();
    ThreeFX_Fixed(Arduino_GFX *gfx);
    ThreeFX_Fixed(Arduino_GFX *, double, double, double);
    ThreeFX_Fixed(Arduino_GFX *, double, const Fix16 &, const Fix16 &);

    void setZNear(double);
    void setZNear(const Fix16 &);
    void setZFar(double);
    void setZFar(const Fix16 &);
    void setGFX(Arduino_GFX *);
    void setFovAngle(double);
    void setCameraAngle(const Fix16 &, const Fix16 &, const Fix16 &);
    void setCameraOrigin(const Point3d &);

    Point2d worldToPixel(const Point3d &);

    Fix16 getCameraYaw();
    Fix16 getCameraPitch();
    Fix16 getCameraRoll();
    Point3d getCameraOrigin();

    bool drawPoint3d(const Point3d &, const uint16_t);
    bool drawLine3d(const Line3d &, const uint16_t);

protected:
    Arduino_GFX *gfx;

    bool cohenSutherlandClip(Point2d &, Point2d &);
    bool cohenSutherlandClip(Point2d &, Point2d &, const Point2d &, const Point2d &);

    bool cohenSutherlandClip(Point3d &, Point3d &);
    bool cohenSutherlandClip(Point3d &, Point3d &, const Point3d &, const Point3d &);

private:
    Fix16
        fov_angle,
        a,
        z_near,
        z_far,
        z_scale,
        fov,
        lambda,
        gfx_width,
        gfx_height,
        // https://msl.cs.uiuc.edu/planning/node102.html
        alpha,   // alpha
        beta, // beta
        gamma;  // gamma
    Point3d
        cam_origin;

    Fix16 cam_mtrx[3][3];

    void updateLambdaScale();
    void updateAspect();
    void updateCamTransform();
    unsigned char cohenSutherlandCode(const Point2d &, const Point2d &, const Point2d &);
    unsigned char cohenSutherlandCode(const Point3d &, const Point3d &, const Point3d &);
    Point3d worldCamera(const Point3d &);
    Point3d worldToScreen(const Point3d &);
    Point2d screenToPixel(const Point3d &);
};

#endif