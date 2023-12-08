#ifndef _3d_fixed_h
#define _3d_fixed_h

#include <stdint.h>
#include <stdbool.h>
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

// storage for clipping line segments (see drawLineOnScreen)
#define CL_INSIDE (0)
#define CL_LEFT (1 << 0)
#define CL_RIGHT (1 << 1)
#define CL_BOTTOM (1 << 2)
#define CL_TOP (1 << 3)

#define THREEFX_Z_NEAR_DEFAULT (fix16_from_int(0))
#define THREEFX_Z_FAR_DEFAULT (fix16_from_int(1))
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
    Point3d(const Fix16&, const Fix16&, const Fix16&);
    Point3d(int16_t, int16_t, int16_t);

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
    Point2d(const Fix16&, const Fix16&);
    Point2d(int16_t, int16_t);

    void debugPrint() const;
};

class Line3d
{
public:
    Point3d
        p1,
        p2;

    Line3d();
    Line3d(const Line3d &);
    const Line3d &operator=(const Line3d &);
    ~Line3d();
    Line3d(const Point3d&, const Point3d&);

    void debugPrint();
};

class Line2d
{
    public:
        Point2d
            p1,
            p2;
        
        Line2d();
        Line2d(const Line2d &);
        const Line2d &operator=(const Line2d &);
        ~Line2d();
        Line2d(const Point2d&, const Point2d&);
    
        void debugPrint();
};

class ThreeFX
{
public:
    ThreeFX();
    ThreeFX(Arduino_GFX *gfx);
    ThreeFX(Arduino_GFX *, double, double, double);
    ThreeFX(Arduino_GFX *, double, const Fix16& , const Fix16&);

    void setZNear(double);
    void setZNear(Fix16);
    void setZFar(double);
    void setZFar(Fix16);
    void setGFX(Arduino_GFX *);
    void setFovAngle(double);

    inline Point3d worldToScreen(const Point3d &);
    inline Point2d screenToPixel(const Point3d &);
    inline Point2d worldToPixel(const Point3d &);
    bool drawPoint3d(const Point3d &, const uint16_t);
    bool drawLine3d(const Line3d &, const uint16_t);

protected:
    Arduino_GFX *gfx;

    bool cohenSutherlandClip(Point2d&, Point2d&);

private:
    Fix16
        fov_angle,
        a,
        z_near,
        z_far,
        fov,
        lambda,
        gfx_width,
        gfx_height;

    void updateLambda();
    void updateAspect();
    inline unsigned char cohenSutherlandCode(const Point2d &);
};

#endif