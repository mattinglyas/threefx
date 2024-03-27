#ifndef _ThreeFX_hpp
#define _ThreeFX_hpp

#include <stdint.h>
#include <stdbool.h>
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>
#include <memory>
#include <queue>
#include <DebugLog.h>

// storage for clipping line segments (see drawLineOnScreen)
#define CL_INSIDE (0)
#define CL_LEFT (1 << 0)   // x < min
#define CL_RIGHT (1 << 1)  // x > max
#define CL_BOTTOM (1 << 2) // y > max (left-handed coordinates)
#define CL_TOP (1 << 3)    // y < min
#define CL_FRONT (1 << 4)  // front is min, toward camera
#define CL_BACK (1 << 5)   // back is max, further from camera

#define DEFAULT_COLOR (MAGENTA)

#define THREEFX_Z_NEAR_DEFAULT (Fix16(0.))
#define THREEFX_Z_FAR_DEFAULT (Fix16(1.))
#define THREEFX_FOV_DEFAULT (PI / 2)

#define IN_RANGE_INC(val, min, max) ((val >= min) && (val <= max))
#define SCREEN_COORD_ON_SCREEN(sc) (IN_RANGE_INC(sc.x, Fix16(-1.), Fix16(1.)) && IN_RANGE_INC(sc.y, Fix16(-1.), Fix16(1.)))

class ThreeFX_Fixed;

class Coord3d
{
public:
    Fix16
        x,
        y,
        z;

    Coord3d();
    Coord3d(const Coord3d &);
    const Coord3d &operator=(const Coord3d &);
    ~Coord3d();

    Coord3d(double, double, double);
    Coord3d(const Fix16 &, const Fix16 &, const Fix16 &);
    Coord3d(int16_t, int16_t, int16_t);

    Coord3d &operator+=(const Coord3d &);
    Coord3d &operator-=(const Coord3d &);
    const Coord3d operator+(const Coord3d &);
    const Coord3d operator-(const Coord3d &);

    void debugPrint() const;
};

class Coord2d
{
public:
    Fix16
        x,
        y;

    void setX(const Fix16 &);
    void setY(const Fix16 &);

    Coord2d();
    Coord2d(const Coord2d &);
    const Coord2d &operator=(const Coord2d &);
    ~Coord2d();
    Coord2d(double, double);
    Coord2d(const Fix16 &, const Fix16 &);
    Coord2d(int16_t, int16_t);

    void debugPrint() const;
};

/**
 * @brief A Drawable3d is an abstract for any object that can be drawn
 * (after 3d-2d projection) to the screen. Examples include facets, lines,
 * points. The ThreeFX object contains a queue of Drawable3d objects to
 * commit to the screen.
 */
class Drawable3d
{
public:
    uint16_t color;
    virtual Coord3d getBarycenter() const;
    virtual void debugPrint() const;
    virtual bool draw(ThreeFX_Fixed &) const;
    // virtual std::unique_ptr<Drawable3d> clone() const;
};

class Point3d : public Drawable3d
{
public:
    Coord3d p0;
    Coord3d getBarycenter() const;
    void debugPrint() const;
    bool draw(ThreeFX_Fixed &) const;
    
    Point3d();
    Point3d(const Point3d &);
    const Point3d &operator=(const Point3d &);
    ~Point3d();
    Point3d(const Coord3d &);
    Point3d(const Coord3d &, const uint16_t);
    Point3d(const Fix16 &, const Fix16 &, const Fix16 &);
    Point3d(const Fix16 &, const Fix16 &, const Fix16 &, const uint16_t);
};

class Line3d : public Drawable3d
{
public:
    Coord3d p0, p1;

    Coord3d getBarycenter() const;
    void debugPrint() const;
    bool draw(ThreeFX_Fixed &) const;
    
    Coord3d getP0() const;
    Coord3d getP1() const;
    void setP0(const Coord3d &);
    void setP1(const Coord3d &);

    Line3d();
    Line3d(const Line3d &);
    const Line3d &operator=(const Line3d &);
    ~Line3d();
    Line3d(const Coord3d &, const Coord3d &);
    Line3d(const Coord3d &, const Coord3d &, const uint16_t);
};

class Facet3d : public Drawable3d
{
public:
    Coord3d p0, p1, p2;

    Coord3d getBarycenter() const;    
    void debugPrint() const;
    bool draw(ThreeFX_Fixed &) const;
    
    Coord3d getP0() const;
    Coord3d getP1() const;
    Coord3d getP2() const;
    void setP0(const Coord3d &);
    void setP1(const Coord3d &);
    void setP2(const Coord3d &);

    Facet3d();
    Facet3d(const Facet3d &);
    const Facet3d &operator=(const Facet3d &);
    ~Facet3d();
    Facet3d(const Coord3d &, const Coord3d &, const Coord3d &);
    Facet3d(const Coord3d &, const Coord3d &, const Coord3d &, const uint16_t);
};

class Model3d
{
private:
    Coord3d origin;
    std::unique_ptr<Facet3d> *facets;
    int16_t facet_cnt;

public:
    Model3d();
    Model3d(const Model3d &);
    const Model3d &operator=(const Model3d &);
    ~Model3d();
    void addFacet(const Facet3d *, uint16_t);
    void addFacet(const Facet3d *);
    void clear();
};

typedef std::pair<const Drawable3d *, Coord3d> Queued3d;
class Q3dCompare 
{
public:
    bool operator()(Queued3d &a, Queued3d &b);
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
    void setCameraOrigin(const Coord3d &);

    Coord2d worldToPixel(const Coord3d &);

    Fix16 getCameraYaw();
    Fix16 getCameraPitch();
    Fix16 getCameraRoll();
    Coord3d getCameraOrigin();

    bool drawWorldPoint3d(const Point3d &);
    bool drawWorldLine3d(const Line3d &);
    bool drawWorldFacet3d(const Facet3d &);
    bool drawScreenPoint3d(const Point3d &);
    bool drawScreenLine3d(const Line3d &);
    bool drawScreenFacet3d(const Facet3d &);

    // new draw queue
    int queueDrawable(const Drawable3d *);
    int flushDrawQueue();
    void clearDrawQueue();

    Coord3d worldToScreen(const Coord3d &);
    Coord2d screenToPixel(const Coord3d &);
protected:
    Arduino_GFX *gfx;

    bool cohenSutherlandClip(Coord2d &, Coord2d &);
    bool cohenSutherlandClip(Coord2d &, Coord2d &, const Coord2d &, const Coord2d &);

    bool cohenSutherlandClip(Coord3d &, Coord3d &);
    bool cohenSutherlandClip(Coord3d &, Coord3d &, const Coord3d &, const Coord3d &);

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
        alpha, // alpha
        beta,  // beta
        gamma; // gamma
    Coord3d
        cam_origin;

    Fix16 cam_mtrx[3][3];

    std::priority_queue<Queued3d, std::vector<Queued3d>, Q3dCompare> *draw_queue;

    void updateLambdaScale();
    void updateAspect();
    void updateCamTransform();
    unsigned char cohenSutherlandCode(const Coord2d &, const Coord2d &, const Coord2d &);
    unsigned char cohenSutherlandCode(const Coord3d &, const Coord3d &, const Coord3d &);
    Coord3d worldCamera(const Coord3d &);
};

#endif