#include <DebugLog.h>
#include "3fx_fixed.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 */
Point3d::Point3d()
{
    this->x = Fix16();
    this->y = Fix16();
    this->z = Fix16();
}

double function(const double& x)
{
    return 0.;
}

/**
 * @brief Copy constructor for a new Point 3d:: Point 3d object
 *
 * @param p
 */
Point3d::Point3d(const Point3d &p)
{
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;
}

/**
 * @brief Assignment operator
 *
 * @param p other Point3d
 * @return Point3d&
 */
const Point3d &Point3d::operator=(const Point3d &p)
{
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;

    return p;
}

/**
 * @brief Destroy the Point 3d:: Point 3d object
 *
 */
Point3d::~Point3d() {}

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 * @param x x coordinate (float)
 * @param y y coordinate (float)
 * @param z z coordinate (float)
 */
Point3d::Point3d(double x, double y, double z)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
    this->z = Fix16(z);
}

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 * @param x x coordinate (fix16)
 * @param y y coordinate (fix16)
 * @param z z coordinate (fix16)
 */
Point3d::Point3d(const Fix16 &x, const Fix16 &y, const Fix16 &z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 * @param x x coordinate (int16_t)
 * @param y y coordinate (int16_t)
 * @param z z coordinate (int16_t)
 */
Point3d::Point3d(int16_t x, int16_t y, int16_t z)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
    this->z = Fix16(z);
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Point3d::debugPrint() const
{
    char buf[13];
    PRINT("(");
    fix16_to_str(this->x, buf, 4);
    PRINT(buf);
    PRINT(", ");
    fix16_to_str(this->y, buf, 4);
    PRINT(buf);
    PRINT(", ");
    fix16_to_str(this->z, buf, 4);
    PRINT(buf);
    PRINT(")");
}

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 */
Point2d::Point2d()
{
    this->x = Fix16();
    this->y = Fix16();
}

/**
 * @brief Copy construct a new Point 2d:: Point 2d object
 *
 * @param p other point
 */
Point2d::Point2d(const Point2d &p)
{
    this->x = p.x;
    this->y = p.y;
}

/**
 * @brief Assignment operator for Point2d
 *
 * @param p other point
 * @return const Point2d& p
 */
const Point2d &Point2d::operator=(const Point2d &p)
{
    this->x = p.x;
    this->y = p.y;

    return p;
}

/**
 * @brief Destructor for Point2d
 *
 */
Point2d::~Point2d() {}

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 * @param x x coordinate (double)
 * @param y y coordinate (double)
 */
Point2d::Point2d(double x, double y)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
}

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 * @param x x coordinate (fix16)
 * @param y y coordinate (fix16)
 */
Point2d::Point2d(const Fix16 &x, const Fix16 &y)
{
    this->x = x;
    this->y = y;
}

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 * @param x x coordinate (int)
 * @param y y coordinate (int)
 */
Point2d::Point2d(int16_t x, int16_t y)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Point2d::debugPrint() const
{
    char buf[13];
    PRINT("(");
    fix16_to_str(this->x, buf, 4);
    PRINT(buf);
    PRINT(", ");
    fix16_to_str(this->y, buf, 4);
    PRINT(buf);
    PRINT(")");
}

/**
 * @brief Construct a new Line 3d:: Line 3d object
 *
 */
Line3d::Line3d()
{
    this->p1 = Point3d();
    this->p2 = Point3d();
}

/**
 * @brief Copy construct a new Line 3d:: Line 3d object
 *
 * @param l
 */
Line3d::Line3d(const Line3d &l)
{
    this->p1 = l.p1;
    this->p2 = l.p2;
}

/**
 * @brief Assignment operator
 *
 * @param l other Line3d
 * @return const Line3d& l
 */
const Line3d &Line3d::operator=(const Line3d &l)
{
    this->p1 = l.p1;
    this->p2 = l.p2;

    return l;
}

/**
 * @brief Destructor for Line3d
 *
 */
Line3d::~Line3d() {}

/**
 * @brief Construct a new Line 3d:: Line 3d object
 *
 * @param p1 Point3d point 1
 * @param p2 Point3d point 2
 */
Line3d::Line3d(const Point3d &p1, const Point3d &p2)
{
    this->p1 = p1;
    this->p2 = p2;
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Line3d::debugPrint()
{
    this->p1.debugPrint();
    PRINT(" ,");
    this->p2.debugPrint();
}

Line2d::Line2d()
{
    this->p1 = Point2d();
    this->p2 = Point2d();
}

Line2d::Line2d(const Line2d &l)
{
    this->p1 = l.p1;
    this->p2 = l.p2;
}

const Line2d &Line2d::operator=(const Line2d &l)
{
    this->p1 = l.p1;
    this->p2 = l.p2;

    return l;
}

Line2d::~Line2d() {}

Line2d::Line2d(const Point2d &p1, const Point2d &p2)
{
    this->p1 = p1;
    this->p2 = p2;
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 */
ThreeFX_Fixed::ThreeFX_Fixed()
{
    this->setGFX(NULL);
    this->setZNear(THREEFX_Z_NEAR_DEFAULT);
    this->setZFar(THREEFX_Z_FAR_DEFAULT);
    this->setFovAngle(THREEFX_FOV_DEFAULT);
    this->updateLambda();
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx pointer to attached gfx object
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx)
{
    this->setGFX(gfx);
    this->setZNear(THREEFX_Z_NEAR_DEFAULT);
    this->setZFar(THREEFX_Z_FAR_DEFAULT);
    this->setFovAngle(THREEFX_FOV_DEFAULT);
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx pointer to attached gfx object
 * @param fov camera field of view
 * @param z_near z_near coordinate
 * @param z_far z_far coordinate
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx, double fov, double z_near, double z_far)
{
    this->setGFX(gfx);
    this->setZNear(z_near);
    this->setZFar(z_far);
    this->setFovAngle(fov);
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx point to attached gfx object
 * @param fov camera field of view
 * @param z_near z_near coordinate
 * @param z_far z_far coordinate
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx, double fov, const Fix16 &z_near, const Fix16 &z_far)
{
    this->setGFX(gfx);
    this->setZNear(z_near);
    this->setZFar(z_far);
    this->setFovAngle(fov);
}

/**
 * @brief Setter for z_near
 *
 * @param z_near
 */
void ThreeFX_Fixed::setZNear(double z_near) { this->setZNear(Fix16(z_near)); }

/**
 * @brief Setter for z_near
 *
 * @param z_near
 */
void ThreeFX_Fixed::setZNear(Fix16 z_near)
{
    this->z_near = z_near;
    this->updateLambda();
}

/**
 * @brief Setter for z_far
 *
 * @param z_far
 */
void ThreeFX_Fixed::setZFar(double z_far) { this->setZFar(Fix16(z_far)); }

/**
 * @brief Setter for z_far
 *
 * @param z_far
 */
void ThreeFX_Fixed::setZFar(Fix16 z_far)
{
    this->z_far = z_far;    
    this->updateLambda();
}

/**
 * @brief Setter for gfx
 *
 * @param gfx
 */
void ThreeFX_Fixed::setGFX(Arduino_GFX *gfx)
{
    this->gfx = gfx;
    this->gfx_width = Fix16(this->gfx->width());
    this->gfx_height = Fix16(this->gfx->height());
    this->updateAspect();
}

/**
 * @brief Setter for FOV angle
 *
 * @param f fov angle in radians
 */
void ThreeFX_Fixed::setFovAngle(double f)
{
    this->fov_angle = f;
    this->fov = Fix16(1.) / (this->fov_angle / 2.).tan();
}
/**
 * @brief Converts a world coordinate to a screen coordinate normalized within
 * the bounds of the screen box
 *
 * @param p point to convert
 * @return Point3d point in screen coordinates
 */
inline Point3d ThreeFX_Fixed::worldToScreen(const Point3d &p)
{
    Point3d res = Point3d();

    // need to do one int divide to get 1 / dz
    Fix16 inv_z = Fix16(1.) / p.z;

    // then everything becomes int multiply
    res.x = this->a * this->fov * p.x * inv_z;
    res.y = this->fov * p.y * inv_z;
    res.z = (p.z - this->z_near) * lambda * inv_z;

    return res;
}

/**
 * @brief Converts a screen point (normalized to [-1, 1] that are edges of
 * the screen) to actual pixels
 *
 * @param p Point3d to convert
 * @return Point2d in screen coordinates
 */
inline Point2d ThreeFX_Fixed::screenToPixel(const Point3d &p)
{
    Point2d res;
    Fix16 half = 1. / 2.;
    res.x = (p.x + half) * this->gfx_width;
    res.y = (half - p.y) * this->gfx_height;
    return res;
}

/**
 * @brief Chains worldToScreen and screenToPixel
 *
 * @param p Point3d point
 * @return Point2d screen point
 */
inline Point2d ThreeFX_Fixed::worldToPixel(const Point3d &p)
{
    Point2d p2 = this->screenToPixel(this->worldToScreen(p));
    return p2;
}

/**
 * @brief Draws a 3d world coordinate on screen using Arduino_GFX
 *
 * @param p 3d world coordinate
 * @param color 6-bit color
 * @return bool of if coordinate was on screen
 */
bool ThreeFX_Fixed::drawPoint3d(const Point3d &p, const uint16_t color)
{
    Point2d p1 = worldToPixel(p);

    bool val = IN_RANGE_INC(p1.x, Fix16((int16_t)0), this->gfx_width) &&
               IN_RANGE_INC(p1.y, Fix16((int16_t)0), this->gfx_height);

    if (val)
    {
        this->gfx->drawPixel(
            (int16_t)p1.x,
            (int16_t)p1.y,
            color);
    }

    return val;
}

/**
 * @brief Draws a line from 3d world coordinates on screen using Arduino_GFX
 *
 * @param wl 3d world line
 * @param color 6-bit color
 * @return bool, true if line was drawn on screen
 */
bool ThreeFX_Fixed::drawLine3d(const Line3d &wl, const uint16_t color)
{
    Point2d p1 = this->worldToPixel(wl.p1);
    Point2d p2 = this->worldToPixel(wl.p2);

    bool val = this->cohenSutherlandClip(p1, p2);

    if (val)
    {
        this->gfx->drawLine(
            int16_t(p1.x),
            int16_t(p1.y),
            int16_t(p2.x),
            int16_t(p2.y),
            color);
    }
    return val;
}

bool ThreeFX_Fixed::cohenSutherlandClip(Point2d &p0, Point2d &p1)
{
    unsigned char code0 = this->cohenSutherlandCode(p0);
    unsigned char code1 = this->cohenSutherlandCode(p1);

    bool accept = false;
    while (true)
    {
        unsigned char code_out;
        Point2d p = Point2d();

        if ((code0 == 0) && (code1 == 0))
            break;
        if (code0 & code1)
            break;

        if (code0)
        {
            code_out = code0;
        }
        else
        {
            code_out = code1;
        }

        // find intersection point
        // TODO is there a better way to get coordinates than integer division
        if (code_out & CL_TOP)
        {
            p.x = p0.x + (p1.x - p0.x) * (Fix16(0.) - p0.y) / (p1.y - p0.y);
            p.y = Fix16(0.);
        }
        else if (code_out & CL_BOTTOM)
        {
            p.x = p0.x + (p1.x - p0.x) * (this->gfx_height - p0.y) / (p1.y - p0.y);
            p.y = this->gfx_height;
        }
        else if (code_out & CL_RIGHT)
        {
            p.x = this->gfx_width;
            p.y = p0.y + (p1.y - p0.y) * (this->gfx_width - p0.x) / (p1.x - p0.x);
        }
        else if (code_out & CL_LEFT)
        {
            p.x = Fix16(0.);
            p.y = p0.y + (p1.y - p0.y) * (Fix16(0.) - p0.x) / (p1.x - p0.x);
        }
        else 
        {
            // something must have gone wrong to get here...
        }
        
        if (code_out == code0)
        {
            p0 = p;
            code0 = this->cohenSutherlandCode(p0);
        }
        else
        {
            p1 = p;
            code1 = this->cohenSutherlandCode(p1);
        }
    }

    // this line can be drawn on screen if both segment codes are 0 (centered)
    return ((code0 | code1) == CL_INSIDE);
}

/**
 * @brief Recalculates lambda value; called on any change to z_near, z_far
 *
 */
void ThreeFX_Fixed::updateLambda()
{
    this->lambda = this->z_far / (this->z_far - this->z_near);

    char buf[16];
    fix16_to_str(this->lambda.value, buf, 4);
    LOG_DEBUG("update lambda to", buf);
}

/**
 * @brief Recalculates aspect ratio; called on any change to gfx
 *
 */
void ThreeFX_Fixed::updateAspect()
{
    double height = this->gfx->height();
    double width = this->gfx->width();
    this->a = Fix16(height / width);

    char buf[16];
    fix16_to_str(this->a, buf, 4);
    LOG_DEBUG("update aspect to", buf);
}

inline unsigned char ThreeFX_Fixed::cohenSutherlandCode(const Point2d &p)
{
    unsigned char code = CL_INSIDE;

    if (p.x < Fix16(0.))
    {
        code |= CL_LEFT;
    }
    else if (p.x > this->gfx_width)
    {
        code |= CL_RIGHT;
    }

    if (p.y < Fix16(0.))
    {
        code |= CL_TOP;
    }
    else if (p.y > this->gfx_height)
    {
        code |= CL_BOTTOM;
    }

    return code;
}