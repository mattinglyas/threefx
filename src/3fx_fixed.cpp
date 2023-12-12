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

double function(const double &x)
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
 * @brief Relative adder operator for point3d
 *
 * @param rhs
 * @return Point3d&
 */
Point3d &Point3d::operator+=(const Point3d &rhs)
{
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;

    return *this;
}

/**
 * @brief Adder operator for point3d; adds per coordinate
 *
 * @param rhs
 * @return Point3d&
 */
const Point3d Point3d::operator+(const Point3d &rhs)
{
    Point3d ret = *this;
    ret.x += rhs.x;
    ret.y += rhs.y;
    ret.z += rhs.z;

    return ret;
}

/**
 * @brief Relative subtractor operator for point3d
 *
 * @param rhs
 * @return Point3d&
 */
Point3d &Point3d::operator-=(const Point3d &rhs)
{
    this->x -= rhs.x;
    this->y -= rhs.y;
    this->z -= rhs.z;

    return *this;
}

/**
 * @brief Subtractor operator for point3d; subtracts per coordinate
 *
 * @param rhs
 * @return Point3d&
 */
const Point3d Point3d::operator-(const Point3d &rhs)
{
    Point3d ret = *this;
    ret.x -= rhs.x;
    ret.y -= rhs.y;
    ret.z -= rhs.z;

    return ret;
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
    this->p0 = Point3d();
    this->p1 = Point3d();
}

/**
 * @brief Copy construct a new Line 3d:: Line 3d object
 *
 * @param l
 */
Line3d::Line3d(const Line3d &l)
{
    this->p0 = l.p0;
    this->p1 = l.p1;
}

/**
 * @brief Assignment operator
 *
 * @param l other Line3d
 * @return const Line3d& l
 */
const Line3d &Line3d::operator=(const Line3d &l)
{
    this->p0 = l.p0;
    this->p1 = l.p1;

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
Line3d::Line3d(const Point3d &p0, const Point3d &p1)
{
    this->p0 = p0;
    this->p1 = p1;
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Line3d::debugPrint()
{
    this->p0.debugPrint();
    PRINT(" ,");
    this->p1.debugPrint();
}

Line2d::Line2d()
{
    this->p0 = Point2d();
    this->p1 = Point2d();
}

Line2d::Line2d(const Line2d &l)
{
    this->p0 = l.p0;
    this->p1 = l.p1;
}

const Line2d &Line2d::operator=(const Line2d &l)
{
    this->p0 = l.p0;
    this->p1 = l.p1;

    return l;
}

Line2d::~Line2d() {}

Line2d::Line2d(const Point2d &p0, const Point2d &p1)
{
    this->p0 = p0;
    this->p1 = p1;
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

    this->yaw = this->roll = this->pitch = Fix16(0.);
    this->cam_origin = Point3d(0., 0., 0.);
    this->updateCamTransform();
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx pointer to attached gfx object
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx)
{
    this->setGFX(gfx);
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx pointer to attached gfx object
 * @param fov camera field of view
 * @param z_near z_near coordinate
 * @param z_far z_far coordinate
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx, double fov, double z_near, double z_far) : ThreeFX_Fixed(gfx)
{
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
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx, double fov, const Fix16 &z_near, const Fix16 &z_far) : ThreeFX_Fixed(gfx)
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
void ThreeFX_Fixed::setZNear(const Fix16 &z_near)
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
void ThreeFX_Fixed::setZFar(const Fix16 &z_far)
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

    if (this->gfx)
    {
        this->gfx_width = Fix16(this->gfx->width());
        this->gfx_height = Fix16(this->gfx->height());
        this->updateAspect();

        LOG_DEBUG("attached GFX");
    }
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
 * @brief Sets camera angle
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void ThreeFX_Fixed::setCameraAngle(const Fix16 &yaw, const Fix16 &pitch, const Fix16 &roll)
{
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;

    this->updateCamTransform();
}

/**
 * @brief Sets camera origin point
 *
 * @param pt
 */
void ThreeFX_Fixed::setCameraOrigin(const Point3d &pt)
{
    this->cam_origin = pt;
}

/**
 * @brief Chains worldToScreen and screenToPixel
 *
 * @param p Point3d point
 * @return Point2d screen point
 */
Point2d ThreeFX_Fixed::worldToPixel(const Point3d &p)
{
    Point2d p2 = this->screenToPixel(this->worldToScreen(p));
    return p2;
}

/**
 * @brief Get the Camera Yaw object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraYaw()
{
    return this->yaw;
}

/**
 * @brief Get the Camera pitch object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraPitch()
{
    return this->pitch;
}

/**
 * @brief Get the Camera roll object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraRoll()
{
    return this->roll;
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
    Point3d sp0 = this->worldToScreen(wl.p0);
    Point3d sp1 = this->worldToScreen(wl.p1);

    bool val = this->cohenSutherlandClip(sp0, sp1);

    if (val)
    {
        Point2d p0 = this->screenToPixel(sp0);
        Point2d p1 = this->screenToPixel(sp1);
        this->gfx->drawLine(
            int16_t(p0.x),
            int16_t(p0.y),
            int16_t(p1.x),
            int16_t(p1.y),
            color);
    }

    return val;
}

/**
 * @brief Performs cohen-sutherland clipping on line formed by p0 and p1
 * boxed into the screen size
 *
 * @param p0 point 0
 * @param p1 point 1
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Point2d &p0, Point2d &p1)
{
    return this->cohenSutherlandClip(
        p0,
        p1,
        Point2d(0., 0.),
        Point2d(this->gfx_width, this->gfx_height));
}

/**
 * @brief Performs cohen-sutherland clipping on line formed by p0 and p1
 * boxed into the screen size
 *
 * @param p0 point 0
 * @param p1 point 1
 * @param min window minimum coordinates
 * @param max window maximum coordinates
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Point2d &p0, Point2d &p1, const Point2d &min, const Point2d &max)
{
    unsigned char code0 = this->cohenSutherlandCode(p0, min, max);
    unsigned char code1 = this->cohenSutherlandCode(p1, min, max);

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
            p.x = p0.x + ((p1.x - p0.x) / (p1.y - p0.y)) * (min.y - p0.y);
            p.y = min.y;
        }
        else if (code_out & CL_BOTTOM)
        {
            p.x = p0.x + ((p1.x - p0.x) / (p1.y - p0.y)) * (max.y - p0.y);
            p.y = max.y;
        }
        else if (code_out & CL_RIGHT)
        {
            p.x = max.x;
            p.y = p0.y + ((p1.y - p0.y) / (p1.x - p0.x)) * (max.x - p0.x);
        }
        else if (code_out & CL_LEFT)
        {
            p.x = min.x;
            p.y = p0.y + ((p1.y - p0.y) / (p1.x - p0.x)) * (min.x - p0.x);
        }
        else
        {
            // something must have gone wrong to get here...
        }

        if (code_out == code0)
        {
            p0 = p;
            code0 = this->cohenSutherlandCode(p0, min, max);
        }
        else
        {
            p1 = p;
            code1 = this->cohenSutherlandCode(p1, min, max);
        }
    }

    // this line can be drawn on screen if both segment codes are 0 (centered)
    return ((code0 | code1) == CL_INSIDE);
}

/**
 * @brief Performs cohen-sutherland clipping on line formed by p0 and p1
 * boxed into the on-screen screen coordinates (NOT pixel coordinates)
 *
 * @param p0 point 0
 * @param p1 point 1
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Point3d &p0, Point3d &p1)
{
    return this->cohenSutherlandClip(
        p0,
        p1,
        Point3d(-1., -1., 0.),
        Point3d(1., 1., 1.));
}

bool ThreeFX_Fixed::cohenSutherlandClip(Point3d &p0, Point3d &p1, const Point3d &min, const Point3d& max)
{
    
    unsigned char code0 = this->cohenSutherlandCode(p0, min, max);
    unsigned char code1 = this->cohenSutherlandCode(p1, min, max);

    bool accept = false;
    while (true)
    {
        unsigned char code_out;
        Point3d p = Point3d();

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
        // TODO is there a better way to get coordinates than integer division,
        // look to see if compiler optimizes duplicated parameters like slope
        if (code_out & CL_LEFT)
        {
            p.x = min.x;
            p.y = p0.y + ((p1.y - p0.y) / (p1.x - p0.x)) * (min.x - p0.x);
            p.z = p0.z + ((p1.z - p0.z) / (p1.x - p0.x)) * (min.x - p0.x);
        }
        else if (code_out & CL_RIGHT)
        {
            p.x = max.x;
            p.y = p0.y + ((p1.y - p0.y) / (p1.x - p0.x)) * (max.x - p0.x);
            p.z = p0.z + ((p1.z - p0.z) / (p1.x - p0.x)) * (max.x - p0.x);
        }
        else if (code_out & CL_TOP)
        {
            p.x = p0.x + ((p1.x - p0.x) / (p1.y - p0.y)) * (min.y - p0.y);
            p.y = min.y;
            p.z = p0.z + ((p1.z - p0.z) / (p1.y - p0.y)) * (min.y - p0.y);
        }
        else if (code_out & CL_BOTTOM)
        {
            p.x = p0.x + ((p1.x - p0.x) / (p1.y - p0.y)) * (max.y - p0.y);
            p.y = max.y;
            p.z = p0.z + ((p1.z - p0.z) / (p1.y - p0.y)) * (max.y - p0.y);
        }
        else if (code_out & CL_FRONT)
        {
            p.x = p0.x + ((p1.x - p0.x) / (p1.z - p0.z)) * (min.z - p0.z);
            p.y = p0.y + ((p1.y - p0.y) / (p1.z - p0.z)) * (min.z - p0.z);
            p.z = min.z;
        } 
        else if (code_out & CL_BACK)
        {
            p.x = p0.x + ((p1.x - p0.x) / (p1.z - p0.z)) * (max.z - p0.z);
            p.y = p0.y + ((p1.y - p0.y) / (p1.z - p0.z)) * (max.z - p0.z);
            p.z = max.z;
        }
        else
        {
            // something must have gone wrong to get here...
        }

        if (code_out == code0)
        {
            p0 = p;
            code0 = this->cohenSutherlandCode(p0, min, max);
        }
        else
        {
            p1 = p;
            code1 = this->cohenSutherlandCode(p1, min, max);
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

/**
 * @brief Re-calculates camera transform matrix based on stored yaw, pitch, roll
 * (see https://msl.cs.uiuc.edu/planning/node102.html); matrix stored in [y][x]
 *
 */
void ThreeFX_Fixed::updateCamTransform()
{
    this->cam_mtrx[0][0] = this->yaw.cos() * this->pitch.cos();
    this->cam_mtrx[0][1] = this->yaw.cos() * this->pitch.sin() * this->roll.sin() - this->yaw.sin() * this->roll.cos();
    this->cam_mtrx[0][1] = this->yaw.cos() * this->pitch.sin() * this->roll.cos() + this->yaw.sin() * this->roll.sin();
    this->cam_mtrx[1][0] = this->yaw.sin() * this->pitch.cos();
    this->cam_mtrx[1][1] = this->yaw.sin() * this->yaw.cos() * this->roll.sin() + this->yaw.cos() * this->roll.cos();
    this->cam_mtrx[1][2] = this->yaw.sin() * this->pitch.sin() * this->roll.cos() - this->yaw.cos() * this->roll.sin();
    this->cam_mtrx[2][0] = this->pitch.sin() * -1.;
    this->cam_mtrx[2][1] = this->pitch.cos() * this->roll.sin();
    this->cam_mtrx[2][2] = this->pitch.cos() * this->roll.cos();

    LOG_DEBUG("update camera matrix: ");
    LOG_DEBUG(this->cam_mtrx[0][0].value, this->cam_mtrx[0][1].value, this->cam_mtrx[0][2].value);
    LOG_DEBUG(this->cam_mtrx[1][0].value, this->cam_mtrx[1][1].value, this->cam_mtrx[1][2].value);
    LOG_DEBUG(this->cam_mtrx[2][0].value, this->cam_mtrx[2][1].value, this->cam_mtrx[2][2].value);
}

/**
 * @brief Calculates Cohen-Sutherland code for a 2d point
 *
 * @param p 2d point
 * @param min minimum bounding rectangle  
 * @param max maximum bounding rectangle
 * @return unsigned char
 */
unsigned char ThreeFX_Fixed::cohenSutherlandCode(const Point2d &p, const Point2d &min, const Point2d &max)
{
    unsigned char code = CL_INSIDE;

    if (p.x < min.x)
    {
        code |= CL_LEFT;
    }
    else if (p.x > max.x)
    {
        code |= CL_RIGHT;
    }

    if (p.y < min.y)
    {
        code |= CL_TOP;
    }
    else if (p.y > max.y)
    {
        code |= CL_BOTTOM;
    }

    return code;
}

/**
 * @brief Calculates Cohen-Sutherland code for a 2d point
 *
 * @param p
 * @return unsigned char
 */
unsigned char ThreeFX_Fixed::cohenSutherlandCode(const Point3d &p, const Point3d &min, const Point3d &max)
{
    unsigned char code = CL_INSIDE;

    if (p.x < min.x)
        code |= CL_LEFT;
    else if (p.x > max.x)
        code |= CL_RIGHT;
    
    if (p.y < min.y)
        code |= CL_TOP;
    else if (p.y > max.y)
        code |= CL_BOTTOM;
    
    if (p.z < min.z)
        code |= CL_FRONT;
    else if (p.z > max.z)
        code |= CL_BACK;
    
    return code;
}

/**
 * @brief Adjusts 3d point for camera angle and position
 *
 * @return Point3d
 */
Point3d ThreeFX_Fixed::worldCamera(const Point3d &p)
{
    Point3d p1 = p;

    // subtract camera origin from point to place camera at origin
    p1 -= this->cam_origin;

    // apply rotation about origin
    Point3d p2;
    p2.x = p1.x * this->cam_mtrx[0][0] + p1.y * this->cam_mtrx[1][0] + p1.z * this->cam_mtrx[2][0];
    p2.y = p1.x * this->cam_mtrx[0][1] + p1.y * this->cam_mtrx[1][1] + p1.z * this->cam_mtrx[2][1];
    p2.z = p1.x * this->cam_mtrx[0][2] + p1.y * this->cam_mtrx[1][2] + p1.z * this->cam_mtrx[2][2];

    return p2;
}

/**
 * @brief Converts a world coordinate to a screen coordinate normalized within
 * the bounds of the screen box
 *
 * @param p point to convert
 * @return Point3d point in screen coordinates
 */
Point3d ThreeFX_Fixed::worldToScreen(const Point3d &p)
{
    Point3d p1, res;
    p1 = this->worldCamera(p);

    // need to do one int divide to get 1 / dz
    Fix16 inv_z = Fix16(1.) / p.z;

    // then everything becomes int multiply
    res.x = this->a * this->fov * p1.x * inv_z;
    res.y = this->fov * p1.y * inv_z;
    res.z = (p1.z - this->z_near) * this->lambda * inv_z;

    return res;
}

/**
 * @brief Converts a screen point (normalized to [-1, 1] that are edges of
 * the screen) to actual pixels
 *
 * @param p Point3d to convert
 * @return Point2d in screen coordinates
 */
Point2d ThreeFX_Fixed::screenToPixel(const Point3d &p)
{
    Point2d res;
    res.x = (p.x + Fix16(1.)) * this->gfx_width * Fix16(1. / 2.);
    res.y = (Fix16(1.) - p.y) * this->gfx_height * Fix16(1. / 2.);
    return res;
}