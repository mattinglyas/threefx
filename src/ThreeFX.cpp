#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

bool Q3dCompare::operator()(Queued3d &a, Queued3d &b)
{
    if (a.second.z < b.second.z) 
        return true;
    else
        return false;
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
    this->updateLambdaScale();

    this->draw_queue = new std::priority_queue<Queued3d, std::vector<Queued3d>, Q3dCompare>();
    this->alpha = this->gamma = this->beta = Fix16(0.);
    this->cam_origin = Coord3d(0., 0., 0.);
    this->updateCamTransform();
}

/**
 * @brief Construct a new ThreeFX_Fixed::ThreeFX_Fixed object
 *
 * @param gfx pointer to attached gfx object
 */
ThreeFX_Fixed::ThreeFX_Fixed(Arduino_GFX *gfx) : ThreeFX_Fixed()
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
    this->updateLambdaScale();
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
    this->updateLambdaScale();
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
 * @param alpha
 * @param beta
 * @param gamma
 */
void ThreeFX_Fixed::setCameraAngle(const Fix16 &alpha, const Fix16 &beta, const Fix16 &gamma)
{
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;

    this->updateCamTransform();
}

/**
 * @brief Sets camera origin point
 *
 * @param pt
 */
void ThreeFX_Fixed::setCameraOrigin(const Coord3d &pt)
{
    this->cam_origin = pt;
}

/**
 * @brief Chains worldToScreen and screenToPixel
 *
 * @param p Coord3d point
 * @return Coord2d screen point
 */
Coord2d ThreeFX_Fixed::worldToPixel(const Coord3d &p)
{
    Coord2d p2 = this->screenToPixel(this->worldToScreen(p));
    return p2;
}

/**
 * @brief Get the Camera Yaw object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraYaw()
{
    return this->alpha;
}

/**
 * @brief Get the Camera beta object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraPitch()
{
    return this->beta;
}

/**
 * @brief Get the Camera gamma object
 *
 * @return Fix16
 */
Fix16 ThreeFX_Fixed::getCameraRoll()
{
    return this->gamma;
}

/**
 * @brief Draws a 3d world coordinate on screen using Arduino_GFX
 *
 * @param p 3d world coordinate
 * @param color 6-bit color
 * @return bool of if coordinate was on screen
 */
bool ThreeFX_Fixed::drawWorldPoint3d(const Point3d &wp)
{
    return this->drawScreenPoint3d(
        Point3d(this->worldToScreen(wp.p0), wp.color)
    );
}

/**
 * @brief Draws a line from 3d world coordinates on screen using Arduino_GFX
 *
 * @param wl 3d world line
 * @param color 6-bit color
 * @return bool, true if line was drawn on screen
 */
bool ThreeFX_Fixed::drawWorldLine3d(const Line3d &wl)
{
    Coord3d sp0 = this->worldToScreen(wl.p0);
    Coord3d sp1 = this->worldToScreen(wl.p1);

    return this->drawScreenLine3d(Line3d(sp0, sp1, wl.color));
}

bool ThreeFX_Fixed::drawWorldFacet3d(const Facet3d &wf)
{
    Coord3d sp0 = this->worldToScreen(wf.p0);
    Coord3d sp1 = this->worldToScreen(wf.p1);
    Coord3d sp2 = this->worldToScreen(wf.p2);

    return this->drawScreenFacet3d(Facet3d(sp0, sp1, sp2, wf.color));
}

bool ThreeFX_Fixed::drawScreenPoint3d(const Point3d &sp)
{
    // could likely use cohenSutherlandCode
    bool val = SCREEN_COORD_ON_SCREEN(sp.p0);

    if (val)
    {
        Coord2d pp0 = this->screenToPixel(sp.p0);
        this->gfx->drawPixel((int16_t)pp0.x, (int16_t)pp0.y, sp.color);
    }

    return val;
}

bool ThreeFX_Fixed::drawScreenLine3d(const Line3d &sl)
{
    Coord3d sp0 = sl.p0;
    Coord3d sp1 = sl.p1;
    bool val = this->cohenSutherlandClip(sp0, sp1);

    if (val)
    {
        Coord2d pp0 = this->screenToPixel(sp0);
        Coord2d pp1 = this->screenToPixel(sp1);
        this->gfx->drawLine(
            int16_t(pp0.x),
            int16_t(pp0.y),
            int16_t(pp1.x),
            int16_t(pp1.y),
            sl.color);
    }

    return val;
}

bool ThreeFX_Fixed::drawScreenFacet3d(const Facet3d &sf)
{
    // TODO for now, only draw triangles that are entirely within draw window
    bool val = (SCREEN_COORD_ON_SCREEN(sf.p0) &&
                SCREEN_COORD_ON_SCREEN(sf.p1) &&
                SCREEN_COORD_ON_SCREEN(sf.p2));
    
    if (val)
    {
        Coord2d pp0 = this->screenToPixel(sf.p0);
        Coord2d pp1 = this->screenToPixel(sf.p1);
        Coord2d pp2 = this->screenToPixel(sf.p2);

        this->gfx->fillTriangle(
            int16_t(pp0.x),
            int16_t(pp0.y),
            int16_t(pp1.x),
            int16_t(pp1.y),
            int16_t(pp2.x),
            int16_t(pp2.y),
            sf.color
        );
    }

    return val;
}

/**
 * @brief Adds a drawable to the draw queue, re-ordering the queue based on
 * barycenter of drawable (for painter's algorithm)
 * 
 * @return int 0 if succesful, other if error
 */
int ThreeFX_Fixed::queueDrawable(const Drawable3d *d)
{
    // TODO calculate barycenter with camera transform, store pointer to drawable
    // to allow polymorphism with flush draw queue 
    Queued3d n_draw;
    n_draw.first = d;
    n_draw.second = this->worldToScreen(d->getBarycenter());
    
    this->draw_queue->push(n_draw);

    return 0;
}

/**
 * @brief Clears draw queue by drawing all Queued3d to the screen in
 * order of z-depth (for painter's algorithm)
 * 
 * @return int 0 if successful, other if error
 */
int ThreeFX_Fixed::flushDrawQueue()
{
    Queued3d curr;
    int res = 0;
    while (!this->draw_queue->empty())
    {
        curr = this->draw_queue->top();
        this->draw_queue->pop();

        res |= curr.first->draw(*this);
    }

    return res;
}

/**
 * @brief Clears draw queue of all items without drawing anything to screen
 * 
 */
void ThreeFX_Fixed::clearDrawQueue()
{
    while (!this->draw_queue->empty())
    {
        this->draw_queue->pop();
    }
}

/**
 * @brief Performs cohen-sutherland clipping on line formed by p0 and p1
 * boxed into the pixel size
 *
 * @param p0 Coord2d 0
 * @param p1 Coord2d 1
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Coord2d &p0, Coord2d &p1)
{
    return this->cohenSutherlandClip(
        p0,
        p1,
        Coord2d(0., 0.),
        Coord2d(this->gfx_width, this->gfx_height));
}

/**
 * @brief Performs cohen-sutherland clipping on line formed by p0 and p1
 * boxed into the pixel size
 *
 * @param p0 Coord2d 0
 * @param p1 Coord2d 1
 * @param min window minimum Coord2d
 * @param max window maximum Coord2d
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Coord2d &p0, Coord2d &p1, const Coord2d &min, const Coord2d &max)
{
    unsigned char code0 = this->cohenSutherlandCode(p0, min, max);
    unsigned char code1 = this->cohenSutherlandCode(p1, min, max);

    bool accept = false;
    while (true)
    {
        unsigned char code_out;
        Coord2d p = Coord2d();

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
 * boxed into the screen coordinates (NOT pixel coordinates)
 *
 * @param p0 Coord3d 0
 * @param p1 Coord3d 1
 * @return true if the line can be drawn
 * @return false if the line cannot be drawn
 */
bool ThreeFX_Fixed::cohenSutherlandClip(Coord3d &p0, Coord3d &p1)
{
    return this->cohenSutherlandClip(
        p0,
        p1,
        Coord3d(-1., -1., 0.),
        Coord3d(1., 1., 1.));
}

bool ThreeFX_Fixed::cohenSutherlandClip(Coord3d &p0, Coord3d &p1, const Coord3d &min, const Coord3d& max)
{
    
    unsigned char code0 = this->cohenSutherlandCode(p0, min, max);
    unsigned char code1 = this->cohenSutherlandCode(p1, min, max);

    bool accept = false;
    while (true)
    {
        unsigned char code_out;
        Coord3d p = Coord3d();

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
 * @brief Recalculates lambda value and z scale; called on any change to z_near, z_far
 *
 */
void ThreeFX_Fixed::updateLambdaScale()
{
    this->lambda = this->z_far / (this->z_far - this->z_near);
    this->z_scale = (Fix16(1.) / (this->z_far - this->z_near));

    char buf[16];
    fix16_to_str(this->lambda.value, buf, 4);
    LOG_DEBUG("update lambda to", buf);

    fix16_to_str(this->z_scale.value, buf, 4);
    LOG_DEBUG("update z_scale to", buf);
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
 * @brief Re-calculates camera transform matrix based on stored alpha, beta, gamma
 * (see https://msl.cs.uiuc.edu/planning/node102.html); matrix stored in [y][x]
 *
 */
void ThreeFX_Fixed::updateCamTransform()
{
    // cos(a) cos(b)
    this->cam_mtrx[0][0] = this->alpha.cos() * this->beta.cos();
    // cos(a) sin(b) sin(g) - sin(a) cos(g)
    this->cam_mtrx[0][1] = (this->alpha.cos() * this->beta.sin() * this->gamma.sin()) - (this->alpha.sin() * this->gamma.cos());
    // cos(a) sin(b) cos(g) + sin(a) sin(g)
    this->cam_mtrx[0][2] = (this->alpha.cos() * this->beta.sin() * this->gamma.cos()) + (this->alpha.sin() * this->gamma.sin());
    // sin(a) cos(b)
    this->cam_mtrx[1][0] = this->alpha.sin() * this->beta.cos();
    // sin(a) sin(b) sin(g) + cos(a) cos(g)
    this->cam_mtrx[1][1] = (this->alpha.sin() * this->beta.cos() * this->gamma.sin()) + (this->alpha.cos() * this->gamma.cos());
    // sin(a) sin(b) cos(g) - cos(a) sin(g)
    this->cam_mtrx[1][2] = (this->alpha.sin() * this->beta.sin() * this->gamma.cos()) - (this->alpha.cos() * this->gamma.sin());
    // -sin(b)
    this->cam_mtrx[2][0] = this->beta.sin() * -1.;
    // cos(b) sin(g)
    this->cam_mtrx[2][1] = this->beta.cos() * this->gamma.sin();
    // cos(b) cos(g)
    this->cam_mtrx[2][2] = this->beta.cos() * this->gamma.cos();

    LOG_TRACE("update camera matrix");
    LOG_TRACE(this->cam_mtrx[0][0].value, this->cam_mtrx[0][1].value, this->cam_mtrx[0][2].value);
    LOG_TRACE(this->cam_mtrx[1][0].value, this->cam_mtrx[1][1].value, this->cam_mtrx[1][2].value);
    LOG_TRACE(this->cam_mtrx[2][0].value, this->cam_mtrx[2][1].value, this->cam_mtrx[2][2].value);
}

/**
 * @brief Calculates Cohen-Sutherland code for a 2d point
 *
 * @param p 2d point
 * @param min minimum bounding rectangle  
 * @param max maximum bounding rectangle
 * @return unsigned char
 */
unsigned char ThreeFX_Fixed::cohenSutherlandCode(const Coord2d &p, const Coord2d &min, const Coord2d &max)
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
unsigned char ThreeFX_Fixed::cohenSutherlandCode(const Coord3d &p, const Coord3d &min, const Coord3d &max)
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
 * @return Coord3d
 */
Coord3d ThreeFX_Fixed::worldCamera(const Coord3d &p)
{
    Coord3d p1 = p;

    // subtract camera origin from point to place camera at origin
    p1 -= this->cam_origin;

    // apply rotation about origin
    Coord3d p2;
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
 * @return Coord3d point in screen coordinates
 */
Coord3d ThreeFX_Fixed::worldToScreen(const Coord3d &p)
{
    Coord3d p1, res;
    p1 = this->worldCamera(p);

    // need to do one int divide to get 1 / dz
    Fix16 inv_z = Fix16(1.) / p1.z;

    // then everything becomes int multiply
    res.x = this->a * this->fov * p1.x * inv_z;
    res.y = this->fov * p1.y * inv_z;
    res.z = (p1.z - this->z_near) * this->lambda * this->z_scale;

    return res;
}

/**
 * @brief Converts a screen point (normalized to [-1, 1] that are edges of
 * the screen) to actual pixels
 *
 * @param p Coord3d to convert
 * @return Coord2d in screen coordinates
 */
Coord2d ThreeFX_Fixed::screenToPixel(const Coord3d &p)
{
    Coord2d res;
    res.x = (p.x + Fix16(1.)) * this->gfx_width * Fix16(1. / 2.);
    res.y = (Fix16(1.) - p.y) * this->gfx_height * Fix16(1. / 2.);

    return res;
}