#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Point 3d:: Point 3d object
 * 
 */
Point3d::Point3d()
{
    this->color = DEFAULT_COLOR;
    this->p0 = Coord3d();
}

Point3d::Point3d(const Coord3d &p0) : Point3d()
{
    this->p0 = p0;
}

Point3d::Point3d(const Coord3d &p0, const uint16_t color) : Point3d(p0)
{
    this->color = color;
}

Point3d::Point3d(const Fix16 &x, const Fix16 &y, const Fix16 &z) : Point3d(Coord3d(x, y, z)) {}
Point3d::Point3d(const Fix16 &x, const Fix16 &y, const Fix16 &z, const uint16_t color) : Point3d(Coord3d(x,y,z), color) {}

/**
 * @brief Copy construct a new Point 3d:: Point 3d object
 * 
 * @param p 
 */
Point3d::Point3d(const Point3d &p)
{
    this->p0 = p.p0;
}

/**
 * @brief Assignment operator
 * 
 * @param p other Point3d
 * @return const Point3d&  
 */
const Point3d &Point3d::operator=(const Point3d &p)
{
    this->p0 = p.p0;

    return p;
}

/**
 * @brief Destroy the Point 3d:: Point 3d object
 * 
 */
Point3d::~Point3d() {}

/**
 * @brief Prints self using debug_print and debug_println
 * 
 */
void Point3d::debugPrint() const
{
    this->p0.debugPrint();
}

/**
 * @brief Return barycenter of point3d
 * 
 * @return Coord3d 
 */
Coord3d Point3d::getBarycenter() const
{
    return this->p0;
}

bool Point3d::draw(ThreeFX_Fixed &tfx) const
{
    return tfx.drawWorldPoint3d(*this);
}
