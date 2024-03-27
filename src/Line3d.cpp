#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Line 3d:: Line 3d object
 *
 */
Line3d::Line3d()
{
    this->color = DEFAULT_COLOR; // obvious default color
    this->p0 = Coord3d();
    this->p1 = Coord3d();
}

Line3d::Line3d(const Coord3d &p0, const Coord3d &p1) : Line3d()
{
    this->p0 = p0;
    this->p1 = p1;
}

Line3d::Line3d(const Coord3d &p0, const Coord3d &p1, const uint16_t color) : Line3d(p0, p1)
{
    this->color = color;
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
 * @brief Prints self using debug_print and debug_println
 *
 */
void Line3d::debugPrint() const
{
    this->p0.debugPrint();
    PRINT(" ,");
    this->p1.debugPrint();
}

/**
 * @brief Return barycenter of line3d
 * 
 * @return Coord3d 
 */
Coord3d Line3d::getBarycenter() const
{
    Coord3d bc;
    bc.x = (this->p0.x + this->p1.x) / 2;
    bc.y = (this->p0.y + this->p1.y) / 2;
    return bc;
}

bool Line3d::draw(ThreeFX_Fixed &tfx) const
{
    return tfx.drawWorldLine3d(*this);
}
