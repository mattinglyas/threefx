#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 */
Coord2d::Coord2d()
{
    this->x = Fix16();
    this->y = Fix16();
}

/**
 * @brief Copy construct a new Point 2d:: Point 2d object
 *
 * @param p other point
 */
Coord2d::Coord2d(const Coord2d &p)
{
    this->x = p.x;
    this->y = p.y;
}

/**
 * @brief Assignment operator for Coord2d
 *
 * @param p other point
 * @return const Coord2d& p
 */
const Coord2d &Coord2d::operator=(const Coord2d &p)
{
    this->x = p.x;
    this->y = p.y;

    return p;
}

/**
 * @brief Destructor for Coord2d
 *
 */
Coord2d::~Coord2d() {}

/**
 * @brief Construct a new Point 2d:: Point 2d object
 *
 * @param x x coordinate (double)
 * @param y y coordinate (double)
 */
Coord2d::Coord2d(double x, double y)
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
Coord2d::Coord2d(const Fix16 &x, const Fix16 &y)
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
Coord2d::Coord2d(int16_t x, int16_t y)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Coord2d::debugPrint() const
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
