#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 */
Coord3d::Coord3d()
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
Coord3d::Coord3d(const Coord3d &p)
{
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;
}

/**
 * @brief Assignment operator
 *
 * @param p other Coord3d
 * @return Coord3d&
 */
const Coord3d &Coord3d::operator=(const Coord3d &p)
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
Coord3d::~Coord3d() {}

/**
 * @brief Construct a new Point 3d:: Point 3d object
 *
 * @param x x coordinate (float)
 * @param y y coordinate (float)
 * @param z z coordinate (float)
 */
Coord3d::Coord3d(double x, double y, double z)
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
Coord3d::Coord3d(const Fix16 &x, const Fix16 &y, const Fix16 &z)
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
Coord3d::Coord3d(int16_t x, int16_t y, int16_t z)
{
    this->x = Fix16(x);
    this->y = Fix16(y);
    this->z = Fix16(z);
}

/**
 * @brief Relative adder operator for point3d
 *
 * @param rhs
 * @return Coord3d&
 */
Coord3d &Coord3d::operator+=(const Coord3d &rhs)
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
 * @return Coord3d&
 */
const Coord3d Coord3d::operator+(const Coord3d &rhs)
{
    Coord3d ret = *this;
    ret.x += rhs.x;
    ret.y += rhs.y;
    ret.z += rhs.z;

    return ret;
}

/**
 * @brief Relative subtractor operator for point3d
 *
 * @param rhs
 * @return Coord3d&
 */
Coord3d &Coord3d::operator-=(const Coord3d &rhs)
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
 * @return Coord3d&
 */
const Coord3d Coord3d::operator-(const Coord3d &rhs)
{
    Coord3d ret = *this;
    ret.x -= rhs.x;
    ret.y -= rhs.y;
    ret.z -= rhs.z;

    return ret;
}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Coord3d::debugPrint() const
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
