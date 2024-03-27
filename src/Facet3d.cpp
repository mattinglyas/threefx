#include <DebugLog.h>
#include "ThreeFX.hpp"
#include <Arduino_GFX_Library.h>
#include <libfixmath/fix16.hpp>

/**
 * @brief Construct a new Facet 3d:: Facet 3d object
 *
 */
Facet3d::Facet3d()
{
    this->color = DEFAULT_COLOR;
    this->p0 = Coord3d();
    this->p1 = Coord3d();
    this->p2 = Coord3d();
}

Facet3d::Facet3d(const Coord3d &p0, const Coord3d &p1, const Coord3d &p2) : Facet3d()
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
}

Facet3d::Facet3d(const Coord3d &p0, const Coord3d &p1, const Coord3d &p2, const uint16_t color) : Facet3d(p0, p1, p2)
{
    this->color = color;
}

/**
 * @brief Copy construct a new Facet 3d:: Facet 3d object
 *
 * @param l
 */
Facet3d::Facet3d(const Facet3d &f)
{
    this->p0 = f.p0;
    this->p1 = f.p1;
    this->p2 = f.p2;
}

/**
 * @brief Assignment operator
 *
 * @param l other Facet3d
 * @return const Facet3d& l
 */
const Facet3d &Facet3d::operator=(const Facet3d &f)
{
    this->p0 = f.p0;
    this->p1 = f.p1;
    this->p2 = f.p2;

    return f;
}

/**
 * @brief Destructor for Facet3d
 *
 */
Facet3d::~Facet3d() {}

/**
 * @brief Prints self using debug_print and debug_println
 *
 */
void Facet3d::debugPrint() const
{
    this->p0.debugPrint();
    PRINT(" ,");
    this->p1.debugPrint();
    PRINT(" ,");
    this->p2.debugPrint();
}

/**
 * @brief Return barycenter of Facet3d
 *
 * @return Coord3d
 */
Coord3d Facet3d::getBarycenter() const
{
    Coord3d bc;
    bc.x = (this->p0.x + this->p1.x + this->p2.x) / 3;
    bc.y = (this->p0.y + this->p1.y + this->p2.y) / 3;
    bc.z = (this->p0.z + this->p1.z + this->p2.z) / 3;
    return bc;
}

bool Facet3d::draw(ThreeFX_Fixed &tfx) const
{
    return tfx.drawWorldFacet3d(*this);
}
