#include "ThreeFX.hpp"
#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_Coord2d_constructor(void)
{
    Coord2d p0 = Coord2d(15., 20.);
    TEST_ASSERT_EQUAL(p0.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p0.y, Fix16(20.));

    Coord2d p1 = Coord2d(p0);
    TEST_ASSERT_EQUAL(p1.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p1.y, Fix16(20.));

    Coord2d p2 = p1;    
    TEST_ASSERT_EQUAL(p2.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p2.y, Fix16(20.));
}

void test_Coord3d_constructor(void)
{
    Coord3d p0 = Coord3d(15., 20., -12.);
    TEST_ASSERT_EQUAL(p0.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p0.y, Fix16(20.));
    TEST_ASSERT_EQUAL(p0.z, Fix16(-12.));

    Coord3d p1 = Coord3d(p0);
    TEST_ASSERT_EQUAL(p1.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p1.y, Fix16(20.));
    TEST_ASSERT_EQUAL(p1.z, Fix16(-12.));

    Coord3d p2 = p1;    
    TEST_ASSERT_EQUAL(p2.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p2.y, Fix16(20.));
    TEST_ASSERT_EQUAL(p2.z, Fix16(-12.));
}


void setup()
{
    UNITY_BEGIN();

    RUN_TEST(test_Coord2d_constructor);

    UNITY_END();
}

void loop() {}