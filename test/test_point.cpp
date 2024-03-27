#include "3fx_fixed.hpp"
#include "3fx_fixed.cpp"
#include <unity.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_point2d_constructor(void)
{
    Point2d p0 = Point2d(15., 20.);
    TEST_ASSERT_EQUAL(p0.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p0.y, Fix16(20.));

    Point2d p1 = Point2d(p0);
    TEST_ASSERT_EQUAL(p1.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p1.y, Fix16(20.));

    Point2d p2 = p1;    
    TEST_ASSERT_EQUAL(p2.x, Fix16(15.));
    TEST_ASSERT_EQUAL(p2.y, Fix16(20.));
}

void setup()
{
    UNITY_BEGIN();

    RUN_TEST(test_point2d_constructor);

    UNITY_END();
}

void loop() {}