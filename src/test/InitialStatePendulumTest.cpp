#include <gtest/gtest.h>
#include "../main/Pendulum.hpp"

#define ABS_ERROR 0.000001

TEST(InitialStatePendulumTest, InitialState) {
    Pendulum pendulum;

    Coordinate center = pendulum.getPivot();
    ASSERT_NEAR(center.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(center.y, 2.0, ABS_ERROR);

    Coordinate weight_1 = pendulum.getWeight1();
    ASSERT_NEAR(weight_1.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_1.y, 2.0, ABS_ERROR);

    Coordinate weight_2 = pendulum.getWeight2();
    ASSERT_NEAR(weight_2.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_2.y, 2.0, ABS_ERROR);
}

TEST(InitialStatePendulumTest, InitStateUpUp) {
    Pendulum pendulum;
    
    pendulum.init(180, 180);

    Coordinate center = pendulum.getPivot();
    ASSERT_NEAR(center.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(center.y, 2.0, ABS_ERROR);

    Coordinate weight_1 = pendulum.getWeight1();
    ASSERT_NEAR(weight_1.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_1.y, center.y + 1.0, ABS_ERROR);

    Coordinate weight_2 = pendulum.getWeight2();
    ASSERT_NEAR(weight_2.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_2.y, center.y + 1.9, ABS_ERROR);
}

TEST(InitialStatePendulumTest, InitStateDownDown) {
    Pendulum pendulum;
    
    pendulum.init(0, 0);

    Coordinate center = pendulum.getPivot();
    ASSERT_NEAR(center.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(center.y, 2.0, ABS_ERROR);

    Coordinate weight_1 = pendulum.getWeight1();
    ASSERT_NEAR(weight_1.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_1.y, center.y - 1.0, ABS_ERROR);

    Coordinate weight_2 = pendulum.getWeight2();
    ASSERT_NEAR(weight_2.x, 0.0, ABS_ERROR);
    ASSERT_NEAR(weight_2.y, center.y - 1.9, ABS_ERROR);
}
