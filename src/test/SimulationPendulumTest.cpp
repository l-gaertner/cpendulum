#include <gtest/gtest.h>
#include "../main/Pendulum.hpp"

#define ABS_ERROR 0.000001

TEST(SimulationPendulumTest, InitialState) {
    Pendulum pendulum;

    pendulum.init(90, 90);

    auto energy = pendulum.energy();
    ASSERT_TRUE(energy > 0.0);
}

TEST(SimulationPendulumTest, StateAfterOneIteration) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy();

    pendulum.calculateAndApplyForces(1000);

    auto e1 = pendulum.energy();

    ASSERT_NEAR(e0, e1, ABS_ERROR);
}
