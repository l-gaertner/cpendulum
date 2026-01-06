#include <gtest/gtest.h>
#include "../main/Pendulum.hpp"

#define ABS_ERROR 0.000001

TEST(SimulationPendulumTest, InitialState) {
    Pendulum pendulum;

    pendulum.init(90, 90);

    auto energy = pendulum.energy().total();
    ASSERT_TRUE(energy > 0.0);
}

TEST(SimulationPendulumTest, StateAfterOneIteration) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy().total();

    pendulum.calculateAndApplyForcesEulerCromer(2);

    auto e1 = pendulum.energy().total();

    float small_error = 0.001;
    ASSERT_NEAR(e0, e1, small_error);
}

TEST(SimulationPendulumTest, StateAfter20Iterations) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy().total();

    for (int a = 0; a < 20; a++)
        pendulum.calculateAndApplyForcesEulerCromer(1);

    auto e1 = pendulum.energy().total();

    float small_error = 0.001;
    ASSERT_NEAR(e0, e1, small_error);
}

TEST(SimulationPendulumTest, DownDownZeroKineticEnergy) {
    Pendulum pendulum;

    pendulum.init(0.0, 0.0);

    pendulum.calculateAndApplyForcesEulerCromer(10);
    pendulum.calculateAndApplyForcesEulerCromer(10);
    pendulum.calculateAndApplyForcesEulerCromer(10);
    pendulum.calculateAndApplyForcesEulerCromer(10);
    pendulum.calculateAndApplyForcesEulerCromer(10);

    ASSERT_NEAR(pendulum.energy().kin, 0.0, 0.001);
}

TEST(SimulationPendulumTest, StateAfter100IterationsEuler) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy().total();

    for (int a = 0; a < 100; a++)
        pendulum.calculateAndApplyForcesEulerCromer(1);

    auto e1 = pendulum.energy().total();

    float small_error = 0.01;
    ASSERT_NEAR(e0, e1, small_error);
}

TEST(SimulationPendulumTest, StateAfterOneIterationRungeKutta) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy().total();

    pendulum.calculateAndApplyForcesRungeKutta(1);

    auto e1 = pendulum.energy().total();

    float small_error = 0.001;
    ASSERT_NEAR(e0, e1, small_error);
}

TEST(SimulationPendulumTest, StateAfter100IterationsRungeKutta) {
    Pendulum pendulum;
    pendulum.init(90, 90);
    auto e0 = pendulum.energy().total();

    for (int a = 0; a < 100; a++)
        pendulum.calculateAndApplyForcesRungeKutta(1);

    auto e1 = pendulum.energy().total();

    float small_error = 0.001;
    ASSERT_NEAR(e0, e1, small_error);
}
