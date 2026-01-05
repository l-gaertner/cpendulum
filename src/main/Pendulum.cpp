#include "Pendulum.hpp"
#include <cmath>

Coordinate Pendulum::getPivot() {
    return pivot;
}

Coordinate Pendulum::getWeight1() {
    return pivot.add(weight1.position());
}

 Coordinate Pendulum::getWeight2() {
     return pivot.add(weight1.position().add(weight2.position()));
 }

void Pendulum::init(int deg_w1, int deg_w2) {
    double rad_w1 = ((double) deg_w1) * M_PI / 180.0;
    double rad_w2 = ((double) deg_w2) * M_PI / 180.0;

    weight1 = Weight(rad_w1, 1.0);
    weight2 = Weight(rad_w2, 0.9);
}

PendulumEnergy Pendulum::energy() {
    double g = GRAVITATIONAL_CONSTANT;

    double v1 = weight1.angularVelocity * weight1.length;
    double sq_v1 = v1 * v1;
    double w1_kin = 0.5 * weight1.mass * sq_v1;

    double delta_angle = weight1.angularPosition - weight2.angularPosition;

    double isolated_v2 = weight2.angularVelocity * weight2.length;
    double sq_isolated_v2 = isolated_v2 * isolated_v2;
    double extra_v2 = 2 * weight1.length * weight2.length * weight1.angularVelocity * weight2.angularVelocity * cos(delta_angle);
    double sq_v2 = sq_v1 + sq_isolated_v2 + extra_v2;
    double w2_kin = 0.5 * weight2.mass * sq_v2;

    double w1_pot = getWeight1().y * weight1.mass * g;
    double w2_pot = getWeight2().y * weight2.mass * g;

    return PendulumEnergy{
        w1_kin + w2_kin,
        w1_pot + w2_pot,
    };
}

double PendulumEnergy::total() {
    return kin + pot;
}

double weight_1_acceleration(Weight weight1, Weight weight2) {
    double delta_angle = weight1.angularPosition - weight2.angularPosition;
    double g = GRAVITATIONAL_CONSTANT;

    double acc_1_t1 = -1 * g * (2 * weight1.mass + weight2.mass) * sin(weight1.angularPosition);
    double acc_1_t2 = -1 * weight2.mass * g * sin(weight1.angularPosition - 2 * weight2.angularPosition);
    double acc_1_t3 = -2 * sin(delta_angle) * weight2.mass * (weight2.angularVelocity * weight2.angularVelocity * weight2.length + weight1.angularVelocity * weight1.angularVelocity * weight1.length * cos(delta_angle));
    double acc_1_t4 = weight1.length * (2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * weight1.angularPosition - 2 * weight2.angularPosition));

    return (acc_1_t1 + acc_1_t2 + acc_1_t3) / acc_1_t4;

}

double weight_2_acceleration(Weight weight1, Weight weight2) {
    double delta_angle = weight1.angularPosition - weight2.angularPosition;
    double g = GRAVITATIONAL_CONSTANT;

    double acc_2_t1 = weight1.angularVelocity * weight1.angularVelocity * weight1.length * (weight1.mass  + weight2.mass);
    double acc_2_t2 = g * (weight1.mass + weight2.mass) * cos(weight1.angularPosition);
    double acc_2_t3 = weight2.angularVelocity * weight2.angularVelocity * weight2.length * weight2.mass * cos(delta_angle);
    double acc_2_t4 = 2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * weight1.angularPosition - 2 * weight2.angularPosition);

    return (2 * sin(delta_angle) * (acc_2_t1 + acc_2_t2 + acc_2_t3))/(weight2.length * acc_2_t4);

}


void Pendulum::calculateAndApplyForcesEulerCromer(unsigned int durationMillis) {
    auto h = ((double) durationMillis / 1000.0);

    auto acceleration_w1 = weight_1_acceleration(weight1, weight2);
    auto angularVelocity_w1 = weight1.angularVelocity + acceleration_w1 * h;
    auto angularPosition_w1 = weight1.angularPosition + angularVelocity_w1 * h;

    auto acceleration_w2 = weight_2_acceleration(weight1, weight2);
    auto angularVelocity_w2 = weight2.angularVelocity + acceleration_w2 * h;
    auto angularPosition_w2 = weight2.angularPosition + angularVelocity_w2 * h;

    weight1.angularVelocity = angularVelocity_w1;
    weight1.angularPosition = angularPosition_w1;

    weight2.angularVelocity = angularVelocity_w2;
    weight2.angularPosition = angularPosition_w2;
}

void Pendulum::calculateAndApplyForcesRungeKutta(unsigned int durationMillis) {
    auto h = ((double) durationMillis / 1000.0);

    // k1
    auto k1_w1_a = weight_1_acceleration(weight1, weight2);
    auto k1_w1_v = weight1.angularVelocity + k1_w1_a * h;
    auto k1_w1_theta = weight1.angularPosition + k1_w1_v * h;
    Weight k1_w1 = weight1.of(k1_w1_theta, k1_w1_v);

    auto k1_w2_a = weight_2_acceleration(weight1, weight2);
    auto k1_w2_v = weight2.angularVelocity + k1_w2_a * h;
    auto k1_w2_theta = weight2.angularPosition + k1_w2_v * h;
    Weight k1_w2 = weight2.of(k1_w2_theta, k1_w2_v);

    //k2
    auto k2_w1_a = weight_1_acceleration(k1_w1, k1_w2);
    auto k2_w1_v = weight1.angularVelocity + k2_w1_a * h/2;
    auto k2_w1_theta = weight1.angularPosition + k2_w1_v * h/2;
    Weight k2_w1 = weight1.of(k2_w1_theta, k2_w1_v);

    auto k2_w2_a = weight_2_acceleration(k1_w1, k1_w2);
    auto k2_w2_v = weight2.angularVelocity + k2_w2_a * h/2;
    auto k2_w2_theta = weight2.angularPosition + k2_w2_v * h/2;
    Weight k2_w2 = weight2.of(k2_w2_theta, k2_w2_v);

    //k3
    auto k3_w1_a = weight_1_acceleration(k2_w1, k2_w2);
    auto k3_w1_v = weight1.angularVelocity + k3_w1_a * h/2;
    auto k3_w1_theta = weight1.angularPosition + k3_w1_v * h/2;
    Weight k3_w1 = weight1.of(k3_w1_theta, k3_w1_v);

    auto k3_w2_a = weight_2_acceleration(k2_w1, k2_w2);
    auto k3_w2_v = weight2.angularVelocity + k3_w2_a * h/2;
    auto k3_w2_theta = weight2.angularPosition + k3_w2_v * h/2;
    Weight k3_w2 = weight2.of(k3_w2_theta, k3_w2_v);

    //k4
    auto k4_w1_a = weight_1_acceleration(k3_w1, k3_w2);
    auto k4_w1_v = weight1.angularVelocity + k4_w1_a * h;
    auto k4_w1_theta = weight1.angularPosition + k4_w1_v * h;
    Weight k4_w1 = weight1.of(k4_w1_theta, k4_w1_v);

    auto k4_w2_a = weight_2_acceleration(k3_w1, k3_w2);
    auto k4_w2_v = weight2.angularVelocity + k4_w2_a * h;
    auto k4_w2_theta = weight2.angularPosition + k4_w2_v * h;
    Weight k4_w2 = weight2.of(k4_w2_theta, k4_w2_v);

    //y_{n+1}

    weight1.angularVelocity = weight1.angularVelocity + 1.0/6.0 * h * (k1_w1_a + 2 * k2_w1_a + 2 * k3_w1_a + k4_w1_a);
    weight1.angularPosition = weight1.angularPosition + 1.0/6.0 * h * (k1_w1_v + 2 * k2_w1_v + 2 * k3_w1_v + k4_w1_v);

    weight2.angularVelocity = weight2.angularVelocity + 1.0/6.0 * h * (k1_w2_a + 2 * k2_w2_a + 2 * k3_w2_a + k4_w2_a);
    weight2.angularPosition = weight2.angularPosition + 1.0/6.0 * h * (k1_w2_v + 2 * k2_w2_v + 2 * k3_w2_v + k4_w2_v);
}
