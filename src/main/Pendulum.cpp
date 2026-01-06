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
    double acc_1_t3_2 = weight2.angularVelocity * weight2.angularVelocity * weight2.length + weight1.angularVelocity * weight1.angularVelocity * weight1.length * cos(delta_angle);
    double acc_1_t3 = -2 * sin(delta_angle) * weight2.mass * (acc_1_t3_2);
    double acc_1_t4 = weight1.length * (2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * delta_angle));

    return (acc_1_t1 + acc_1_t2 + acc_1_t3) / acc_1_t4;

}

double weight_2_acceleration(Weight weight1, Weight weight2) {
    double delta_angle = weight1.angularPosition - weight2.angularPosition;
    double g = GRAVITATIONAL_CONSTANT;

    double acc_2_t1 = weight1.angularVelocity * weight1.angularVelocity * weight1.length * (weight1.mass  + weight2.mass);
    double acc_2_t2 = g * (weight1.mass + weight2.mass) * cos(weight1.angularPosition);
    double acc_2_t3 = weight2.angularVelocity * weight2.angularVelocity * weight2.length * weight2.mass * cos(delta_angle);
    double acc_2_t4 = weight2.length * (2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * delta_angle));

    return (2 * sin(delta_angle) * (acc_2_t1 + acc_2_t2 + acc_2_t3))/ acc_2_t4;

}


void Pendulum::calculateAndApplyForcesEulerCromer(unsigned int durationMillis) {
    double h = ((double) durationMillis / 1000.0);

    double acceleration_w1 = weight_1_acceleration(weight1, weight2);
    double angularVelocity_w1 = weight1.angularVelocity + acceleration_w1 * h;
    double angularPosition_w1 = weight1.angularPosition + angularVelocity_w1 * h;

    double acceleration_w2 = weight_2_acceleration(weight1, weight2);
    double angularVelocity_w2 = weight2.angularVelocity + acceleration_w2 * h;
    double angularPosition_w2 = weight2.angularPosition + angularVelocity_w2 * h;

    weight1.angularVelocity = angularVelocity_w1;
    weight1.angularPosition = angularPosition_w1;

    weight2.angularVelocity = angularVelocity_w2;
    weight2.angularPosition = angularPosition_w2;
}

void Pendulum::calculateAndApplyForcesRungeKutta(unsigned int durationMillis) {
    double h = ((double) durationMillis / 1000.0);

    // k = velocity slope, l = position slope
    // k1
    double k1_w1 = h * weight_1_acceleration(weight1, weight2);
    double l1_w1 = h * weight1.angularVelocity;
    double k1_w2 = h * weight_2_acceleration(weight1, weight2);
    double l1_w2 = h * weight2.angularVelocity;

    //k2
    Weight weight1_t1 = weight1.of(weight1.angularPosition + 0.5 * l1_w1, weight1.angularVelocity + 0.5 * k1_w1);
    Weight weight2_t1 = weight2.of(weight2.angularPosition + 0.5 * l1_w2, weight2.angularVelocity + 0.5 * k1_w2);

    double k2_w1 = h * weight_1_acceleration(weight1_t1, weight2_t1);
    double l2_w1 = h * (weight1_t1.angularVelocity);
    double k2_w2 = h * weight_2_acceleration(weight1_t1, weight2_t1);
    double l2_w2 = h * (weight2_t1.angularVelocity);

    //k3
    Weight weight1_t2 = weight1.of(weight1.angularPosition + 0.5 * l2_w1, weight1.angularVelocity + 0.5 * k2_w1);
    Weight weight2_t2 = weight2.of(weight2.angularPosition + 0.5 * l2_w2, weight2.angularVelocity + 0.5 * k2_w2);

    double k3_w1 = h * weight_1_acceleration(weight1_t2, weight2_t2);
    double l3_w1 = h * (weight1_t2.angularVelocity);
    double k3_w2 = h * weight_2_acceleration(weight1_t2, weight2_t2);
    double l3_w2 = h * (weight2_t2.angularVelocity);

    //k4
    Weight weight1_t3 = weight1.of(weight1.angularPosition + l3_w1, weight1.angularVelocity + k3_w1);
    Weight weight2_t3 = weight2.of(weight2.angularPosition + l3_w2, weight2.angularVelocity + k3_w2);

    double k4_w1 = h * weight_1_acceleration(weight1_t3, weight2_t3);
    double l4_w1 = h * (weight1_t3.angularVelocity);
    double k4_w2 = h * weight_2_acceleration(weight1_t3, weight2_t3);
    double l4_w2 = h * (weight2_t3.angularVelocity);

    //y_{n+1}
    weight1.angularVelocity = weight1.angularVelocity + 1.0/6.0 * (k1_w1 + 2 * k2_w1 + 2 * k3_w1 + k4_w1);
    weight1.angularPosition = weight1.angularPosition + 1.0/6.0 * (l1_w1 + 2 * l2_w1 + 2 * l3_w1 + l4_w1);

    weight2.angularVelocity = weight2.angularVelocity + 1.0/6.0 * (k1_w2 + 2 * k2_w2 + 2 * k3_w2 + k4_w2);
    weight2.angularPosition = weight2.angularPosition + 1.0/6.0 * (l1_w2 + 2 * l2_w2 + 2 * l3_w2 + l4_w2);
}
