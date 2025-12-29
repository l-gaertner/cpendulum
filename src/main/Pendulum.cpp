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
    float rad_w1 = ((float) deg_w1) * M_PI / 180.0;
    float rad_w2 = ((float) deg_w2) * M_PI / 180.0;

    weight1 = Weight(rad_w1, 1.0);
    weight2 = Weight(rad_w2, 0.9);
}

int Pendulum::energy() {
    return weight1.energy() + weight2.energy();
}

void Pendulum::calculateAndApplyForces(unsigned int durationMillis) {
    // euler-cromer
    auto h = ((float) durationMillis / 1000.0);

    float g = 9.81;
    float delta_angle = weight1.angularPosition - weight2.angularPosition;

    float acc_1_t1 = -1 * g * (2 * weight1.mass + weight2.mass) * sin(weight1.angularPosition);
    float acc_1_t2 = -1 * weight2.mass * g * sin(weight1.angularPosition - 2 * weight2.angularPosition);
    float acc_1_t3 = -2 * sin(delta_angle) * weight2.mass * (weight2.angularVelocity * weight2.angularVelocity * weight2.length + weight1.angularVelocity * weight1.angularVelocity * weight1.length * cos(delta_angle));
    float acc_1_t4 = weight1.length * (2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * weight1.angularPosition - 2 * weight2.angularPosition));

    auto acceleration_w1 = (acc_1_t1 + acc_1_t2 + acc_1_t3) / acc_1_t4;
    auto angularVelocity_w1 = weight1.angularVelocity + acceleration_w1 * h;
    auto angularPosition_w1 = weight1.angularPosition + angularVelocity_w1 * h;

    float acc_2_t1 = weight1.angularVelocity * weight1.angularVelocity * weight1.length * (weight1.mass  + weight2.mass);
    float acc_2_t2 = g * (weight1.mass + weight2.mass) * cos(weight1.angularPosition);
    float acc_2_t3 = weight2.angularVelocity * weight2.angularVelocity * weight2.length * weight2.mass * cos(delta_angle);
    float acc_2_t4 = 2 * weight1.mass + weight2.mass - weight2.mass * cos(2 * weight1.angularPosition - 2 * weight2.angularPosition);

    auto acceleration_w2 = (2 * sin(delta_angle) * (acc_2_t1 + acc_2_t2 + acc_2_t3))/(weight2.length * acc_2_t4);
    auto angularVelocity_w2 = weight2.angularVelocity + acceleration_w2 * h;
    auto angularPosition_w2 = weight2.angularPosition + angularVelocity_w2 * h;

    weight1.angularVelocity = angularVelocity_w1;
    weight1.angularPosition = angularPosition_w1;

    weight2.angularVelocity = angularVelocity_w2;
    weight2.angularPosition = angularPosition_w2;
}
