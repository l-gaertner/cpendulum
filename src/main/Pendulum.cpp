#include "Pendulum.hpp"
#include <cmath>

Coordinate Pendulum::getPivot() {
    return pivot;
}

Coordinate Pendulum::getWeight1() {
    return pivot.add(weight1.position());
}

// Coordinate Pendulum::getWeight2() {
//     return pivot.add(weight1.position().add(weight2.position()));
// }

void Pendulum::init(int deg_w1, int deg_w2) {
    float rad_w1 = ((float) deg_w1) * M_PI / 180.0;
    // float rad_w2 = ((float) deg_w2) * M_PI / 180.0;

    weight1 = Weight(rad_w1);
    // weight2 = Weight(rad_w2);
}

int Pendulum::energy() {
    // return weight1.energy() + weight2.energy();
    return weight1.energy();
}

float sin_90_deg(float angle) {
    return sin(angle - M_PI/2);
}

void Pendulum::calculateAndApplyForces(unsigned int durationMillis) {
    // d^2 * angularPosition / dt^2 = -g/L * sin(angularPosition)
    // angular_velocity = d/dt * angularPosition


    // euler-cromer
    auto h = ((float) durationMillis / 1000.0);

    auto acceleration_w1 = (-1 * weight1.mass * (9.81 / 1) * sin_90_deg(weight1.angularPosition) * h);
    auto angularVelocity_w1 = weight1.angularVelocity + acceleration_w1;
    auto angularPosition_w1 = weight1.angularPosition + angularVelocity_w1 * h;

    weight1.angularVelocity = angularVelocity_w1;
    weight1.angularPosition = angularPosition_w1;
}
