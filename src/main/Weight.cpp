#include "Weight.hpp"

Coordinate Coordinate::add(Coordinate val) {
    return Coordinate{x + val.x, y + val.y};
}

float sin_90_deg(float angle) {
    return sin(angle + M_PI/2);
}

float cos_90_deg(float angle) {
    return cos(angle + M_PI/2);
}

Coordinate Weight::position() {
    return Coordinate{length * cos_90_deg(angularPosition), length * sin_90_deg(angularPosition)};
}

Weight::Weight(float angle, float l) {
    angularPosition = angle;
    angularVelocity = 0.0;
    mass = 0.5;
    length = l;
}

void Weight::update(int durationMillis) {
}

float Weight::energy() {
    float potentialEnergy = position().y * mass;
    float kineticEnergy = 0.0;

    return potentialEnergy + kineticEnergy;
}
