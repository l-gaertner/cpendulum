#include "Weight.hpp"

Coordinate Coordinate::add(Coordinate val) {
    return Coordinate{x + val.x, y + val.y};
}

Coordinate Weight::position() {
    return Coordinate{cos(angularPosition), sin(angularPosition)};
}

Weight::Weight(float angle) {
    angularPosition = angle;
    angularVelocity = 0.0;
    mass = 0.3;
}

void Weight::update(int durationMillis) {
}

float Weight::energy() {
    float potentialEnergy = position().y * mass;
    float kineticEnergy = 0.0;

    return potentialEnergy + kineticEnergy;
}
