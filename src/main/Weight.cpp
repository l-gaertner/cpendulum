#include "Weight.hpp"
#include "Pendulum.hpp"
#include <stdexcept>

Coordinate Coordinate::add(Coordinate val) {
    return Coordinate{x + val.x, y + val.y};
}

double sin_90_deg(double angle) {
    return sin(angle - M_PI/2);
}

double cos_90_deg(double angle) {
    return cos(angle - M_PI/2);
}

Coordinate Weight::position() {
    return Coordinate{length * cos_90_deg(angularPosition), length * sin_90_deg(angularPosition)};
}

Weight::Weight(double angle, double l) {
    angularPosition = angle;
    angularVelocity = 0.0;
    mass = 0.5;
    length = l;
}

Weight::Weight(double angle, double velocity, double m, double l) {
    angularPosition = angle;
    angularVelocity = velocity;
    mass = m;
    length = l;
}

Weight Weight::of(double angularPosition, double angularVelocity) {
    return Weight(angularPosition, angularVelocity, mass, length);
}
