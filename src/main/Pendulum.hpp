#ifndef PENDULUM_H
#define PENDULUM_H

#include <utility>
#include "Weight.hpp"

#define GRAVITATIONAL_CONSTANT 9.80665f;

struct PendulumEnergy {
    double kin;
    double pot;

    double total();
};

class Pendulum {
    private:
        Coordinate pivot = Coordinate{0.0, 2.0};
        Weight weight1 = Weight(0.0, 0.0);
        Weight weight2 = Weight(0.0, 0.0);

    public:
        Coordinate getPivot();
        Coordinate getWeight1();
        Coordinate getWeight2();
        void init(int w1, int w2);
        PendulumEnergy energy();
        void calculateAndApplyForcesEulerCromer(unsigned int durationMillis);
        void calculateAndApplyForcesRungeKutta(unsigned int durationMillis);
};
#endif
