#ifndef WEIGHT_H
#define WEIGHT_H

#include <cmath>

struct Coordinate {
    public:
        double x,y;
        Coordinate add(Coordinate val);

};

struct Weight {
    public:
        double angularPosition;
        double angularVelocity;
        double mass;
        double length;

        Weight(double angularPosition, double length);
        Coordinate position();
        void update(int durationMillis);
};

#endif
