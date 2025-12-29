#ifndef WEIGHT_H
#define WEIGHT_H

#include <cmath>

struct Coordinate {
    public:
        float x,y;
        Coordinate add(Coordinate val);

};

struct Weight {
    public:
        float angularPosition;
        float angularVelocity;
        float mass;
        Weight(float angularPosition);
        Coordinate position();
        float energy();
        void update(int durationMillis);
};

#endif
