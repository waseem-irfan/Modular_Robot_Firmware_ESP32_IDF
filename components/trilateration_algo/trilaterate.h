#ifndef TRILATERATE_H
#define TRILATERATE_H


struct Anchor{
    float x, y, z;
    double distance;
};

struct Position
{
    float x, y, z;
};

bool trilateration(const Anchor& A1, const Anchor& A2, const Anchor& A3, Position& result);

#endif