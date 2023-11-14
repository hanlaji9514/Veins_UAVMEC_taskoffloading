#ifndef CIRCLE_MOBILITY_H
#define CIRCLE_MOBILITY_H

#include "veins/base/modules/BaseMobility.h"


// developed from linear mobility, the example
namespace veins{

class VEINS_API CircleMobility : public BaseMobility{
    protected:
        double angle;
        double R;
        Coord center;       // input: only use x & y to define a 2-D circle
        Coord stepTarget;

    public:
        virtual void initialize(int);
        virtual double arc2deg(double, Coord, Coord);

    protected:
        virtual void makeMove();
        virtual void fixIfHostGetsOutside();

};

#endif
}
