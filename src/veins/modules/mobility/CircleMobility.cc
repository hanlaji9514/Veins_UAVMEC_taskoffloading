#include "veins/modules/mobility/CircleMobility.h"
#include "veins/veins.h"

using namespace veins;

Define_Module(veins::CircleMobility);

double CircleMobility::arc2deg(double angle, veins::Coord a, veins::Coord b)
{
    angle *= 180 / M_PI;
    if((b.y - a.y) * angle < 0 || (angle == 0 && b.x < a.x))
        angle += 180;             // make angle located in [0, 360]
    else if(angle < 0) angle += 360;
    return angle;
}

void CircleMobility::initialize(int stage)
{
    BaseMobility::initialize(stage);
    EV << "initializing Circle Mobility stage " << stage << endl;
    if(stage == 0)
    {
        move.setSpeed(par("speed").doubleValue());
        center.x = par("cx").doubleValue();
        center.y = par("cy").doubleValue();
        center.z = par("cz").doubleValue();
    }
    else if(stage == 1)
    {
        move.setSpeed(par("speed").doubleValue());
        //EV << "speed = " << move.getSpeed() << endl;
        stepTarget = move.getStartPos();
        // calculate angle and R from center and stepTarget here, test passed
        center.z = stepTarget.z;
        R = par("r").doubleValue();
        angle = arc2deg( atan2((center.y - stepTarget.y), (stepTarget.x - center.x)), center, stepTarget );
        if (angle < 0) angle += 2 * M_PI; // ensure angle is in [0, 2�]

    }
}

void CircleMobility::makeMove()
{
    //EV << "start makeMove " << move.info() <<endl;
    move.setStart(stepTarget, simTime());
    //EV << "current_position : " << move.getPositionAt(simTime()) << endl;

    double r = move.getSpeed() * updateInterval.dbl();
    angle += r / R * 180 / M_PI; // keep it in radian
    //EV << "angle = " << angle << endl;
    stepTarget.x = center.x + R * cos(angle * M_PI / 180.0);
    stepTarget.y = center.y + R * sin(angle * M_PI / 180.0);
    //move.setDirectionByVector(stepTarget - /*move.getStartPos()*/move.getPositionAt(simTime()));
    Coord direction = stepTarget - move.getPositionAt(simTime());
    if (direction.length() != 0)
    {
        direction /= direction.length();  // Normalize the direction vector
    }
    if (!math::almost_equal(direction.squareLength(), 1.0) && !math::almost_equal(direction.squareLength(), 0.0))
    {
        direction /= direction.length();
    }
    move.setDirectionByVector(direction);
    //EV << "new step target of circle: " << stepTarget.info() << endl;
    fixIfHostGetsOutside();
}

void CircleMobility::fixIfHostGetsOutside()
{
    Coord dummy = Coord::ZERO;
    handleIfOutside(REFLECT, stepTarget, dummy, dummy, angle);
}

