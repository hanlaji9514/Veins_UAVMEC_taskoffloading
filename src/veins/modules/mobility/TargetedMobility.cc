//
// Copyright (C) 2005 Emin Ilker Cetinbas
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

// Author: Emin Ilker Cetinbas (niw3_at_yahoo_d0t_com)

#include "veins/modules/mobility/TargetedMobility.h"
#include "veins/modules/mobility/LinearMobility.h"
#include "veins/veins.h"

using namespace veins;

Define_Module(veins::TargetedMobility);

void TargetedMobility::initialize(int stage)
{
    BaseMobility::initialize(stage);

    EV_TRACE << "initializing TargetedMobility stage " << stage << endl;

    if (stage == 0)
    {
        move.setSpeed(par("speed").doubleValue());
        destination.x = par("destinationX");
        destination.y = par("destinationY");
    }
    else if (stage == 1)
    {
        stepTarget = move.getStartPos();
    }
}

void TargetedMobility::fixIfHostGetsOutside()
{
    Coord dummy = Coord::ZERO;
    handleIfOutside(WRAP, stepTarget, dummy, dummy, angle);
}

void TargetedMobility::updateDestination(const Coord& newDestination)
{
    EV_INFO << "My Destination changed to : " << newDestination << endl;
    destination = newDestination;
    move.setSpeed(par("speed").doubleValue());
}

void TargetedMobility::makeMove()
{
    EV_INFO << "start makeMove " << move.info() << endl;

    move.setStart(stepTarget, simTime());

    double dx = destination.x - move.getStartPos().x;
    double dy = destination.y - move.getStartPos().y;
    double distance = sqrt(dx*dx + dy*dy);

    if (distance <= move.getSpeed() * SIMTIME_DBL(updateInterval))
    {
        // We have reached the destination
        stepTarget = destination;
        //move.setSpeed(0);
        move.setStart(stepTarget, simTime()); // 能夠抵達目的地，直接移至目的地
    }
    else
    {
        // Move towards the destination
        double angle = atan2(dy, dx);
        stepTarget.x = move.getStartPos().x + move.getSpeed() * cos(angle) * SIMTIME_DBL(updateInterval);
        stepTarget.y = move.getStartPos().y + move.getSpeed() * sin(angle) * SIMTIME_DBL(updateInterval);
        move.setDirectionByTarget(stepTarget);
    }

    fixIfHostGetsOutside();

    EV_INFO << "My Destination : " << destination << " / speed : " << move.getSpeed();
    EV_INFO << " new stepTarget: " << stepTarget.info() << endl;

}
