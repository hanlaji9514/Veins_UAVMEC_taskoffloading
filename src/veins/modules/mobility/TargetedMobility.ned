//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.modules.mobility;

import org.car2x.veins.base.modules.BaseMobility;

simple TargetedMobility extends BaseMobility
{
    parameters:
        @class(veins::TargetedMobility);
        double speed @unit(mps); // speed of the host (in m/s)
        double angle @unit(deg); // angle of linear motion (degreees)
        double updateInterval @unit(s); // time interval to update the hosts position (seconds)
        double destinationX @unit(m);
        double destinationY @unit(m);
}