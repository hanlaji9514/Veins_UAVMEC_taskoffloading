package org.car2x.veins.nodes;

import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;


module UAV
{
    parameters:
        string applType;
        string nicType = default("Nic80211p");
        string mobilityType = default("org.car2x.veins.modules.mobility.CircleMobility");
    gates:
        input veinsradioIn;		// gate for sendDirect? 
    submodules:
        appl: <applType> like org.car2x.veins.base.modules.IBaseApplLayer {
            parameters:
            	@display("p=60,50");
        }
        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=60,166");
        }
        mobility: <mobilityType> like org.car2x.veins.base.modules.IMobility { 	// write a mobility file like LinearMobility
            parameters:
                @display("p=130,172;i=block/cogwheel");
        }
        
    connections:
        nic.upperLayerOut --> appl.lowerLayerIn;
        nic.upperLayerIn <-- appl.lowerLayerOut;
        nic.upperControlOut --> appl.lowerControlIn;
        nic.upperControlIn <-- appl.lowerControlOut;
        
        veinsradioIn --> nic.radioIn;
        
}