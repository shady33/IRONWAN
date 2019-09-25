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

#include "NeighbourTalker.h"
#include "inet/networklayer/ipv4/IPv4Datagram.h"
#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "../misc/cSimulinkRTScheduler.h"

namespace inet {

Define_Module(NeighbourTalker);

NeighbourTalker::~NeighbourTalker()
{
    
}

void NeighbourTalker::initialize(int stage)
{
    if (stage == 0) {

    }else if (stage == INITSTAGE_APPLICATION_LAYER){
    
    }
}

void NeighbourTalker::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        delete msg;
    }else if(msg->isSelfMessage()){
        EV << "Self message in GW" << endl;
    }   
}

void NeighbourTalker::finish()
{

}

} //namespace inet
