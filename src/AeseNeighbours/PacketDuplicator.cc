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

#include "inet/common/LayeredProtocolBase.h"
#include "inet/common/ModuleAccess.h"
#include "PacketDuplicator.h"

namespace inet {

Define_Module(PacketDuplicator);

void PacketDuplicator::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL) {
        // number of ports
        numPorts = gate("layerOut", 0)->size();
        if (gate("layerIn", 0)->size() != numPorts)
            throw cRuntimeError("the sizes of the layerOut[] and layerIn[] gate vectors must be the same");
    }
}

void PacketDuplicator::handleMessage(cMessage *msg)
{
    // if(msg->arrivedOn("lowerLayerIn"))
    broadcastFrame(msg);
    // else
        // send(msg,"lowerLayerOut");
}

void PacketDuplicator::broadcastFrame(cMessage *msg)
{
    int idx = (msg->getArrivalGate())->getIndex();
    for (int i = 0; i < numPorts; ++i) {
        if(i != idx){
            cMessage *clone = msg->dup();
            if (msg->getControlInfo())
                clone->setControlInfo((msg->getControlInfo())->dup());
            send(clone, "layerOut", i);
        }
    }
    delete msg;
}

// bool PacketDuplicator::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
// {
//     Enter_Method_Silent();
//     return true;
// }

void PacketDuplicator::finish()
{

}

} // namespace inet

