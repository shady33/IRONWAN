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


namespace inet {

Define_Module(NeighbourTalker);

NeighbourTalker::~NeighbourTalker()
{
    for (auto iter = ReceivedPacketsList->begin(); iter != ReceivedPacketsList->end(); ) {
        auto cur=iter++;
        delete cur->second.frame;
        ReceivedPacketsList->erase(cur);
    }
}

void NeighbourTalker::initialize(int stage)
{
    if (stage == 0) {
        AeseGWEnabled = par("AeseGWEnabled");
    }else if (stage == INITSTAGE_APPLICATION_LAYER){
        ReceivedPacketsList = new ReceivedPacketsMap();
    }
}

void NeighbourTalker::handleMessage(cMessage *msg)
{
    if (!AeseGWEnabled){
        delete msg;
        return;
    }

    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        handleLowerLayer(PK(msg));
    }else if(msg->isSelfMessage()){
        EV << "Self message in GW" << endl;
    }   
}

void NeighbourTalker::finish()
{

}

void NeighbourTalker::handleLowerLayer(cPacket* pkt)
{
    std::string className(pkt->getClassName());

    if(className.compare("inet::LoRaMacFrame") == 0)
        handleLoRaFrame(pkt);
    else
        delete pkt;
}

void NeighbourTalker::handleLoRaFrame(cPacket *pkt)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(pkt);
    if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS){
        // LAKSH: This is an uplink message from nodes for LoRa gateways 
        // or they are ping messages from neighbouring gateways.
        if(frame->getMsgType() == GW_PING_MESSAGE){
            // This gets added to Gateway neighbour table
            DevAddr txAddr = frame->getTransmitterAddress();
            auto iter = GatewayNeighboursList->find(txAddr);
            if(iter == GatewayNeighboursList->end()){
                // Not found in table, insert
                (*GatewayNeighboursList)[txAddr] = GatewayInfo(LORA,simTime());
            }else{
                // Already in table, delete old and add new
                GatewayInfo& rxPkt = iter->second;
                rxPkt.lastSeen = simTime();
                // Update Trust based on this?
            }
        }else{
            // This gets added to received packets table
            DevAddr txAddr = frame->getTransmitterAddress();
            auto iter = ReceivedPacketsList->find(txAddr);
            if(iter == ReceivedPacketsList->end()){
                // Not found in table, insert
                (*ReceivedPacketsList)[txAddr] = ReceivedPacket(LORA,frame->getSequenceNumber(),frame,simTime());
            }else{
                // Already in table, delete old and add new
                ReceivedPacket& rxPkt = iter->second;
                rxPkt.insertionTime = simTime();
                delete rxPkt.frame;
                rxPkt.frame = frame;
                rxPkt.lastSeqNo = frame->getSequenceNumber();
            }
        }
    }else{
        // LAKSH: This is possibly a downlik message as receiver address is set
        // We are ammending source address right now as part of header
        // Its an assumption that can be challenged or changed
        // This is overhearing of downlink messages
        // It is one of the methods for neighbour discovery, but we are 
        // ignoring it for now
        delete pkt;
    }
}

} //namespace inet
