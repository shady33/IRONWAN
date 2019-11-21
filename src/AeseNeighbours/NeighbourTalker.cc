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
    cancelAndDelete(transmitPingMessage);
    cancelAndDelete(checkAnyUnsentMessages);
}

void NeighbourTalker::initialize(int stage)
{
    if (stage == 0) {
        AeseGWEnabled = par("AeseGWEnabled");
        periodicPingInterval = par("periodicPingInterval");
        transmitPingMessage = new cMessage("Time To Transmit Ping Message");
        currentProtocol = LORA;
        numberOfGateways = par("numberOfGateways");
        getSimulation()->getSystemModule()->subscribe("GW_TRANSMITTED_PACKET", this);
    }else if (stage == INITSTAGE_APPLICATION_LAYER){
        ReceivedPacketsList = new ReceivedPacketsMap();
        DownlinkLastDropRequestList = new DownlinkLastDropRequest();
        checkAnyUnsentMessages = new cMessage("Check if any unsent messages in queue");
        if(AeseGWEnabled){
            scheduleAt(simTime() + 0.1, checkAnyUnsentMessages);
            // scheduleAt(simTime() + periodicPingInterval, transmitPingMessage);
        }
    }
}

void NeighbourTalker::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), 2500);

    std::string gwAddrsString;
    std::stringstream ss;
    for(int i=0;i<numberOfGateways;i++)
         ss << "loRaGWs[" << i << "] ";
    gwAddrsString = ss.str();
    const char *gwAddrsNew = gwAddrsString.c_str();

    cStringTokenizer tokenizerGW(gwAddrsNew);
    const char *tokenGW;

    // Create UDP sockets to multiple destination addresses (network servers)
    while ((tokenGW = tokenizerGW.nextToken()) != nullptr) {
        L3Address result;
        L3AddressResolver().tryResolve(tokenGW, result);
        if (result.isUnspecified())
            EV_ERROR << "cannot resolve destination address: " << tokenGW << endl;
        else
            EV << "Got destination address: " << tokenGW << " with result: " << result << endl;
        gwAddresses.push_back(result);
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
    }else if(msg == transmitPingMessage){
        EV << "Time to transmit a ping message" << endl;
        transmitPing();
        scheduleAt(simTime() + periodicPingInterval, transmitPingMessage);
    }else if(msg == checkAnyUnsentMessages){
        EV << "Time to transmit a ping message" << endl;
        handleDownlinkQueue();
        scheduleAt(simTime() + 0.1, checkAnyUnsentMessages);
    }   
}

void NeighbourTalker::finish()
{
    for(auto& elem: downlinkList){
        delete elem.pkt;
    }
}

void NeighbourTalker::handleDownlinkQueue()
{
    // for(auto& elem : downlinkList){
    for(std::list<DownlinkPacket>::iterator it=downlinkList.begin()++; it != downlinkList.end(); ++it){
        // std::cout << "Added to queue:" << elem.addedToQueue << std::endl;
        // std::cout << "Dead by time:" << elem.deadByTime << std::endl;
        // std::cout << "Dev Addr:" << elem.addr << std::endl;
        // std::cout << "Pkt" << elem.pkt << std::endl;
        int sequenceNumberInList = -1;
        auto iter = DownlinkLastDropRequestList->find((*it).addr);
        if(iter != DownlinkLastDropRequestList->end()){
            sequenceNumberInList = iter->second;
        }
    
        if((*it).sequenceNumber == sequenceNumberInList){
            // The message was added to queue and broadcast by the node
            delete (*it).pkt; 
            downlinkList.erase(it++);
         }else{
            if(simTime() > (*it).deadByTime){
                std::cout << "Past Dead By time, bye bye!" << std::endl;
                delete (*it).pkt; 
                downlinkList.erase(it++); 
            }else{
                if(simTime() - (*it).addedToQueue > 0.1){
                    std::cout << "Time to forward and delete message" << std::endl;;
                    delete (*it).pkt; 
                    downlinkList.erase(it++); 
                }
            }
         }
    }
}

void NeighbourTalker::transmitPing()
{
    if(currentProtocol == LORA) {
        transmitLoRaMessage();
    }else{
        throw cRuntimeError("Unhandled protocol");
    }
}

void NeighbourTalker::transmitLoRaMessage()
{
    // LAKSH: No request packet in this yet
    // AeseAppPacket* app = new AeseAppPacket("PingFrame");
    // LoRaMacFrame* frame = 
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
        // Update 1: This is also now a message from my own packetforwarder
        // which is the Ackpacket and we need to cache it for my packetforwarder
        // Update 2: Overhearing of downlink messages has been disabled in LoRaGWMac

        if(frame->getMsgType() == ACK_ADR_PACKET){
            // std::cout << getParentModule()->str() << frame->getReceiverAddress() << " " << frame->getMsgType() << std::endl;
            downlinkList.emplace_back(simTime(),simTime(),frame->getSequenceNumber(),frame->getReceiverAddress(),pkt);
        }else{
           delete pkt; 
        }
        // delete pkt;
    }
}

void NeighbourTalker::receiveSignal(cComponent *source, simsignal_t signalID, const char* s, cObject* details )
{
    if(!strcmp(getSignalName(signalID),"GW_TRANSMITTED_PACKET")){
        std::string receivedString = s;
        std::string addrstring = receivedString.substr(0,11);
        DevAddr addr = DevAddr(addrstring.c_str());
        // auto iter = DownlinkLastDropRequestList->find(addr);
        // if( iter != DownlinkLastDropRequestList.end()){
        (*DownlinkLastDropRequestList)[addr] = std::stoi(receivedString.substr(12));
        // }else{

        // }
        // DownlinkLastDropRequestList
        // for(std::list<DownlinkPacket>::iterator it=downlinkList.begin()++; it != downlinkList.end(); ++it){
        //     if(!strcmp((((*it).addr).str()).c_str(),s)){
        //         std::cout << "Deleting from list" << std::endl;
        //         delete (*it).pkt;
        //         downlinkList.erase(it++);
        //     }
        // }
    }
}

} //namespace inet
