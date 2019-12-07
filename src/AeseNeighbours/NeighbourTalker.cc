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
        AeseGWMode = (AeseGWModes)(int)par("AeseGWMode");
        periodicPingInterval = par("periodicPingInterval");
        transmitPingMessage = new cMessage("Time To Transmit Ping Message");
        currentProtocol = LORA;
        numberOfGateways = par("numberOfGateways");
    }else if (stage == INITSTAGE_APPLICATION_LAYER){
        ReceivedPacketsList = new ReceivedPacketsMap();
        DownlinkLastDropRequestList = new DownlinkLastDropRequest();
        NeighbourDropRequestList = new DownlinkLastDropRequest();

        checkAnyUnsentMessages = new cMessage("Check if any unsent messages in queue");
        loRaGwMac = check_and_cast<LoRaGWMac *>((getParentModule()->getSubmodule("LoRaGWNic"))->getSubmodule("mac"));
        failedBids = 0;
        requestedBids = 0;
        acceptedBids = 0;

        if(AeseGWMode != NO_NEIGHBOUR){
            getSimulation()->getSystemModule()->subscribe("GW_TRANSMITTED_PACKET", this);
            scheduleAt(simTime() + 0.1, checkAnyUnsentMessages);
            startUDP();
            // scheduleAt(simTime() + periodicPingInterval, transmitPingMessage);
        }

    }
}

void NeighbourTalker::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), 3333);

    std::stringstream ss1;
    ss1 << getParentModule();
    std::string str = ss1.str();
    unsigned int first = str.find('[');
    unsigned int last = str.find(']');
    std::string strNew = str.substr(first+1,last-first-1);
    unsigned int idx = stoi(strNew);

    std::string gwAddrsString;
    std::stringstream ss;
    for(unsigned int i=0;i<numberOfGateways;i++){
        if(i!=idx)
            ss << "loRaGWs[" << i << "] ";
    }

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
    if (AeseGWMode == NO_NEIGHBOUR){
        delete msg;
        return;
    }

    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        handleLowerLayer(PK(msg));
    }else if(msg->arrivedOn("udpIn")){
        EV << "Received Request from neighbour" << endl;
        NeighbourTalkerMessage *request = check_and_cast<NeighbourTalkerMessage*>(PK(msg));
        UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(msg->getControlInfo());
        if(request->getMsgType() == REQUEST_HANDOFF)
            canIHandleThisMessage(PK(msg));
        else if(request->getMsgType() == CONFIRM_HANDOFF){
            collectBids(cInfo->getSrcAddr(),request->getDeviceAddress(),request->getRSSI());
            delete msg;
        }else if(request->getMsgType() == ACCEPT_HANDOFF){
            auto iter = NeighbourDropRequestList->find(request->getDeviceAddress());
            if(iter != NeighbourDropRequestList->end()) NeighbourDropRequestList->erase(iter);
            delete msg;
        }
        return;
    }

    if(msg->isSelfMessage()){
        if(msg == transmitPingMessage){
            EV << "Time to transmit a ping message" << endl;
            transmitPing();
            scheduleAt(simTime() + periodicPingInterval, transmitPingMessage);
        }else if(msg == checkAnyUnsentMessages){
            EV << "Time to transmit a ping message" << endl;
            handleDownlinkQueue();
            scheduleAt(simTime() + 0.1, checkAnyUnsentMessages);
        }else if(!strcmp(msg->getName(),"DecideWhichNodeBidToAccept")){
            acceptBid(check_and_cast<DecideWhichNode*>(msg)->getAddr());
            delete msg;
        }else if(!strcmp(msg->getName(),"HaveIReceivedBidConfirmation")){
            checkConfirmationAndDelete(check_and_cast<DecideWhichNode*>(msg)->getAddr());
            delete msg;
        }
    }
}

void NeighbourTalker::finish()
{
    recordScalar("RequestedBids",requestedBids);
    recordScalar("AcceptedBids",acceptedBids);
    recordScalar("FailedBids",failedBids);
    for(auto& elem: downlinkList){
        delete elem.frame;
    }
}

void NeighbourTalker::canIHandleThisMessage(cPacket* pkt)
{
    bool deletePacket = true;
    NeighbourTalkerMessage *frame = check_and_cast<NeighbourTalkerMessage*>(pkt);
    simtime_t freeAfter = loRaGwMac->getTimeForWhenNextMessageIsPossible();
    if (freeAfter == 0 || freeAfter < frame->getSendingTime()){
        // This can be scheduled
        auto iter = ReceivedPacketsList->find(frame->getDeviceAddress());
        // Do I know this node?
        if(iter != ReceivedPacketsList->end()){
            auto insertionTime = (iter->second).insertionTime;
            // Have I seen the node in last 2 seconds?
            if(simTime() - insertionTime < 2){
                if(AeseGWMode == NEIGHBOUR_WITH_BIDS || AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM) {
                    sendConfirmationToNeighbour(pkt);
                }
                send(pkt->decapsulate(),"lowerLayerOut");
                deletePacket = false;
            }
        }
    }
    if(deletePacket) {delete pkt->decapsulate();}
    delete pkt;
    // {
        // std::cout << "Cannot handle" << frame->getReceiverAddress() << ":" << frame->getSequenceNumber() << std::endl;
        // delete pkt;
    // }
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
            delete (*it).frame;
            downlinkList.erase(it++);
         }else{
            if(simTime() > (*it).deadByTime){
                std::cout << "Past Dead By time, bye bye!" << std::endl;
                delete (*it).frame;
                downlinkList.erase(it++);
            }else{
                if(simTime() - (*it).addedToQueue > 0.1){
                    EV << "Time to forward and delete message" << endl;
                    auto frame = (*it).frame;
                    requestedBids++;
                    // std::cout << getParentModule() << " requests handoff" << std::endl;
                    for(auto gw: gwAddresses){
                        NeighbourTalkerMessage *request = new NeighbourTalkerMessage("Request_Handoff");
                        request->encapsulate(frame->dup());
                        request->setMsgType(REQUEST_HANDOFF);
                        request->setSendingTime((*it).deadByTime);
                        request->setDeviceAddress((*it).addr);
                        socket.sendTo(request,gw,3333);
                    }
                    if(AeseGWMode == NEIGHBOUR_WITH_BIDS || AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM){
                        DecideWhichNode *msg = new DecideWhichNode("DecideWhichNodeBidToAccept");
                        msg->setAddr((*it).addr);
                        scheduleAt(simTime() + 0.5, msg);
                    }
                    delete (*it).frame;
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
        physicallayer::ReceptionIndication *cInfo = check_and_cast<physicallayer::ReceptionIndication *>(pkt->getControlInfo());
        W w_rssi = cInfo->getMinRSSI();
        double rssi = w_rssi.get()*1000;
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
                (*ReceivedPacketsList)[txAddr] = ReceivedPacket(LORA,frame->getSequenceNumber(),frame,simTime(),rssi);
            }else{
                // Already in table, delete old and add new
                ReceivedPacket& rxPkt = iter->second;
                rxPkt.insertionTime = simTime();
                delete rxPkt.frame;
                rxPkt.frame = frame;
                rxPkt.lastSeqNo = frame->getSequenceNumber();
                rxPkt.RSSI = rssi;
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
            downlinkList.emplace_back(simTime(),frame->getSendingTime(),frame->getSequenceNumber(),frame->getReceiverAddress(),frame);
        }else{
           delete pkt;
        }
        // delete pkt;
    }
    frame->removeControlInfo();
}

void NeighbourTalker::receiveSignal(cComponent *source, simsignal_t signalID, const char* s, cObject* details )
{
    if(AeseGWMode == NO_NEIGHBOUR) return;
    if(!strcmp(getSignalName(signalID),"GW_TRANSMITTED_PACKET")){
        std::string receivedString = s;
        std::string addrstring = receivedString.substr(0,11);
        DevAddr addr = DevAddr(addrstring.c_str());
        (*DownlinkLastDropRequestList)[addr] = std::stoi(receivedString.substr(12));
    }
}

void NeighbourTalker::sendConfirmationToNeighbour(cPacket* pkt)
{
    UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(pkt->getControlInfo());
    NeighbourTalkerMessage *request = check_and_cast<NeighbourTalkerMessage*>(pkt);
    auto iter = ReceivedPacketsList->find(request->getDeviceAddress());

    NeighbourTalkerMessage *confirm = new NeighbourTalkerMessage("CONFIRM_HANDOFF");
    confirm->setMsgType(CONFIRM_HANDOFF);
    confirm->setDeviceAddress(request->getDeviceAddress());
    confirm->setRSSI((iter->second).RSSI);
    socket.sendTo(confirm,cInfo->getSrcAddr(),3333);

    (*NeighbourDropRequestList)[request->getDeviceAddress()] = 1;
    DecideWhichNode *msg = new DecideWhichNode("HaveIReceivedBidConfirmation");
    msg->setAddr(request->getDeviceAddress());
    scheduleAt(simTime()+0.5,msg);
}

void NeighbourTalker::collectBids(L3Address gateway, DevAddr addr, double RSSI)
{
    // std::cout << getParentModule() << " received bid from " << gateway << " for" << addr << std::endl;
    bool bidsForGWExists = false;
    for(uint i=0;i<currentBids.size();i++){
        if(currentBids[i].addr == addr){
            currentBids[i].gatewaysThatBid.emplace_back(gateway,RSSI);
            bidsForGWExists = true;
        }
    }
    if(!bidsForGWExists) {
        BidQueue bdq;
        bdq.addr = addr;
        bdq.gatewaysThatBid.emplace_back(gateway,RSSI);
        currentBids.push_back(bdq);
    }
}

void NeighbourTalker::acceptBid(DevAddr addr)
{
    int idx = -1;
    double RSSIinGW = -99999999999;
    L3Address pickedGateway;
    if(AeseGWMode == NEIGHBOUR_WITH_BIDS){
        for(int i=0;i<currentBids.size();i++){
            if(currentBids[i].addr == addr){
                for(auto gw: currentBids[i].gatewaysThatBid){
                    if(RSSIinGW < std::get<1>(gw)) pickedGateway = std::get<0>(gw);
                }
                idx = i;
                break;
            }
        }
    }else if(AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM){
        for(int i=0;i<currentBids.size();i++){
            if(currentBids[i].addr == addr){
                if(currentBids[i].gatewaysThatBid.size() > 0){
                    int val = intuniform(0,currentBids[i].gatewaysThatBid.size()-1);
                    int j=0;
                    for(auto gw: currentBids[i].gatewaysThatBid){
                        if(j == val)
                            pickedGateway = std::get<0>(gw);
                        j++;
                    }
                }
            }
        }
    }
    if(idx != -1){
        // std::cout << getParentModule() << " Accepts bid for " << addr << " from " << pickedGateway << std::endl;
        NeighbourTalkerMessage *accept = new NeighbourTalkerMessage("ACCEPT_HANDOFF");
        accept->setMsgType(ACCEPT_HANDOFF);
        accept->setDeviceAddress(addr);
        socket.sendTo(accept,pickedGateway,3333);
        currentBids.erase(currentBids.begin() + idx);
        acceptedBids++;
    }else{
        failedBids++;
        // std::cout << "No Bids" << std::endl;
    }
}

void NeighbourTalker::checkConfirmationAndDelete(DevAddr addr)
{
    auto iter = NeighbourDropRequestList->find(addr);
    if(iter != NeighbourDropRequestList->end()){
        // std::cout << getParentModule() << " Deleting " << addr << std::endl;;
        loRaGwMac->popDevAddr(addr);
        NeighbourDropRequestList->erase(iter);
    }

}

} //namespace inet
