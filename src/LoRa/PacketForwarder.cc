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

#include "PacketForwarder.h"
#include "inet/networklayer/ipv4/IPv4Datagram.h"
#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "../misc/cSimulinkRTScheduler.h"

namespace inet {

Define_Module(PacketForwarder);

PacketForwarder::~PacketForwarder()
{
    cancelAndDelete(rtEvent);
    cancelAndDelete(sendFeedback);
    cancelAndDelete(sendActuation);
    cancelAndDelete(sendActuationNoSimulink);
    for(int i = 0; i < 8;i++){
        cancelAndDelete(sendDownlink[i]);
    }
   // for(uint i=0;i<knownNodes.size();i++)
   // {
   //     delete knownNodes[i].historyAllSNIR;
   //     delete knownNodes[i].historyAllRSSI;
   //     delete knownNodes[i].receivedSeqNumber;
   // }
}

void PacketForwarder::initialize(int stage)
{
    schedulerClass = getSimulation()->getScheduler()->str();

    if (stage == 0) {
        GWinGrid();

        LoRa_GWPacketReceived = registerSignal("LoRa_GWPacketReceived");
        getSimulation()->getSystemModule()->subscribe("LoRaMiniSlotCollision", this);
        getSimulation()->getSystemModule()->subscribe("LoRaDataSlotCollision", this);
        getSimulation()->getSystemModule()->subscribe("GW_USED_TIME", this);
        localPort = par("localPort");
        destPort = par("destPort");

        rtEvent = new cMessage("rtEvent");
        sendFeedback = new cMessage("SendFeedback");
        sendActuation = new cMessage("SendActuation");
        sendActuationNoSimulink = new cMessage("sendActuationNoSimulink");
        enableDQ = par("enableDQ");
        enableActuation = par("enableActuation");

        actuationPeriod = par("actuationPeriod");
        timeToStartSignal = par("timeToStartSignal");
        sendImmediateActuation = par("sendImmediateActuation");
        currentCntActuators = 0;

        numberOfNodes = par("numberOfNodes");
        numberOfAeseNodes = par("numberOfAeseNodes");
        numberOfAeseActuatorNodes = par("numberOfAeseActuatorNodes");
        numberOfSubSystems = par("numberOfSubSystems");
        int numberOfNS = par("numberOfNS");
        gwNSNumber = DevAddr::generateGwNSNumber(numberOfNS);
        numberOfGateways = par("numberOfGateways");

        if(enableDQ){
            noOfMslots = par("noOfMslots");
            noOfNslots = par("noOfNslots");
            mSlotDuration = par("mSlotDuration"); //s
            nSlotDuration = par("nSlotDuration"); //s
            numberOfChannels = par("numberOfChannels");
            dataOnSameChannel = par("dataOnSameChannel");

            // std::cout << noOfNslots << " " << nSlotDuration << " " << noOfMslots << " " << mSlotDuration << std::endl;
            if(dataOnSameChannel){
                frameDuration = (noOfNslots*nSlotDuration) + (noOfMslots*mSlotDuration);
            }else{
                frameDuration = (noOfMslots*mSlotDuration);
            }
        }
        if(schedulerClass == "cSimulinkRTScheduler"){
            rtScheduler = check_and_cast<cSimulinkRTScheduler *>(getSimulation()->getScheduler());
            rtScheduler->setInterfaceModule(this, rtEvent, recvBuffer, 4000, &numRecvBytes,true,false);
        }

        for(int i = 0; i < 8; i++)
            sendDownlink[i] = new cMessage("sendDownlink");

        if(enableDQ){
            for(int i = 0; i < noOfMslots; i++)
                miniSlotInformation[i] = 0;

            for(int i = 0; i < noOfNslots; i++){
                miniSlotsWithDataExpected[i] = -1;
                dataSlotInformation[i] = 0;
            }
            dataExpectedfromMiniSlot = false;
            for(int i=0;i<8;i++){
                dataQueues[i] = 0;
            }
        }

    }else if (stage == INITSTAGE_APPLICATION_LAYER){
        listOfSuccessfulMessages.setName("listOfSuccessfulMessages");
        listOfSuccessfulNodes.setName("listOfSuccessfulNodes");
        actualCntValues.setName("listOfCntValues");
	    macCntValues.setName("listOfMacCntValues");
        
	    if(enableDQ){
            CRQVector.setName("CRQVector");
            DTQVector.setName("DTQVector");
            dataReceived.setName("ReceivedQueue");
            MiniSlots.setName("EmptyMSlots");
        }
        if(enableActuation){
            if(schedulerClass == "cSimulinkRTScheduler"){
               actuationQueue.setName("actuationQueue");
               scheduleAt(simTime() + actuationPeriod,sendActuation);
            }else if(!sendImmediateActuation){
                actuationQueue.setName("actuationQueue");
                scheduleAt(simTime() + actuationPeriod,sendActuationNoSimulink);
            }
        }
        NodesBelongToMe = new NodesBelongToMeStruct();
        startUDP();
        getSimulation()->getSystemModule()->subscribe("LoRa_AppPacketSent", this);
    }
}

void PacketForwarder::GWinGrid()
{
    auto *mobility = check_and_cast<StationaryMobility *>(getParentModule()->getSubmodule("mobility"));

    int GWNumber = DevAddr::generateGatewayNumber();
    int numberOfGateways = par("numberOfGateways");

    int val1 = ceil(sqrt(numberOfGateways));
    while(numberOfGateways % val1 != 0) val1 = val1 + 1;
    int val2 = numberOfGateways/val1;

    if(val2 == 1)
        val2 = val2 + 1;
    int xID = 1 + (GWNumber%val1);
    int yID = 1 + (GWNumber/val1);

    double xGap = (par("constraintAreaMaxX").doubleValue()) / (val1+1);
    double yGap = (par("constraintAreaMaxY").doubleValue()) / (val2+1);

    double xpos = xID * xGap;
    double ypos = yID * yGap;

    mobility->par("initialX").setDoubleValue(xpos);
    mobility->par("initialY").setDoubleValue(ypos);
}

void PacketForwarder::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);

    int sendToAll = par("sendToAll");

    std::string destAddrsString;
    if(sendToAll == 0){
        const char *destAddrs = par("destAddresses");
        if ((destAddrs != NULL) && (destAddrs[0] == '\0')) {
            std::stringstream ss;
            ss << "networkServer[" << gwNSNumber << "]";
            destAddrsString = ss.str();
        }
    }else{
        std::stringstream ss;
        int numberOfNS = par("numberOfNS");
        for(int i=0;i<numberOfNS;i++) ss << "networkServer[" << i << "] ";
        destAddrsString = ss.str();
    }

    const char *destAddrsNew = destAddrsString.c_str();
    cStringTokenizer tokenizer(destAddrsNew);
    const char *token;

    // Create UDP sockets to multiple destination addresses (network servers)
    while ((token = tokenizer.nextToken()) != nullptr) {
        L3Address result;
        L3AddressResolver().tryResolve(token, result);
        if (result.isUnspecified())
            EV_ERROR << "cannot resolve destination address: " << token << endl;
        else
            EV << "Got destination address: " << token << " with result: " << result << endl;
        destAddresses.push_back(result);
        std::stringstream ss;
        ss << "networkServer[" << gwNSNumber << "]";
        if(strcmp(ss.str().c_str(),token) == 0){
            myNetworkServer = result;
        }
    }
}


void PacketForwarder::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received LoRaMAC frame" << endl;
        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(PK(msg));
        if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS){
            // updateAndLogNode(frame);
            CountUniqueNodes(frame);
            processLoraMACPacket(PK(msg));
        }else{
            delete msg;
        }
        //send(msg, "upperLayerOut");
        //sendPacket();
    } else if (msg->arrivedOn("udpIn")) {
        EV << "Received UDP packet" << endl;
        sentMsgs++;
        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(PK(msg));
        DevAddr addr = frame->getReceiverAddress();
        // This is supposed to be handled by me
        if(frame->getMsgType() == JOIN_REPLY){
            (*NodesBelongToMe)[addr] = destAddresses[addr.getAddressByte(0)];
        }
        frame->setType(MY_ACKS);
        send(frame, "lowerLayerOut");
    }
}

void PacketForwarder::processLoraMACPacket(cPacket *pk)
{
    emit(LoRa_GWPacketReceived, 42);
    if (simTime() >= getSimulation()->getWarmupPeriod())
        counterOfReceivedPackets++;
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(pk);

    physicallayer::ReceptionIndication *cInfo = check_and_cast<physicallayer::ReceptionIndication *>(pk->getControlInfo());
    W w_rssi = cInfo->getMinRSSI();
    double rssi = w_rssi.get()*1000;
    frame->setRSSI(math::mW2dBm(rssi));
    frame->setSNIR(cInfo->getMinSNIR());
    EV << frame->getTransmitterAddress() << frame->getMsgType() << endl;
    // std::cout << getParentModule() << "," << frame->getLoRaCF() << "," << frame->getLoRaSF() << "," << frame->getLoRaBW() << "," << frame->getLoRaCR() << std::endl;
    // listOfSuccessfulMessages.record(frame->getLoRaCF().get());
    // LJB: UPDATING listOfSuccessfulMessages to output frequency and time on air
    int PayloadLength = frame->getPayloadLength();
    if(PayloadLength == 0)
        PayloadLength = 20;
    double toa = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
    listOfSuccessfulMessages.record(frame->getLoRaCF().get()+toa);

    //(frame->getTransmitterAddress().getAddressByte(2) << 8) + frame->getTransmitterAddress().getAddressByte(3);
    listOfSuccessfulNodes.record((frame->getTransmitterAddress().getAddressByte(2) << 8) + frame->getTransmitterAddress().getAddressByte(3));
    macCntValues.record(frame->getSequenceNumber());
    AeseAppPacket *packet = check_and_cast<AeseAppPacket *>((frame)->getEncapsulatedPacket());
    actualCntValues.record(packet->getActuatorSequenceNumbers(0));

    int packettype = frame->getMsgType();
    //for (std::vector<nodeEntry>::iterator it = knownNodes.begin() ; it != knownNodes.end(); ++it)
    // std::cout << "Frame recv from " << frame->getTransmitterAddress() << std::endl;
    
    if(packettype == JOIN_REQUEST){
        EV << "Sending to network server 0" << endl;
        if (frame->getControlInfo())
            delete frame->removeControlInfo();
        socket.sendTo(frame,destAddresses[0],destPort);
    }else if(packettype == DATA){
        // Send to correct networkserver
        auto iter = NodesBelongToMe->find(frame->getTransmitterAddress());
        if(iter != NodesBelongToMe->end()){
            EV << "Sending to server" << iter->second << endl;
            socket.sendTo(frame, iter->second, destPort);
        }else delete frame;
    }
}

void PacketForwarder::sendPacket(int val)
{
    // std::cout << "Sending message from GW to: " << frameCopy[val]->getReceiverAddress() << " at time " << simTime() << std::endl;

    cnt &= ~(1 << val);
    LoRaMacFrame *response = frameCopy[val]->dup();
    send(response, "lowerLayerOut");
    delete frameCopy[val];
}

void PacketForwarder::receiveSignal(cComponent *source, simsignal_t signalID, long value ,cObject *details)
{
    if(!strcmp(getSignalName(signalID),"LoRaMiniSlotCollision")){
        // std::cout << "Receivd collision at minislot " << value << std::endl;
        miniSlotInformation[value] = miniSlotInformation[value] + 1;
    }else if(!strcmp(getSignalName(signalID),"LoRaDataSlotCollision")){
        dataSlotInformation[value] = dataSlotInformation[value] + 1;
    }else{
        if (simTime() >= getSimulation()->getWarmupPeriod())
            counterOfSentPacketsFromNodes++;
    }
}

void PacketForwarder::receiveSignal(cComponent *src, simsignal_t signalID, double value, cObject *details)
{
    if(!strcmp(getSignalName(signalID),"GW_USED_TIME")){
       totalUsedTimes = totalUsedTimes + value;
    }
}

void PacketForwarder::finish()
{
    recordScalar("LoRa_GW_DER", double(counterOfReceivedPackets)/counterOfSentPacketsFromNodes);
    recordScalar("Sent Messages by Gateway",sentMsgs);
    recordScalar("UniqueNodesCount",knownNodes.size());
    recordScalar("AverageUsedTimePerNode", knownNodes.size()/totalUsedTimes);
    recordScalar("DownlinkTotalUsedTimes", totalUsedTimes);
}

void PacketForwarder::updateAndLogNode(LoRaMacFrame* pkt)
{
    bool nodeExist = false;
    int nodeIndex = 0;
    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == pkt->getTransmitterAddress())
        {
            nodeExist = true;
            nodeIndex = i;
            break;
        }
    }
    if(nodeExist == false)
    {
        nodeLog newNode;
        newNode.srcAddr= pkt->getTransmitterAddress();
        std::string str = pkt->getTransmitterAddress().str();
        // std::vector<int> array;
        // std::stringstream ss(str);
        // std::string line;
        // while (std::getline(ss,line,'-'))
        //     array.push_back(std::strtol(line.c_str(), 0, 16));

        newNode.historyAllSNIR = new cOutVector;
        newNode.historyAllSNIR->setName((std::string("SNIR:") + str).c_str());
        newNode.historyAllSNIR->record(math::fraction2dB(pkt->getSNIR()));
        newNode.historyAllRSSI = new cOutVector;
        newNode.historyAllRSSI->setName((std::string("RSSI:") + str).c_str());
        newNode.historyAllRSSI->record(pkt->getRSSI());
        newNode.receivedSeqNumber = new cOutVector;
        newNode.receivedSeqNumber->setName((std::string("SeqNumber:") + str).c_str());
        knownNodes.push_back(newNode);
    }else{
        knownNodes[nodeIndex].historyAllSNIR->record(math::fraction2dB(pkt->getSNIR()));
        knownNodes[nodeIndex].historyAllRSSI->record(pkt->getRSSI());
        knownNodes[nodeIndex].receivedSeqNumber->record(pkt->getSequenceNumber());
    }
}

void PacketForwarder::CountUniqueNodes(LoRaMacFrame* pkt)
{
    bool nodeExist = false;
    int nodeIndex = 0;
    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == pkt->getTransmitterAddress())
        {
            nodeExist = true;
            nodeIndex = i;
            break;
        }
    }
    if(nodeExist == false)
    {
        nodeLog newNode;
        newNode.srcAddr= pkt->getTransmitterAddress();
        std::string str = pkt->getTransmitterAddress().str();
        // std::vector<int> array;
        // std::stringstream ss(str);
        // std::string line;
        // while (std::getline(ss,line,'-'))
        //     array.push_back(std::strtol(line.c_str(), 0, 16));

        // newNode.historyAllSNIR = new cOutVector;
        // newNode.historyAllSNIR->setName((std::string("SNIR:") + str).c_str());
        // newNode.historyAllSNIR->record(math::fraction2dB(pkt->getSNIR()));
        // newNode.historyAllRSSI = new cOutVector;
        // newNode.historyAllRSSI->setName((std::string("RSSI:") + str).c_str());
        // newNode.historyAllRSSI->record(pkt->getRSSI());
        // newNode.receivedSeqNumber = new cOutVector;
        // newNode.receivedSeqNumber->setName((std::string("SeqNumber:") + str).c_str());
        knownNodes.push_back(newNode);
    }else{
        // knownNodes[nodeIndex].historyAllSNIR->record(math::fraction2dB(pkt->getSNIR()));
        // knownNodes[nodeIndex].historyAllRSSI->record(pkt->getRSSI());
        // knownNodes[nodeIndex].receivedSeqNumber->record(pkt->getSequenceNumber());
    }
}

int PacketForwarder::getGatewayNsNumber()
{
    return gwNSNumber;
}

} //namespace inet
