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
//    for(uint i=0;i<knownNodes.size();i++)
//    {
//        delete knownNodes[i].historyAllSNIR;
//        delete knownNodes[i].historyAllRSSI;
//        delete knownNodes[i].receivedSeqNumber;
//    }
}

void PacketForwarder::initialize(int stage)
{
    schedulerClass = getSimulation()->getScheduler()->str();

    if (stage == 0) {
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
        if(enableDQ){
            CRQVector.setName("CRQVector");
            DTQVector.setName("DTQVector");
            dataReceived.setName("ReceivedQueue");
            MiniSlots.setName("EmptyMSlots");
            sendFeedbackMessage(true);
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
        startUDP();
        getSimulation()->getSystemModule()->subscribe("LoRa_AppPacketSent", this);
    }
}


void PacketForwarder::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);

    const char *destAddrs = par("destAddresses");
    std::string destAddrsString;
    if ((destAddrs != NULL) && (destAddrs[0] == '\0')) {
         std::stringstream ss;
         ss << "networkServer[" << gwNSNumber << "]";
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
    }
}


void PacketForwarder::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received LoRaMAC frame" << endl;
        // std::cout << "Received LoRaMAC frame in GW" << std::endl;
        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(PK(msg));
        // std::cout << frame << std::endl;
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
        // LAKSH: No duty cycling check done here
        EV << "Received UDP packet" << endl;
        // if(simTime()-timeOfLastPacket > 0.5){
            sentMsgs++;
            // timeOfLastPacket = simTime();
            LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(PK(msg));
            send(frame, "lowerLayerOut");
        // }else{
            // delete msg;
        // }
    }else if(msg->isSelfMessage()){
        std::string s(msg->getName());
        if(s.compare("rtEvent") == 0){
            processPacketMatlab();
            delete msg;
        }else if(msg == sendFeedback){
            sendFeedbackMessage(false);
        }else if(msg == sendActuation){
            sendActuationMessage();
        }else if(msg == sendActuationNoSimulink){
            if(currentCntActuators != 0){
                AeseAppPacket *request = new AeseAppPacket("ActuationFrame");
                request->setMsgType(ACTUATION);
                for(int la=0;la<currentCntActuators;la++){
                    request->setActuatorNumber(la,actuatorNumbers[la]);
                    request->setPacketGeneratedTime(la,packetGeneratedTime[la]);
                    request->setActuatorSequenceNumbers(la,actuatorSequenceNumbers[la]);
                }
                for(int la=currentCntActuators;la<128;la++){
                    request->setActuatorNumber(la,-1);
                    request->setPacketGeneratedTime(la,-1);
                    request->setActuatorSequenceNumbers(la,-1);
                }
                sendActuationMessageNow(request);
            }
            actuationPeriod = std::min(0.5,0.0152*(13+(currentCntActuators*2))+0.1748);
            scheduleAt(simTime() + actuationPeriod,sendActuationNoSimulink);
            currentCntActuators = 0;
        }else{
            for(int i = 0; i < 8 ; i++)
                if(msg == sendDownlink[i])
                    sendPacket(i);
        }
    }
}

void PacketForwarder::processPacketMatlab()
{
    while(numRecvBytes > 0){

        // double trigger[numberOfSubSystems];
        double values[numberOfAeseActuatorNodes]; 

        int totalPacketSize = (numberOfSubSystems * 8) + (numberOfAeseActuatorNodes * 8);
        if(numRecvBytes >= totalPacketSize){
            for(int i = 0; i < numberOfSubSystems; i++){
                union{
                    double d;
                    unsigned char c[8];
                }z;
                memcpy(z.c,&recvBuffer[numRecvBytes-totalPacketSize+i*8],8); // memcpy(z.c,&recvBuffer[numRecvBytes-104+i*8],8);
                // trigger[i] = z.d;
                //// std::cout << trigger[i] << " ";
            }
            //// std::cout << std::endl;
              
            for(int i = 0; i < numberOfAeseActuatorNodes; i++){
                union{
                    double d;
                    unsigned char c[8];
                }z;
                memcpy(z.c,&recvBuffer[numRecvBytes-(totalPacketSize - (numberOfSubSystems * 8))+(i*8)],8);
                values[i] = z.d;
                //// std::cout << values[i] << " ";
            }
            //// std::cout << std::endl;

            if(actuationQueue.front() != nullptr){
                delete actuationQueue.pop();
            }
            AeseAppPacket *request = new AeseAppPacket("ActuationFrame");
            request->setMsgType(ACTUATION);
            for(int la=0;la<numberOfAeseActuatorNodes;la++){
                request->setActuationSignal(la,values[la]);
            }
            actuationQueue.insert(request);
            
            numRecvBytes -= totalPacketSize;
        }else{
            // std::cout << "Setting to zero" << std::endl;
            numRecvBytes = 0;
        }
		// std::cout << "Bytes:" << numRecvBytes << std::endl;
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

    int packettype = frame->getMsgType();
    //for (std::vector<nodeEntry>::iterator it = knownNodes.begin() ; it != knownNodes.end(); ++it)
    // std::cout << "Frame recv from " << frame->getTransmitterAddress() << std::endl;
    if(packettype == UPLINKMINISLOT){
        // std::cout << "Frame with minislot " << frame->getMyMiniSlot() << std::endl;
        miniSlotInformation[frame->getMyMiniSlot()] = miniSlotInformation[frame->getMyMiniSlot()] + 1;
        if(!dataOnSameChannel){
            requestedSlots[frame->getMyMiniSlot()] = frame->getNumberOfFrames();
        }
        /* Is information really needed about data in Minislots? */
        // if(frame->getDataInThisFrame()){
        //     miniSlotsWithDataExpected[frame->getMyMiniSlot()] = miniSlotsWithDataExpected[frame->getMyMiniSlot()] + 1;
        //     dataExpectedfromMiniSlot = true;
        // } 
    }
    else if(packettype == UPLINKDATASLOT){
        // std::cout << "Received frame with data " << std::endl;
        AeseAppPacket *packet = check_and_cast<AeseAppPacket *>((frame)->decapsulate());
        if(dataOnSameChannel){
            if(frame->getDataInThisFrame()) {
                dataExpectedfromMiniSlot = true;
                miniSlotsWithDataExpected[frame->getMyDataSlot()] = frame->getMyMiniSlot();
            }
            else DTQ = DTQ - 1;
        }else{
            int channelNumber = ((frame->getLoRaCF() - inet::units::values::Hz(867100000))/inet::units::values::Hz(200000)).get();
            // std::cout << "Frame with data " << frame->getLoRaCF() << " " << channelNumber << std::endl;
            dataQueues[channelNumber] = dataQueues[channelNumber] - frame->getNumberOfFrames();
            dataReceived.record(simTime());
        }
        dataSlotInformation[frame->getMyDataSlot()] = dataSlotInformation[frame->getMyDataSlot()] + 1;
        actuatorNumbers[currentCntActuators] = packet->getSensorNumber();
        packetGeneratedTime[currentCntActuators] = packet->getPacketGeneratedTime(0);
        actuatorSequenceNumbers[currentCntActuators] = packet->getActuatorSequenceNumbers(0);
        currentCntActuators = currentCntActuators + 1;

        if(schedulerClass == "cSimulinkRTScheduler"){
            std::string str = frame->getTransmitterAddress().str();
            std::vector<int> array;
            std::stringstream ss(str);
            std::string line;
            while (std::getline(ss,line,'-'))
                array.push_back(std::strtol(line.c_str(), 0, 16));
            // std::cout << array[3] << " " << packet->getSampleMeasurement() << std::endl;
            double d = -array[3] + (packet->getSampleMeasurement()/100) - 0.1;
            // std::cout << array[3] << " " << packet->getSampleMeasurement() << " " << d << std::endl;
            if(d<-1)
                rtScheduler->sendValue(d,this);
        }
        delete packet;
    }else if(packettype == JOIN_REQUEST){
        L3Address destAddr = destAddresses[0];
        EV << "Sending to server" << destAddr << endl;
        if (frame->getControlInfo())
            delete frame->removeControlInfo();
        
        if(enableActuation){
            AeseAppPacket *packet = check_and_cast<AeseAppPacket *>((frame)->decapsulate());
            if(sendImmediateActuation){
                AeseAppPacket *request = new AeseAppPacket("ActuationFrame");
                request->setMsgType(ACTUATION);
                request->setActuatorNumber(0,packet->getSensorNumber());
                request->setPacketGeneratedTime(0,packet->getPacketGeneratedTime(0));
                request->setActuatorSequenceNumbers(0,packet->getActuatorSequenceNumbers(0));
                for(int la=1;la<128;la++){
                    request->setActuatorNumber(la,-1);
                    request->setPacketGeneratedTime(la,-1);
                    request->setActuatorSequenceNumbers(la,-1);
                }
                sendActuationMessageNow(request);
            }else{
                // Add to queue
                actuatorNumbers[currentCntActuators] = packet->getSensorNumber();
                packetGeneratedTime[currentCntActuators] = packet->getPacketGeneratedTime(0);
                actuatorSequenceNumbers[currentCntActuators] = packet->getActuatorSequenceNumbers(0);
                currentCntActuators = currentCntActuators + 1;
            }

            if(schedulerClass == "cSimulinkRTScheduler"){
                std::string str = frame->getTransmitterAddress().str();

                std::vector<int> array;
                std::stringstream ss(str);
                std::string line;
                while (std::getline(ss,line,'-'))
                    array.push_back(std::strtol(line.c_str(), 0, 16));
                
                double d = -array[3] + numberOfAeseActuatorNodes + (packet->getSampleMeasurement()/100) - 0.1; // 10
                //// std::cout << d << " " << array[3] << " " << packet->getSampleMeasurement() << std::endl;
                if(d<-1){
                    rtScheduler->sendValue(d,this);
                }
            }
        }

        // delete packet;
        // if(frame->getConfirmedMessage())
        socket.sendTo(frame, destAddr, destPort);
        // else
        //     delete pk;
    }

    // FIXME : Identify network server message is destined for.
    // L3Address destAddr = destAddresses[0];
    // if (frame->getControlInfo())
    //    delete frame->removeControlInfo();

    // socket.sendTo(frame, destAddr, destPort);
    // if(packettype != JOIN_REQUEST)
    //     delete pk;
}

void PacketForwarder::scheduleDownlink(int val,cPacket *pk)
{   
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(pk);
    AeseAppPacket *request = check_and_cast<AeseAppPacket *>(frame->decapsulate());
    
    AeseAppPacket *downlink = new AeseAppPacket("DownlinkFrame");
    if(schedulerClass == "cSimulinkRTScheduler"){
        downlink->setKind(DATADOWN);
    }else{
        downlink->setKind(DATANOSOCKET);
    }
    downlink->setSampleMeasurement(request->getSampleMeasurement());

    LoRaMacFrame *response = new LoRaMacFrame("DownlinkFrame");
    response->encapsulate(downlink);
    response->setLoRaTP(frame->getLoRaTP());
    response->setLoRaCF(frame->getLoRaCF());
    response->setLoRaSF(frame->getLoRaSF());
    response->setLoRaBW(frame->getLoRaBW());
    response->setLoRaCR(frame->getLoRaCR());
    response->setReceiverAddress(frame->getTransmitterAddress());

    frameCopy[val] = response;

    scheduleAt(simTime() + 1, sendDownlink[val]);
    cnt |= 1 << val;
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

void PacketForwarder::sendActuationMessage(){
    
    units::values::Hz loraCF=inet::units::values::Hz(869587500);
    units::values::Hz loraBW=inet::units::values::Hz(125000);

    // AeseAppPacket *request = new AeseAppPacket("ActuationFrame");
    // request->setMsgType(ACTUATION);
    if(actuationQueue.front() != nullptr){
        LoRaMacFrame *frameToSend = new LoRaMacFrame("ActuationPacket");
        frameToSend->encapsulate(actuationQueue.pop());
        frameToSend->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);

        frameToSend->setLoRaTP(14);
        frameToSend->setLoRaCF(loraCF);
        frameToSend->setLoRaSF(7);
        frameToSend->setLoRaBW(loraBW);
        frameToSend->setLoRaCR(1);

        // std::cout << "Sending Actuation message " << std::endl;
        frameToSend->setMsgType(ACTUATION);
        
        frameCopy[1] = frameToSend;
        sendPacket(1);
    }
    scheduleAt(simTime() + actuationPeriod,sendActuation);
}

void PacketForwarder::sendActuationMessageNow(AeseAppPacket* appPacket)
{
    units::values::Hz loraCF=inet::units::values::Hz(869587500);
    units::values::Hz loraBW=inet::units::values::Hz(125000);

    LoRaMacFrame *frameToSend = new LoRaMacFrame("ActuationPacket");
    frameToSend->encapsulate(appPacket);
    frameToSend->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);

    frameToSend->setLoRaTP(14);
    frameToSend->setLoRaCF(loraCF);
    frameToSend->setLoRaSF(7);
    frameToSend->setLoRaBW(loraBW);
    frameToSend->setLoRaCR(1);

    // std::cout << "Sending Actuation message " << std::endl;
    frameToSend->setMsgType(ACTUATION);
    
    frameCopy[1] = frameToSend;
    sendPacket(1);
}

void PacketForwarder::sendFeedbackMessage(bool first){
    
    units::values::Hz loraCF;
    if(dataOnSameChannel){
        loraCF=inet::units::values::Hz(868000000);
    }else{
        loraCF=inet::units::values::Hz(869460500);
    }
    units::values::Hz loraBW=inet::units::values::Hz(125000);

    if(first){
        AeseAppPacket *request = new AeseAppPacket("StartFrame");
        request->setMsgType(FEEDBACK);

        LoRaMacFrame *frameToSend = new LoRaMacFrame("StartPacket");
        frameToSend->encapsulate(request);
        frameToSend->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);

        frameToSend->setLoRaTP(14);
        frameToSend->setLoRaCF(loraCF);
        frameToSend->setLoRaSF(7);
        frameToSend->setLoRaBW(loraBW);
        frameToSend->setLoRaCR(4);
        frameToSend->setPayloadLength(20);

        frameToSend->setMsgType(FEEDBACK);
        frameToSend->setDTQ(DTQ);
        frameToSend->setCRQ(CRQ);
        
        frameCopy[0] = frameToSend;
        scheduleAt(simTime() + timeToStartSignal, sendDownlink[0]);
        scheduleAt(simTime() + timeToStartSignal + 0.08 + frameDuration,sendFeedback);
    }else{
        AeseAppPacket *request = new AeseAppPacket("FeedbackFrame");
        request->setMsgType(FEEDBACK);

        LoRaMacFrame *frameToSend = new LoRaMacFrame("FeedbackPacket");
        frameToSend->encapsulate(request);
        frameToSend->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);

        frameToSend->setLoRaTP(14);
        frameToSend->setLoRaCF(loraCF);
        frameToSend->setLoRaSF(7);
        frameToSend->setLoRaBW(loraBW);
        frameToSend->setLoRaCR(4);
        frameToSend->setPayloadLength(20);

        if(CRQ > 0)
            CRQ = CRQ - 1;
       
        int totalMiniSlots = 0;

        if(dataOnSameChannel){
            for(int i = 0; i < noOfNslots; i++){
                if(dataSlotInformation[i] == 0){
                    frameToSend->setN(i,EMPTY);
            totalMiniSlots = totalMiniSlots + 1;
                }else if(dataSlotInformation[i] == 1){
                    frameToSend->setN(i,SUCCESS);
                    if(dataExpectedfromMiniSlot && (miniSlotsWithDataExpected[i] > -1)) {
                        miniSlotInformation[miniSlotsWithDataExpected[i]] = miniSlotInformation[miniSlotsWithDataExpected[i]] - 1;   
                    }
                }else if(dataSlotInformation[i] > 1){
                    frameToSend->setN(i,COLLISION);
                }
                dataSlotInformation[i] = 0;
                miniSlotsWithDataExpected[i] = -1;
            }
            dataExpectedfromMiniSlot = false;
            
            for(int i = 0;i < noOfMslots; i++){
                // std::cout << slotInformation[i];
                if(miniSlotInformation[i] == 0){
                    frameToSend->setM(i,EMPTY);
                }else if(miniSlotInformation[i] == 1){
                    frameToSend->setM(i,SUCCESS);
                    DTQ = DTQ + 1;
                }else if(miniSlotInformation[i] > 1){
                    // This may not be happening because droppedpacket is not sent upthe stack yet
                    frameToSend->setM(i,COLLISION);
                    CRQ = CRQ + 1;
                }
                miniSlotInformation[i] = 0;
            }
            frameToSend->setDTQ(DTQ);
            frameToSend->setCRQ(CRQ);
            // std::cout << "Sending frame with " << DTQ << CRQ << std::endl;
        }else{
            for(int i = 0;i < noOfMslots; i++){
                // std::cout << slotInformation[i];
                if(miniSlotInformation[i] == 0){
                    frameToSend->setM(i,EMPTY);
                    totalMiniSlots = totalMiniSlots + 1;
                }else if(miniSlotInformation[i] == 1){
                    frameToSend->setM(i,SUCCESS);
                    int lowestValue = dataQueues[0];
                    int lowestIndex = 0;
                    for(int j = 1; j < numberOfChannels; j++){
                        if(dataQueues[j] < lowestValue){
                            lowestValue = dataQueues[j];
                            lowestIndex = j;
                        }
                    }
                    frameToSend->setPosition(i,lowestValue);
                    frameToSend->setChannelNumber(i,lowestIndex);
                    dataQueues[lowestIndex] = dataQueues[lowestIndex] + requestedSlots[i];
                    // std::cout << "Node to go to:" << lowestIndex << " " << lowestValue << std::endl;
                }else if(miniSlotInformation[i] > 1){
                    // This may not be happening because droppedpacket is not sent upthe stack yet
                    frameToSend->setM(i,COLLISION);
                    CRQ = CRQ + 1;
                }
                miniSlotInformation[i] = 0;
            }
            frameToSend->setCRQ(CRQ);
            // std::cout << "Sending frame with " << dataQueues[0] << " " << dataQueues[1] << " " << dataQueues[2] << std::endl;
        }
        frameToSend->setMsgType(FEEDBACK);
        CRQVector.record(CRQ); 
        DTQVector.record(dataQueues[0] + dataQueues[1] + dataQueues[2] + dataQueues[3] + dataQueues[4] + dataQueues[5] + dataQueues[6] + dataQueues[7]);
        MiniSlots.record(totalMiniSlots);
        frameCopy[0] = frameToSend;
        sendPacket(0);
        scheduleAt(simTime() + 0.08 + frameDuration,sendFeedback);
        //replace 0.08 with timeonair
    }
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

} //namespace inet
