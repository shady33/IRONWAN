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

#include <SimpleLoRaApp.h>
//#include "inet/mobility/static/StationaryMobility.h"

namespace inet {

Define_Module(SimpleLoRaApp);

SimpleLoRaApp::~SimpleLoRaApp()
{
    cancelAndDelete(rtEvent);
    cancelAndDelete(retryMeasurements);
    cancelAndDelete(sendMeasurements);    
}

void SimpleLoRaApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    schedulerClass = getSimulation()->getScheduler()->str();

    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           // StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           LinearMobility *mobility = check_and_cast<LinearMobility *>(host->getSubmodule("mobility"));
           mobility->par("speed").setDoubleValue(1);
           mobility->par("angle").setDoubleValue(270);
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
        
        rtEvent = new cMessage("rtEvent");
        retryLimit = par("retryLimit");

        if(schedulerClass == "cSimulinkRTScheduler"){
            rtScheduler = check_and_cast<cSimulinkRTScheduler *>(getSimulation()->getScheduler());
            rtScheduler->setInterfaceModule(this, rtEvent, recvBuffer, 4000, &numRecvBytes,false,false);
        }
        e2edelay.setName("EndToEndDelay");
        retransmits.setName("NumberOfRetransmissions");
        timeOfLastPacket = 0;
        seqeuenceNumber = 0;
        first = true;
        success = false;

        sensorNumber = DevAddr::generateSensorNumber();
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        

        retryMeasurements = new cMessage("retryMeasurements");
        sendMeasurements = new cMessage("sendMeasurements");

        if(schedulerClass != "cSimulinkRTScheduler"){
            timeToFirstPacket = par("timeToFirstPacket");
            if(timeToFirstPacket != 0){
                do {
                    timeToFirstPacket = par("timeToFirstPacket");
                    EV << "Wylosowalem czas :" << timeToFirstPacket << endl;
                } while(timeToFirstPacket <= 5);

                // sendMeasurements = new cMessage("sendMeasurements");
                scheduleAt(simTime()+timeToFirstPacket, sendMeasurements);
            }
        }

        sentPackets = 0;
        receivedADRCommands = 0;
        receivedAckMessages = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaDC = par("initialLoRaDC").doubleValue();
        loRaUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");  
        
        // Calculate transmission time according to duty cycle and coding rate
        transmissionTimeTable[0] = (49.408  + loRaCR*7.168) / loRaDC / 10.0; //SF 7
        transmissionTimeTable[1] = (90.624  + loRaCR*12.288) / loRaDC / 10.0; //SF 8
        transmissionTimeTable[2] = (164.864 + loRaCR*20.48) / loRaDC / 10.0; //SF 9
        transmissionTimeTable[3] = (329.728 + loRaCR*40.96) / loRaDC / 10.0; //SF 10
        transmissionTimeTable[4] = (659.456 + loRaCR*81.92) / loRaDC / 10.0; //SF 11
        transmissionTimeTable[5] = (1187.84 + loRaCR*131.072) / loRaDC / 10.0; //SF 12
    }
}

std::pair<double,double> SimpleLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void SimpleLoRaApp::finish()
{
    cModule *host = getContainingNode(this);
//    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    LinearMobility *mobility = check_and_cast<LinearMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("receivedAckMessages", receivedAckMessages);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        std::string s(msg->getName());
        if(s.compare("rtEvent") == 0)
        {
            if(numRecvBytes >= 8){
                union{
                    double d;
                    unsigned char c[8];
                }z;
                memcpy(z.c,&recvBuffer[numRecvBytes-8],8);

                // time * std::pow(10,4) + data * std::pow(10,-4) + sign(data) * std::pow(10,-1)
                double intpart,fractpart,time,sign,value;
                
                fractpart = std::modf(z.d,&intpart);
                time = intpart/10000;
                std::modf(fractpart * 10, &sign);
                value = fractpart * 10000;
                if(sign == 1){
                    value = 1000 - value;
                }

                double dutyCycling;
                if(loRaSF == 7) dutyCycling = transmissionTimeTable[0];
                if(loRaSF == 8) dutyCycling = transmissionTimeTable[1];
                if(loRaSF == 9) dutyCycling = transmissionTimeTable[2];
                if(loRaSF == 10) dutyCycling = transmissionTimeTable[3];
                if(loRaSF == 11) dutyCycling = transmissionTimeTable[4];
                if(loRaSF == 12) dutyCycling = transmissionTimeTable[5];

                dataToSend = value;
                if(noOfRetransmits == 0)
                {
                    if(((time - timeOfLastPacket) > dutyCycling) || first){
                    first = false;
                    startTime = simTime();
                    sendJoinRequest();
                    timeOfLastPacket = time;
                    // retryMeasurements = new cMessage("retryMeasurements");
                    scheduleAt(simTime() + uniform(5,7),retryMeasurements);
                    noOfRetransmits = 1;
                    }
                }

                numRecvBytes = 0;
            }
        }
        else if(msg == retryMeasurements)
        {
            noOfRetransmits++;
            if(noOfRetransmits < retryLimit){
                sendJoinRequest();
                noOfRetransmits = noOfRetransmits + 1;
                // retryMeasurements = new cMessage("retryMeasurements");
                scheduleAt(simTime() + uniform(5,7),retryMeasurements);
            }else{
                noOfRetransmits = 0;
            }
        }
        else if (msg == sendMeasurements)
        {
            // if(noOfRetransmits == 1)
            //     startTime = simTime();
            
            if(!retryMeasurements->isScheduled())
            {
                startTime = simTime();
                seqeuenceNumber++;
                sendJoinRequest();
                noOfRetransmits = 0;
                // retryMeasurements = new cMessage("retryMeasurements");
                scheduleAt(simTime() + uniform(5,7),retryMeasurements);
            }
            // if (simTime() >= getSimulation()->getWarmupPeriod())
                sentPackets++;
            // noOfRetransmits++;
            // if(noOfRetransmits == retryLimit){
            //     retransmits.record(noOfRetransmits);
            //     noOfRetransmits = 1;
            // }else{
            //     // timeToNextPacket = uniform(5,7);
            //     // sendMeasurements = new cMessage("sendMeasurements");
            //     // scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
            //     retryMeasurements = new cMessage("retryMeasurements");
            //     scheduleAt(simTime() + uniform(5,7),retryMeasurements);
            // }
            // Schedule Next send measurements 
            double time = timeOnAir(loRaSF, loRaBW, 40, 1);
            do {
                timeToNextPacket = par("timeToNextPacket");
            } while(timeToNextPacket <= time);

            scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
        }
        // delete msg;
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        delete msg;
    }
}

void SimpleLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    AeseAppPacket *packet = check_and_cast<AeseAppPacket *>(msg);
    if(retryLimit > 1){
        cancelEvent(retryMeasurements);
        e2edelay.record(simTime()-startTime);
        retransmits.record(noOfRetransmits);
        noOfRetransmits = 0;
    }
    if(packet->getKind() == DATADOWN){
        // cancelEvent(retryMeasurements);
        // noOfRetransmits = 0;
        // e2edelay.record(simTime()-startTime);
        // retransmits.record(noOfRetransmits);
        // std::cout << simTime()-startTime << std::endl;
    }else if(packet->getKind() == DATANOSOCKET){
        EV << "Received Downlink not to be sent on socket" << endl;
        // if(retryLimit != 0){
        // cancelEvent(retryMeasurements);
        // retransmits.record(noOfRetransmits);
        // noOfRetransmits = 0;
        // e2edelay.record(simTime()-startTime);
            // double time;
            // if(loRaSF == 7) time = transmissionTimeTable[0];
            // if(loRaSF == 8) time = transmissionTimeTable[1];
            // if(loRaSF == 9) time = transmissionTimeTable[2];
            // if(loRaSF == 10) time = transmissionTimeTable[3];
            // if(loRaSF == 11) time = transmissionTimeTable[4];
            // if(loRaSF == 12) time = transmissionTimeTable[5];
            // do {
            //     timeToNextPacket = par("timeToNextPacket");
            //     //if(timeToNextPacket < 3) error("Time to next packet must be grater than 3");
            // } while(timeToNextPacket <= time);
            // sendMeasurements = new cMessage("sendMeasurements");
            // scheduleAt(simTime()+timeToNextPacket, sendMeasurements);
        // }
    }else if(packet->getMsgType() == JOIN_REPLY){
        receivedAckMessages++;
    }else{
        if (simTime() >= getSimulation()->getWarmupPeriod())
            receivedADRCommands++;
        if(evaluateADRinNode)
        {
            ADR_ACK_CNT = 0;
            if(packet->getMsgType() == TXCONFIG)
            {
                if(packet->getOptions().getLoRaTP() != -1)
                {
                    loRaTP = packet->getOptions().getLoRaTP();
                }
                if(packet->getOptions().getLoRaSF() != -1)
                {
                    loRaSF = packet->getOptions().getLoRaSF();
                }
            }
        }
    }
}

bool SimpleLoRaApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void SimpleLoRaApp::sendJoinRequest()
{
    AeseAppPacket *request = new AeseAppPacket("DataFrame");
    request->setMsgType(JOIN_REQUEST);
    lastSentMeasurement = rand();
    request->setSampleMeasurement(dataToSend);
    request->setPacketGeneratedTime(0,startTime);
    request->setSensorNumber(sensorNumber);
    request->setActuatorSequenceNumbers(0,seqeuenceNumber);

    if(evaluateADRinNode && sendNextPacketWithADRACKReq)
    {
        request->getOptions().setADRACKReq(true);
        sendNextPacketWithADRACKReq = false;
    }

    inet::units::values::Hz freq;
    do{
        freq = inet::units::values::Hz((intuniform(0,2) * 200000) + 868100000);
    }while(loRaCF == freq);
    
    loRaCF = freq;
    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    cInfo->setNumberOfFrames(1);
    cInfo->setPayloadLength(40);
    if(retryLimit == 1){
        cInfo->setConfirmedMessage(false);
    }else{
        cInfo->setConfirmedMessage(true);
    }
    
    request->setControlInfo(cInfo);
    sfVector.record(loRaSF);
    tpVector.record(loRaTP);
    send(request, "appOut");
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT++;
        if(ADR_ACK_CNT >= ADR_ACK_LIMIT) sendNextPacketWithADRACKReq = true;
        if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY)
        {
            ADR_ACK_CNT = 0;
            increaseSFIfPossible();
        }
    }
    emit(LoRa_AppPacketSent, loRaSF);
}

void SimpleLoRaApp::increaseSFIfPossible()
{
    if(loRaTP < 14) loRaTP++;
    if(loRaTP >= 14) {
        loRaTP = 14;
        if(loRaSF < 12){ 
            loRaSF++;
        }
    }
}


std::string SimpleLoRaApp::str() const
{
    return "SimpleLoRaApp";
}

} //end namespace inet
