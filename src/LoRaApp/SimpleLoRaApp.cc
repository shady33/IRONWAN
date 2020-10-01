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
           mobility->par("speed").setDoubleValue(0);
           mobility->par("angle").setDoubleValue(270);
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
        connected = false;
	    switched = false;
        rtEvent = new cMessage("rtEvent");
        retryLimit = par("retryLimit");

        if(retryLimit == 0){
            NodeParams* nodeParams = dynamic_cast<NodeParams*>(getSimulation()->getModuleByPath("nodeParams"));
            retryLimit = nodeParams->getRetryLimit();
        }

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
        numberOfAcks = 0;
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

        timeToNextPacketOnce = par("timeToNextPacket");

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
        sfVector.setName("SFVector");
        tpVector.setName("TPVector");
        sfVector.disable();
        tpVector.disable();

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
    // std::cout << "Number of generated packets:" << sentPackets << std::endl;
    // std::cout << getParentModule() << numberOfAcks << std::endl;
    // std::cout << "Number of retransmits: " << totalNoOfRetransmits << std::endl;

    cModule *host = getContainingNode(this);
    LinearMobility *mobility = check_and_cast<LinearMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    recordScalar("sentPackets", sentPackets);
    if(retryLimit > 1){
        // std::cout << "PDR:" << numberOfAcks << " " << sentPackets << std::endl;
        recordScalar("PDR",((float)numberOfAcks)/((float)sentPackets));
        recordScalar("AckedPacketsTx", sentPackets);
    }else
        recordScalar("UnAckedPacketsTx", sentPackets);

    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("receivedAckMessages", receivedAckMessages);
    recordScalar("TotalNumberOfRetransmits",totalNoOfRetransmits);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if(msg == retryMeasurements)
        {
            noOfRetransmits++;
            if(noOfRetransmits < retryLimit){
                if(!connected)
                    sendJoinRequest();
                else
                    sendDataPacket();
                scheduleAt(simTime() + uniform(10,15),retryMeasurements);
            }else{
                totalNoOfRetransmits += noOfRetransmits;
                noOfRetransmits = 0;
            }
        }
        else if (msg == sendMeasurements)
        {
            if(!retryMeasurements->isScheduled())
            {
                startTime = simTime();
                seqeuenceNumber++;
                if(!connected)
                    sendJoinRequest();
                else
                    sendDataPacket();
                noOfRetransmits = 0;
                scheduleAt(simTime() + uniform(10,15),retryMeasurements);
            }
            sentPackets++;

            // Schedule Next send measurements
            int i = 0;
            double time = timeOnAir(loRaSF, loRaBW, 40, 1)*100;
            do {
                // timeToNextPacket = par("timeToNextPacket");
                timeToNextPacket = timeToNextPacketOnce;
                i = i + 1;
                if(i==5) break;
            } while(timeToNextPacket <= time);
	    if((simTime() > 86400) && (!switched)){
	    	switched = true;
		if(uniform(0,10) < 4){
			timeToNextPacketOnce = 600;
		}
	    }
            if(numberOfPacketsToSend == 0 || numberOfPacketsToSend > sentPackets)
                scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
	        else{
		        numberOfPacketsToSend = numberOfPacketsToSend * 2;
		        scheduleAt(simTime() + uniform(21600,64800),sendMeasurements);
	        }
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
    if(packet->getMsgType() == JOIN_REPLY){
        receivedAckMessages++;
        loRaCF = packet->getLoRaCF();
        connected = true;
    }else{
        if(retryLimit > 1){
            numberOfAcks += 1;
            cancelEvent(retryMeasurements);
            e2edelay.record(simTime()-startTime);
            retransmits.record(noOfRetransmits);
            totalNoOfRetransmits += noOfRetransmits;
            noOfRetransmits = 0;
        }
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


void SimpleLoRaApp::sendDataPacket()
{
    AeseAppPacket *request = new AeseAppPacket("DataFrame");
    request->setMsgType(DATA);
    lastSentMeasurement = rand();
    request->setSampleMeasurement(dataToSend);
    request->setPacketGeneratedTime(0,startTime);
    double timeOnAirValue = timeOnAir(loRaSF, loRaBW, 40, 1);
    request->setPacketGeneratedTime(1,simTime()+timeOnAirValue);
    request->setSensorNumber(sensorNumber);
    request->setActuatorSequenceNumbers(0,seqeuenceNumber);
    
    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    cInfo->setNumberOfFrames(1);
    cInfo->setPayloadLength(40);
    cInfo->setGeneratedTime(simTime()+timeOnAirValue);
    if(retryLimit == 1){
        cInfo->setConfirmedMessage(false);
    }else{
        cInfo->setConfirmedMessage(true);
    }
    request->setControlInfo(cInfo);
    sfVector.record(loRaSF);
    tpVector.record(loRaTP);
    send(request, "appOut");
    emit(LoRa_AppPacketSent, loRaSF);
}

void SimpleLoRaApp::sendJoinRequest()
{
    AeseAppPacket *request = new AeseAppPacket("Join_Request");
    request->setMsgType(JOIN_REQUEST);
    lastSentMeasurement = rand();
    request->setSampleMeasurement(dataToSend);
    request->setPacketGeneratedTime(0,startTime);
    double timeOnAirValue = timeOnAir(loRaSF, loRaBW, 40, 1);
    request->setPacketGeneratedTime(1,simTime()+timeOnAirValue);
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
    cInfo->setGeneratedTime(simTime()+timeOnAirValue);
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
