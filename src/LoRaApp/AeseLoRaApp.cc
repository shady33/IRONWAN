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

#include <AeseLoRaApp.h>
//#include "inet/mobility/static/StationaryMobility.h"
#include "inet/mobility/single/LinearMobility.h"

namespace inet {

Define_Module(AeseLoRaApp);

void AeseLoRaApp::initialize(int stage)
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
        sequenceNumber = 0;

        if(schedulerClass == "cSimulinkRTScheduler"){
            rtScheduler = check_and_cast<cSimulinkRTScheduler *>(getSimulation()->getScheduler());
            rtScheduler->setInterfaceModule(this, rtEvent, recvBuffer, 4000, &numRecvBytes,false,false);
        }
        timeOfLastPacket = 0;
        first = true;
        sensorNumber = DevAddr::generateSensorNumber();

    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {

        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");

        sentPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaDC = par("initialLoRaDC").doubleValue();
        multiplePackets = par("multiplePackets");
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
        
        std::string path = par("FileForTimes");
        if(!path.empty()){
            std::string s = getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac")->str();
            if(s.compare("LoRaMac") == 0){
                LoRaMac *macLayer = check_and_cast<LoRaMac *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
                loadfile(path,macLayer->getAddress().getInt());   
            }else if(s.compare("LoRaAeseMac") == 0){
                LoRaAeseMac *macLayer = check_and_cast<LoRaAeseMac *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
                loadfile(path,macLayer->getAddress().getInt());
            }
        }

        if(schedulerClass != "cSimulinkRTScheduler"){
            timeToFirstPacket = par("timeToFirstPacket");
            if(timeToFirstPacket != 0){
                do {
                    timeToFirstPacket = par("timeToFirstPacket");
                } while(timeToFirstPacket <= 5);
                sendMessage = new cMessage("sendMessage");
                if(!path.empty()){
                    scheduleAt(getNextTime(),sendMessage);
                }else{
                    scheduleAt(simTime()+timeToFirstPacket,sendMessage);
                }
            }
        }
        LoRaAeseMac *macLayer = check_and_cast<LoRaAeseMac *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
        macLayer->setCarrierFrequency(units::values::Hz(par("initialLoRaCF").doubleValue()));
        //macLayer->setNodeType(SENSOR);
        macLayer->setSensor(true);
    }
}

void AeseLoRaApp::loadfile(std::string filename,int nodeID) 
{
    nextTime = 7;
    std::ifstream file(filename);

    std::string line = "";
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(","));
        if(vec[7].compare("TIME")){
            if(std::stoi(strBetweenTwoStr(vec[5],"[","]")) == nodeID-1){
                for(unsigned int i = 8; i < vec.size(); i++) {
                    allData[i] = (stod(vec[i]));
                }
                maxTime = vec.size();
                break;
            }
        }
    }
    // Close the File
    file.close();
}

std::string AeseLoRaApp::strBetweenTwoStr(const std::string &s,
        const std::string &start_delim,
        const std::string &stop_delim)
{
    unsigned first_delim_pos = s.find(start_delim);
    unsigned end_pos_of_first_delim = first_delim_pos + start_delim.length();
    unsigned last_delim_pos = s.find(stop_delim);

    return s.substr(end_pos_of_first_delim,
            last_delim_pos - end_pos_of_first_delim);
}

double AeseLoRaApp::getNextTime(){
    nextTime++;
    if(nextTime < maxTime){
        prevTime = allData[nextTime];
        return allData[nextTime];
    }
    else
        return 0;
}

std::pair<double,double> AeseLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
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

void AeseLoRaApp::finish()
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
    cancelAndDelete(rtEvent);
    cancelAndDelete(sendMessage);
}

void AeseLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        std::string s(msg->getName());
        if(s.compare("rtEvent") == 0)
        {
            if(numRecvBytes != 0){
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
                
                // dataToSend = value;
                sendMessageToQueue(value);
                // if(((time - timeOfLastPacket) > dutyCycling) || first){
                //     // std::cout << "num received bytes" << numRecvBytes << std::endl;
                //     first = false;
                //     dataToSend = value;
                //     sendJoinRequest();
                //     // std::cout << " Value:" << value << std::endl;
                //     timeOfLastPacket = time;
                //     // std::cout << "Time of last packet:" << timeOfLastPacket << std::endl;
                // }

                numRecvBytes = 0;
            }
            delete msg;
        }
        else if (msg == sendMeasurements)
        {
            sendJoinRequest();
            if (simTime() >= getSimulation()->getWarmupPeriod())
                sentPackets++;
            delete msg;
            if(numberOfPacketsToSend == 0 || sentPackets < numberOfPacketsToSend)
            {
                double time;
                if(loRaSF == 7) time = transmissionTimeTable[0];
                if(loRaSF == 8) time = transmissionTimeTable[1];
                if(loRaSF == 9) time = transmissionTimeTable[2];
                if(loRaSF == 10) time = transmissionTimeTable[3];
                if(loRaSF == 11) time = transmissionTimeTable[4];
                if(loRaSF == 12) time = transmissionTimeTable[5];
                do {
                    timeToNextPacket = par("timeToNextPacket");
                    //if(timeToNextPacket < 3) error("Time to next packet must be grater than 3");
                } while(timeToNextPacket <= time);
                sendMeasurements = new cMessage("sendMeasurements");
                scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
            }
        }
        else if(msg == sendMessage)
        {
            sendMessageToQueue();
            do {
                timeToNextPacket = par("timeToNextPacket");
            } while(timeToNextPacket <= transmissionTimeTable[0]);

            cancelAndDelete(sendMessage);
            sendMessage = new cMessage("sendMessage");
            std::string path = par("FileForTimes");
            if(!path.empty()){
                double d = getNextTime();
                if(d != 0)
                    scheduleAt(d, sendMessage);
            }else{
                scheduleAt(simTime() + timeToNextPacket, sendMessage);
            }
        }
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        delete msg;
    }
}

void AeseLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    if(packet->getKind() == DATADOWN){
        std::cout << "Data receied from Gateway at node: " << packet->getSampleMeasurement() << std::endl;
        rtScheduler->sendValue(packet->getSampleMeasurement(),this);
    }else if(packet->getKind() == DATANOSOCKET){
        std::cout << "Received Downlink not to be sent on socket" << std::endl;
    }else if(packet->getKind() == FEEDBACK){
        std::cout << "Feedback recevied at " << simTime() << std::endl;
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

bool AeseLoRaApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void AeseLoRaApp::sendJoinRequest()
{
    LoRaAppPacket *request = new LoRaAppPacket("DataFrame");
    request->setKind(DATA);
    lastSentMeasurement = rand();
    request->setSampleMeasurement(dataToSend);
    request->setPacketGeneratedTime(0,simTime());
    request->setSensorNumber(sensorNumber);

    if(evaluateADRinNode && sendNextPacketWithADRACKReq)
    {
        request->getOptions().setADRACKReq(true);
        sendNextPacketWithADRACKReq = false;
    }

    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    request->setControlInfo(cInfo);
    //sfVector.record(loRaSF);
    //tpVector.record(loRaTP);
    send(request, "appOut");
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT++;
        if(ADR_ACK_CNT == ADR_ACK_LIMIT) sendNextPacketWithADRACKReq = true;
        if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY)
        {
            ADR_ACK_CNT = 0;
            increaseSFIfPossible();
        }
    }
    emit(LoRa_AppPacketSent, loRaSF);
}

void AeseLoRaApp::sendMessageToQueue(double value)
{
    sequenceNumber++;
    // std::cout << "Message sent to queue " << std::endl;
    LoRaAppPacket *request = new LoRaAppPacket("UplinkMessage");
    request->setMsgType(UPLINK);
    request->setSampleMeasurement(value);
    request->setPacketGeneratedTime(0,simTime());
    request->setSensorNumber(sensorNumber);
    request->setActuatorSequenceNumbers(0,sequenceNumber);

    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    if(multiplePackets){
        int size = intuniform(0,100);
        if(size > 35){
            cInfo->setNumberOfFrames(2);
            cInfo->setPayloadLength(size);
        }
        else{
            cInfo->setNumberOfFrames(1);
            cInfo->setPayloadLength(size);
        }
    }else{
        cInfo->setNumberOfFrames(1);
        cInfo->setPayloadLength(40);
    }
    
    request->setControlInfo(cInfo);
    //sfVector.record(loRaSF);
    //tpVector.record(loRaTP);
    sentPackets++;
    send(request, "appOut");

    emit(LoRa_AppPacketSent, loRaSF);
}

void AeseLoRaApp::increaseSFIfPossible()
{
    if(loRaSF < 12) loRaSF++;
}

std::string AeseLoRaApp::str() const
{
    return "AeseLoRaApp";
}

} //end namespace inet
