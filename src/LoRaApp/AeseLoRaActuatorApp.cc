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

#include <AeseLoRaActuatorApp.h>
#include "inet/mobility/static/StationaryMobility.h"
// #include "inet/mobility/single/LinearMobility.h"

namespace inet {

Define_Module(AeseLoRaActuatorApp);

void AeseLoRaActuatorApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    schedulerClass = getSimulation()->getScheduler()->str();

    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           // LinearMobility *mobility = check_and_cast<LinearMobility *>(host->getSubmodule("mobility"));
           mobility->par("speed").setDoubleValue(1);
           mobility->par("angle").setDoubleValue(270);
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
        
        rtEvent = new cMessage("rtEvent");
        lastSequenceNumber = -1;
        if(schedulerClass == "cSimulinkRTScheduler"){
            rtScheduler = check_and_cast<cSimulinkRTScheduler *>(getSimulation()->getScheduler());
            rtScheduler->setInterfaceModule(this, rtEvent, recvBuffer, 4000, &numRecvBytes,false,true);
            previousReading = -99;
        }

        e2edelay.setName("EndtoEndDelayActuator");
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");

        LoRaAeseMac *macLayer = check_and_cast<LoRaAeseMac *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
        macLayer->setCarrierFrequency(units::values::Hz(par("initialLoRaCF").doubleValue()));     
        //macLayer->setNodeType(ACTUATOR);
        macLayer->setSensor(false);
        // std::vector<int> array;
        // std::stringstream ss(macLayer->getAddress().str());
        // std::string line;
        // while (std::getline(ss,line,'-'))
        //     array.push_back(std::strtol(line.c_str(), 0, 16));
        actuatorNumber = DevAddr::generateActuatorNumber();
        // std::cout << macLayer->getAddress().str() << std::endl;
        // std::cout << actuatorNumber << std::endl;
    }
}

std::pair<double,double> AeseLoRaActuatorApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
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

void AeseLoRaActuatorApp::finish()
{
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    // LinearMobility *mobility = check_and_cast<LinearMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    cancelAndDelete(rtEvent);
}

void AeseLoRaActuatorApp::handleMessage(cMessage *msg)
{
    handleMessageFromLowerLayer(msg);
    delete msg;
}

void AeseLoRaActuatorApp::handleMessageFromLowerLayer(cMessage *msg)
{
    AeseAppPacket *packet = check_and_cast<AeseAppPacket *>(msg);
    if(packet->getMsgType() == ACTUATION){
        // std::cout << "Data receied from Gateway at actuator: " << packet->getActuationSignal(actuatorNumber) << std::endl;
        // if(actuatorNumber > 6 && packet->getSystemNumber() == 3){
        //     rtScheduler->sendValue(packet->getActuationSignal(actuatorNumber),this);
        // }
        // else if(actuatorNumber > 2 && packet->getSystemNumber() == 2){
        //     rtScheduler->sendValue(packet->getActuationSignal(actuatorNumber),this);
        // }
        // else if(actuatorNumber > -1 && packet->getSystemNumber() == 1){
        //     rtScheduler->sendValue(packet->getActuationSignal(actuatorNumber),this);
        // }
        // std::cout << actuatorNumber << " " << packet->getActuationSignal(actuatorNumber) << std::endl;
        for(int i=0;i<128;i++){
            if(packet->getActuatorNumber(i) == actuatorNumber){
                if(packet->getActuatorSequenceNumbers(i) > lastSequenceNumber){
                    e2edelay.record(timeOfLastPacket);
                    lastSequenceNumber = packet->getActuatorSequenceNumbers(i);
                    timeOfLastPacket = simTime()-packet->getPacketGeneratedTime(i);
                }else if(packet->getActuatorSequenceNumbers(i) == lastSequenceNumber){
                    timeOfLastPacket = simTime()-packet->getPacketGeneratedTime(i);
                }
            }
        }
        if(packet->getActuationSignal(actuatorNumber) != previousReading){
            previousReading = packet->getActuationSignal(actuatorNumber);
            if(schedulerClass == "cSimulinkRTScheduler"){
                rtScheduler->sendValue(packet->getActuationSignal(actuatorNumber),this);
            }
        }
    }
}

bool AeseLoRaActuatorApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}


std::string AeseLoRaActuatorApp::str() const
{
    return "AeseLoRaActuatorApp";
}

} //end namespace inet
