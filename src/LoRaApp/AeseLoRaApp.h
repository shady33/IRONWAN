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

#ifndef __LORA_OMNET_AESELORAAPP_H_
#define __LORA_OMNET_AESELORAAPP_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "AeseAppPacket_m.h"
#include "../LoRa/LoRaMacControlInfo_m.h"
#include "../misc/cSimulinkRTScheduler.h"
#include "../LoRa/LoRaMac.h"
#include "../LoRa/LoRaAeseMac.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>

using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class INET_API AeseLoRaApp : public cSimpleModule, public ILifecycle
{
    private:
        cMessage *rtEvent;
        cSimulinkRTScheduler *rtScheduler;

        unsigned char recvBuffer[4000];
        int numRecvBytes;
        simtime_t timeOfLastPacket;
        double dataToSend;
        bool first; 
        std::string schedulerClass;
        int sequenceNumber;
        
        /* Stuff for opening a file*/
        void loadfile(std::string filename,int nodeID);
        std::string strBetweenTwoStr(const std::string &s,
            const std::string &start_delim,
            const std::string &stop_delim);
        int nextTime;
        double allData[4000];
        double prevTime;
        int maxTime;
        bool multiplePackets;
        int sensorNumber;

    protected:
        virtual void initialize(int stage) override;
        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;

        void handleMessageFromLowerLayer(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendDownMgmtPacket();
        void sendMessageToQueue(double value = 0);
        double getNextTime();
        int numberOfPacketsToSend;
        int sentPackets;
        int receivedADRCommands;
        int lastSentMeasurement;
        simtime_t timeToFirstPacket;
        simtime_t timeToNextPacket = -1;
        double transmissionTimeTable[6];
        cMessage *configureLoRaParameters;
        cMessage *sendMeasurements;
        cMessage *sendMessage;
        
        //history of sent packets;
        cOutVector sfVector;
        cOutVector tpVector;

        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

    public:
        AeseLoRaApp() {}
        simsignal_t LoRa_AppPacketSent;
        //LoRa physical layer parameters
        double loRaTP;
        units::values::Hz loRaCF;
        int loRaSF;
        units::values::Hz loRaBW;
        int loRaCR;
        bool loRaUseHeader;
        double loRaDC;

        virtual std::string str() const override;
};

}

#endif
