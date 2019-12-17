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

#ifndef __LORANETWORK_PACKETFORWARDER_H_
#define __LORANETWORK_PACKETFORWARDER_H_

#include <omnetpp.h>
#include "RadioControlInfo_m.h"
#include <vector>
#include "inet/common/INETDefs.h"

#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include "../misc/cSimulinkRTScheduler.h"
#include "../LoRaApp/AeseAppPacket_m.h"
#include "inet/mobility/static/StationaryMobility.h"

namespace inet {

class nodeLog
{
public:
    DevAddr srcAddr;
    int framesFromLastADRCommand;
    int lastSeqNoProcessed;
    int numberOfSentADRPackets;
    std::list<double> receivedSNIR;
    cOutVector *historyAllSNIR;
    cOutVector *historyAllRSSI;
    cOutVector *receivedSeqNumber;
    cOutVector *calculatedSNRmargin;
};

class INET_API PacketForwarder : public cSimpleModule, public cListener
{
  private:
    cMessage *rtEvent;
    cMessage *sendDownlink[8];
    cMessage *sendFeedback;
    cMessage *sendActuation;
    cMessage *sendActuationNoSimulink;

    double transmissionTimeTable[6];
    unsigned char recvBuffer[4000];
    int numRecvBytes;
    simtime_t timeOfLastPacket;
    LoRaMacFrame *frameCopy[8];
    unsigned char cnt = 0;
    std::string schedulerClass;
    simtime_t timeToStartSignal;
    simtime_t actuationPeriod;

    int DTQ = 0;
    int CRQ = 0;
    int noOfMslots;
    int noOfNslots;
    simtime_t mSlotDuration; //s
    simtime_t nSlotDuration; //s
    simtime_t frameDuration;
    int miniSlotInformation[64];
    int dataSlotInformation[64];
    bool dataExpectedfromMiniSlot;
    int miniSlotsWithDataExpected[64];

    bool enableDQ;
    int numberOfChannels;
    bool dataOnSameChannel;
    int dataQueues[8];
    int requestedSlots[64];
    cOutVector dataReceived;
    cOutVector MiniSlots;
    cOutVector CRQVector;
    cOutVector DTQVector;

    int numberOfNodes;
    int numberOfAeseNodes;
    int numberOfAeseActuatorNodes;
    int numberOfSubSystems;

    bool enableActuation;
    bool sendImmediateActuation;
    simtime_t packetGeneratedTime[128];
    int actuatorNumbers[128];
    int actuatorSequenceNumbers[128];
    int currentCntActuators;
    int numberOfNS;
    int gwNSNumber;
    int sentMsgs;
  protected:
    std::vector<L3Address> destAddresses;
    std::vector<L3Address> gwAddresses;
    int localPort = -1, destPort = -1;
    int numberOfGateways;
    // state
    UDPSocket socket;
    cMessage *selfMsg = nullptr;
    cPacketQueue actuationQueue;
    cSimulinkRTScheduler *rtScheduler;
    std::vector<nodeLog> knownNodes;
    double totalUsedTimes;
  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    void processLoraMACPacket(cPacket *pk);
    void processPacketMatlab();
    void startUDP();
    void sendPacket(int val);
    void setSocketOptions();
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    void receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details) override;
    void receiveSignal(cComponent *src, simsignal_t id,double value, cObject *details) override;
    void scheduleDownlink(int val,cPacket *pk);
    void sendFeedbackMessage(bool first);
    void sendActuationMessage();
    void sendActuationMessageNow(AeseAppPacket *appPacket);
    void updateAndLogNode(LoRaMacFrame* pkt);
    void CountUniqueNodes(LoRaMacFrame* pkt);
    void GWinGrid();

  public:
    virtual ~PacketForwarder();
    simsignal_t LoRa_GWPacketReceived;
    int counterOfSentPacketsFromNodes = 0;
    int counterOfReceivedPackets = 0;
};
} //namespace inet
#endif
