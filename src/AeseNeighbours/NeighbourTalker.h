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

#ifndef __AESE_NEIGHBOUR_TALKER_H_
#define __AESE_NEIGHBOUR_TALKER_H_

#include <omnetpp.h>
#include "RadioControlInfo_m.h"
#include <vector>
#include "inet/common/INETDefs.h"
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "../misc/cSimulinkRTScheduler.h"
#include "SupportedProtocols.h"
#include "LoRaGWMac.h"

#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"
#include "NeighbourTalkerMessage_m.h"
#include "AeseAppPacket_m.h"
#include "DevAddrMessage_m.h"

namespace inet {

class INET_API NeighbourTalker : public cSimpleModule, public cListener
{
  protected:

    enum AeseGWModes{
        NO_NEIGHBOUR,
        ALL_NEIGHBOUR,
        NEIGHBOUR_WITH_BIDS,
        NEIGHBOUR_WITH_BIDS_RANDOM
    };

    struct ReceivedPacket
    {
        // DevAddr is used as the key to access the Received Packet
        SupportedProtocols protocol;
        int lastSeqNo;
        LoRaMacFrame* frame;
        simtime_t insertionTime;
        double RSSI;
        ReceivedPacket() {}
        ReceivedPacket(SupportedProtocols protocol,int lastSeqNo, LoRaMacFrame* frame, simtime_t insertionTime, double RSSI) :
            protocol(protocol), lastSeqNo(lastSeqNo), frame(frame), insertionTime(insertionTime), RSSI(RSSI) {}
    };

    struct GatewayInfo
    {
        // DevAddr is used as the key to access the Received Packet
        SupportedProtocols protocol;
        unsigned int trustValue = 0.5;
        std::vector<DevAddr> knownNodes;
        simtime_t lastSeen;
        GatewayInfo() {}
        GatewayInfo(SupportedProtocols protocol, simtime_t lastSeen) :
            protocol(protocol), lastSeen(lastSeen) {}
    };

    struct DownlinkPacket
    {
        simtime_t addedToQueue;
        simtime_t deadByTime;
        int sequenceNumber;
        DevAddr addr;
        LoRaMacFrame* frame;
        DownlinkPacket() {}
        DownlinkPacket(simtime_t addedToQueue, simtime_t deadByTime, int sequenceNumber, DevAddr addr, LoRaMacFrame* frame) :
            addedToQueue(addedToQueue), deadByTime(deadByTime), sequenceNumber(sequenceNumber) , addr(addr), frame(frame) {}
    };

    struct DevAddr_compare
    {
        bool operator()(const DevAddr& d1, const DevAddr& d2) const { return d1.compareTo(d2) < 0; }
    };

    typedef std::map<DevAddr, ReceivedPacket, DevAddr_compare> ReceivedPacketsMap;
    ReceivedPacketsMap *ReceivedPacketsList = nullptr;

    typedef std::map<DevAddr, GatewayInfo, DevAddr_compare> GatewayNeighbours;
    GatewayNeighbours *GatewayNeighboursList = nullptr;

    struct BidQueue
    {
        DevAddr addr;
        std::list<std::tuple<L3Address,double>> gatewaysThatBid;
    };

  private:
    AeseGWModes AeseGWMode;
    simtime_t periodicPingInterval;
    cMessage *transmitPingMessage;
    cMessage *checkAnyUnsentMessages;
    // cMessage *devAddrMessage;
    SupportedProtocols currentProtocol;

  protected:
    UDPSocket socket;
    std::vector<L3Address> gwAddresses;
    int numberOfGateways;
    std::list<DownlinkPacket> downlinkList;
    typedef std::map<DevAddr,int, DevAddr_compare> DownlinkLastDropRequest;
    DownlinkLastDropRequest *DownlinkLastDropRequestList = nullptr;
    LoRaGWMac *loRaGwMac;
    std::vector<BidQueue> currentBids;
    DownlinkLastDropRequest *NeighbourDropRequestList = nullptr;
    long failedBids;
    long requestedBids;
    long acceptedBids;

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual std::string str() const override { return "NeighbourTalker"; };
    void handleLoRaFrame(cPacket* pkt);
    void transmitPing();
    void handleDownlinkQueue();
    void transmitLoRaMessage();
    void startUDP();
    void canIHandleThisMessage(cPacket* pkt);
    void sendConfirmationToNeighbour(cPacket* pkt);
    void collectBids(L3Address gateway, DevAddr addr, double RSSI);
    void acceptBid(DevAddr addr);
    void checkConfirmationAndDelete(DevAddr addr);

  public:
    virtual ~NeighbourTalker();
    void handleLowerLayer(cPacket* pkt);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char* s, cObject* details ) override;

};
} //namespace inet
#endif
