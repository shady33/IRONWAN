#ifndef __AESE_PERIOD_CALCULATOR_H_
#define __AESE_PERIOD_CALCULATOR_H_

#include <omnetpp.h>
#include "inet/applications/base/ApplicationBase.h"
#include "DevAddr.h"
#include "LoRaMacFrame_m.h"
#include "DevAddrMessage_m.h"
#include "AeseAppPacket_m.h"
#include "NeighbourTalkerMessage_m.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cmath>
#include <deque>
#include "PacketForwarder.h"

namespace inet {

class INET_API PeriodCalculator : public cSimpleModule, public cListener
{
  private:
    double tValue;

    struct DevAddr_compare
    {
        bool operator()(const DevAddr& d1, const DevAddr& d2) const { return d1.compareTo(d2) < 0; }
    };

    struct Statistics {
      double mean;
      double median;
      double stdev;
      double tValue;
    };

    Statistics calculateTValueAndOthers(const std::deque<double> &i_vector);

    struct NodePeriodInfo
    {
      // Devaddr is the key used
      double currentPeriod;
      simtime_t lastReceivedTime;
      int lastSeqNo;
      int numberOfMessagesSeen;
      int timesTried;
      bool isConfirmed;
      std::deque<double> listOfPeriods;
      cOutVector *allPeriods;
      DevAddrMessage *msg;
      NodePeriodInfo(){}
      NodePeriodInfo(simtime_t lastReceivedTime,int seqno,bool isConfirmed):
          lastReceivedTime(lastReceivedTime), lastSeqNo(seqno), numberOfMessagesSeen(1),currentPeriod(0),isConfirmed(isConfirmed) {}
    };

    typedef std::map<DevAddr, NodePeriodInfo, DevAddr_compare> NodePeriodsStruct;
    NodePeriodsStruct *NodePeriodsList = nullptr;

    typedef std::map<DevAddr, int, DevAddr_compare> NodesBelongToMeStruct;
    NodesBelongToMeStruct *NodesBelongToMe = nullptr;

    int gwNSNumber;
    int requestedperiods;
    int AeseGWMode;
  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual std::string str() const override { return "PeriodCalculator"; };
    void handleLoRaFrame(cPacket *pkt);
    void transmitFindRequest(DevAddr txAddr,int lastSeqNo,bool isConfirmed);
  public:
    virtual ~PeriodCalculator();
};
}
#endif