#ifndef __AESE_PRIORITY_HANDLING_SCHEDULING_H_
#define __AESE_PRIORITY_HANDLING_SCHEDULING_H_

#include <omnetpp.h>
#include "inet/common/INETDefs.h"
#include "inet/applications/base/ApplicationBase.h"

#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"
#include "timeOnAir.h"

namespace inet {

class INET_API PriorityHandlingAndScheduling : public cSimpleModule, public cListener
{
  protected:
    enum MessagesTypesAndPriority
    {
      MY_ACKS = 1,
      ACCEPTED_BIDS_ACKS = 2,
      FIND_NEIGHBOURS_FOR_DOWNLINK=3,
      FIND_NEIGHBOURS_FOR_UPLINK=4,
      SEND_ACCEPT_BIDS_FOR_NEIGHBOURS=5,
    };
    
    struct LoRaSendQueue
    {
        simtime_t sendingTime;
        simtime_t freeAfter;
        MessagesTypesAndPriority type;
        DevAddr addr;
        LoRaMacFrame* frame;
        LoRaSendQueue() {}
        LoRaSendQueue(simtime_t sendingTime, simtime_t freeAfter, MessagesTypesAndPriority type, DevAddr addr,LoRaMacFrame* frame) :
            sendingTime(sendingTime), freeAfter(freeAfter),type(type), addr(addr),frame(frame) {}
    };

    cMessage *sendMessageFromQueue0;
    cMessage *sendMessageFromQueue1;

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual std::string str() const override { return "PriorityHandlingAndScheduling"; };
    std::list<LoRaSendQueue> sendingQueue[2];
    void handleLowerPacket(cPacket *msg);
    bool handleUpperPacket(cPacket *msg);

    simtime_t freeAfterCurrent[2];
    simtime_t freeAfterLast[2];
    int lastPriority[2];

  public:
    virtual ~PriorityHandlingAndScheduling();
    bool canThisBeScehduled(int band, int priority, simtime_t sendingTime);

    // virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char* s, cObject* details ) override;

};
} //namespace inet
#endif
