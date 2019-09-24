#ifndef __AESE_PACKET_DUPLICATOR_H
#define __AESE_PACKET_DUPLICATOR_H

#include "inet/common/INETDefs.h"

#include "inet/common/lifecycle/ILifecycle.h"


namespace inet {

class INET_API PacketDuplicator :public cSimpleModule, public ILifecycle
{
  public:
    void initialize(int stage);
    void handleMessage(cMessage *msg);
    void broadcastFrame(cMessage *msg);
    void finish();
    int numPorts;
    virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;

};

} // namespace inet

#endif // ifndef __AESE_PACKET_DUPLICATOR_H

