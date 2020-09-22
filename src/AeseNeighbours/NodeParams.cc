#include "inet/common/LayeredProtocolBase.h"
#include "inet/common/ModuleAccess.h"
#include "NodeParams.h"
#include "../LoRaApp/SimpleLoRaApp.h"

namespace inet {

Define_Module(NodeParams);

void NodeParams::initialize(int stage)
{
    if(stage == 0){
        numberOfNodes = par("numberOfNodes");
        ratioOfAckToTotal = par("ratioOfAckToTotal");
	nodeParamsRetryLimit = par("nodeParamsRetryLimit");
        currentNumberOfAckedNodes = 0;
        parsedNodes = -1;
    }
}

void NodeParams::finish()
{
    recordScalar("NodesRequestingAcks",currentNumberOfAckedNodes);
}

int NodeParams::getRetryLimit()
{
    parsedNodes++;
    int x = ((numberOfNodes*ratioOfAckToTotal)/100);
    if(ratioOfAckToTotal == 0)
        return 1;
    else if(ratioOfAckToTotal == 100)
        return nodeParamsRetryLimit;
    else{
        int ackNode = bernoulli((float)(ratioOfAckToTotal/100));
        if(currentNumberOfAckedNodes > x)
            return 1;
        if(numberOfNodes - parsedNodes <= (x-currentNumberOfAckedNodes))
            return nodeParamsRetryLimit;
        if(ackNode == 1){
            currentNumberOfAckedNodes += 1;
            return nodeParamsRetryLimit;
        }
        return 1;
    }
}

} // namespace inet

