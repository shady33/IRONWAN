#ifndef __AESE_NODE_PARAMS_H
#define __AESE_NODE_PARAMS_H

#include "inet/common/INETDefs.h"

namespace inet {

class INET_API NodeParams :public cSimpleModule
{
  public:
    void initialize(int stage);
    int getRetryLimit();
    void finish();

  protected:
    int nodeParamsRetryLimit;
    int numberOfNodes;
    int ratioOfAckToTotal;
    int currentNumberOfAckedNodes;
    int parsedNodes;
};

} // namespace inet

#endif // ifndef __AESE_NODE_PARAMS_H

