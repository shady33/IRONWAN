#include "PeriodCalculator.h"

namespace inet {

Define_Module(PeriodCalculator);

PeriodCalculator::~PeriodCalculator()
{
    auto iter = NodePeriodsList->begin();
    while(iter != NodePeriodsList->end()) {
        NodePeriodInfo& nodePeriodInfo = iter->second;
        cancelAndDelete(nodePeriodInfo.msg);
        delete nodePeriodInfo.allPeriods;
        iter++;
    }
}

void PeriodCalculator::finish()
{
    recordScalar("RequestedPeriods",requestedperiods);
}

void PeriodCalculator::initialize(int stage)
{
    if (stage == 0) {
        tValue = par("tValue"); 
        NodePeriodsList = new NodePeriodsStruct();
        requestedperiods = 0;
        AeseGWMode = (int)par("AeseGWMode");
        NodesBelongToMe = new NodesBelongToMeStruct();
    }else if (stage == INITSTAGE_APPLICATION_LAYER) {
        gwNSNumber = (check_and_cast<PacketForwarder *>(getParentModule()->getSubmodule("packetForwarder")))->getGatewayNsNumber();
    }
}

void PeriodCalculator::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        std::string className(msg->getClassName());
        if(className.compare("inet::LoRaMacFrame") == 0)
            handleLoRaFrame(PK(msg));
        else if(className.compare("inet::NeighbourTalkerMessage") == 0){
            NeighbourTalkerMessage *f = check_and_cast<NeighbourTalkerMessage*>(msg);
            DevAddr d = f->getDeviceAddress();
            (*NodesBelongToMe)[d] = 1;
            delete msg;
        }
    }else if(msg->isSelfMessage()){
        if(!strcmp(msg->getName(),"MessageNotReceivedFindIt")){
            DevAddr addr = check_and_cast<DevAddrMessage*>(msg)->getAddr();
            auto iter = NodePeriodsList->find(addr);
            NodePeriodInfo& nodePeriodInfo = iter->second;
            // std::cout << simTime() << ":Message Failed for:" << addr << " should we find it?"<< nodePeriodInfo.currentPeriod << std::endl;
            // if( nodePeriodInfo.timesTried < 2 )
            transmitFindRequest(addr,nodePeriodInfo.lastSeqNo);
            nodePeriodInfo.timesTried = nodePeriodInfo.timesTried + 1;
            
            if(nodePeriodInfo.currentPeriod > 0){
                if(AeseGWMode != 3){
                    auto iter = NodesBelongToMe->find(addr);
                    if(iter != NodesBelongToMe->end()){
                        scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2, nodePeriodInfo.msg);
                    }
                }else{
                    scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2, nodePeriodInfo.msg);
                }
            }

            // if( (nodePeriodInfo.currentPeriod > 0) && (AeseGWMode > 0) )
                // scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2, nodePeriodInfo.msg);
        }
    }else delete msg;
}

void PeriodCalculator::handleLoRaFrame(cPacket *pkt)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(pkt);
    if((frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS) && (frame->getMsgType() != GW_PING_MESSAGE) && (frame->getTransmitterAddress().getAddressByte(0) == gwNSNumber)){
        // This is a Broadcast packet, and we are only interested in uplinks from nodes
        DevAddr txAddr = frame->getTransmitterAddress();
        auto iter = NodePeriodsList->find(txAddr);
        if(iter == NodePeriodsList->end()){
            // Not found in table, insert
            (*NodePeriodsList)[txAddr] = NodePeriodInfo(simTime(),frame->getSequenceNumber());
            
            // Bad implementation, fix this 
            auto iter = NodePeriodsList->find(txAddr);
            NodePeriodInfo& nodePeriodInfo = iter->second;
            nodePeriodInfo.timesTried = 0;
            nodePeriodInfo.msg = new DevAddrMessage("MessageNotReceivedFindIt");
            nodePeriodInfo.msg->setAddr(txAddr);
            nodePeriodInfo.allPeriods = new cOutVector;
            nodePeriodInfo.allPeriods->setName("AllPeriodsDataForNodes");
        }else{
            // Check if already sceduled, if yes, cancel
            // Already in table, delete old and add new
            // All the T_value stuff is part of this loop
            NodePeriodInfo& nodePeriodInfo = iter->second;
            nodePeriodInfo.timesTried = 0;
            if(nodePeriodInfo.msg->isScheduled()){
                cancelEvent(nodePeriodInfo.msg);
            }
            simtime_t timeBetweenMessages = simTime() - nodePeriodInfo.lastReceivedTime;
            AeseAppPacket *packet = check_and_cast<AeseAppPacket *>((frame)->decapsulate());
            if((nodePeriodInfo.lastSeqNo < packet->getActuatorSequenceNumbers(0))){
                // If it is a new message then continue
                if(std::abs(SIMTIME_DBL(nodePeriodInfo.currentPeriod - timeBetweenMessages)) > 1){
                    double x_n = SIMTIME_DBL(timeBetweenMessages)/(packet->getActuatorSequenceNumbers(0) - nodePeriodInfo.lastSeqNo);
                    nodePeriodInfo.listOfPeriods.push_back(x_n);
                }else{
                    nodePeriodInfo.listOfPeriods.clear();
                }
                nodePeriodInfo.numberOfMessagesSeen += 1;
                nodePeriodInfo.lastReceivedTime = simTime();
                nodePeriodInfo.lastSeqNo = packet->getActuatorSequenceNumbers(0);
                // Exactly 10 elements in the array
                if(nodePeriodInfo.listOfPeriods.size() == 10){
                    Statistics stats = calculateTValueAndOthers(nodePeriodInfo.listOfPeriods);
                    if((stats.tValue <= tValue) || (stats.stdev == 0)){
                        nodePeriodInfo.currentPeriod = stats.median;
                        nodePeriodInfo.listOfPeriods.clear();
                    }else{
                        nodePeriodInfo.listOfPeriods.pop_front();
                    }
                }

                // Start a timer if we can do the period stuff here
                if(nodePeriodInfo.numberOfMessagesSeen > 11){
                    nodePeriodInfo.allPeriods->record(nodePeriodInfo.currentPeriod);
                    if(nodePeriodInfo.currentPeriod > 0){
                        if(AeseGWMode != 3){
                            auto iter = NodesBelongToMe->find(txAddr);
                            if(iter != NodesBelongToMe->end()){
                                scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2, nodePeriodInfo.msg);
                            }
                        }else{
                            scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2, nodePeriodInfo.msg);
                        }
                    }  
                    // if( (nodePeriodInfo.currentPeriod > 0) && (AeseGWMode > 0) )
                        // scheduleAt(simTime() + nodePeriodInfo.currentPeriod + 0.2,nodePeriodInfo.msg);
                }
            }
            delete packet;
        }
    }
    delete pkt;
}

PeriodCalculator::Statistics PeriodCalculator::calculateTValueAndOthers(const std::deque<double> &i_vector)
{
   Statistics stats;
   std::deque <double> deque(i_vector);
   std::sort(deque.begin(),deque.end());
   double sumD = std::accumulate(deque.begin(),deque.end(),0.0);
   stats.mean = sumD/double(deque.size());
   std::deque <double> Diff(deque.size());
   std::transform(deque.begin(),deque.end(),Diff.begin(),
                  std::bind2nd(std::minus<double> (),stats.mean));
   stats.stdev  = std::inner_product(Diff.begin(),Diff.end(),
                                      Diff.begin(),0.0);
   stats.median = deque[std::floor(double(deque.size())/2.0)];
   stats.stdev = std::sqrt(stats.stdev/deque.size());
   stats.tValue = (stats.mean - stats.median)/(stats.stdev * std::sqrt(deque.size()));
   return stats;
}

void PeriodCalculator::transmitFindRequest(DevAddr txAddr,int lastSeqNo)
{
    if(AeseGWMode > 0){
        NeighbourTalkerMessage *pkt = new NeighbourTalkerMessage("FIND_NEIGHBOURS_FOR_UPLINK");
        pkt->setDeviceAddress(txAddr);
        pkt->setSequenceNumber(lastSeqNo);
        requestedperiods = requestedperiods + 1;
        send(pkt,"lowerLayerOut");
    }
}

}
