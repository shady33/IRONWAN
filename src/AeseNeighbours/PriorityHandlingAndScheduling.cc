#include "PriorityHandlingAndScheduling.h"

namespace inet {

Define_Module(PriorityHandlingAndScheduling);

PriorityHandlingAndScheduling::~PriorityHandlingAndScheduling()
{

}

void PriorityHandlingAndScheduling::finish()
{
    cancelAndDelete(sendMessageFromQueue0);
    cancelAndDelete(sendMessageFromQueue1);
}

bool PriorityHandlingAndScheduling::handleUpperPacket(cPacket *msg)
{
    bool scheduled = false;
    if(rand()%10 <2)
        return scheduled;
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
    frame->removeControlInfo();
    simtime_t sendingTime = frame->getSendingTime();
    int channelNumber = ((frame->getLoRaCF() - inet::units::values::Hz(868100000))/inet::units::values::Hz(200000)).get();
    int band = 0; // 0 - 868.0 - 868.8
    auto sendTimer = sendMessageFromQueue0;
    int waitTime = 99;
    if(channelNumber > 3){
        band = 1;
        sendTimer = sendMessageFromQueue1;
        waitTime = 9;
    }
    
    int PayloadLength = frame->getPayloadLength();
    if(PayloadLength == 0)
        PayloadLength = 20;
    
    double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
    
    if((sendingTime >= freeAfterCurrent[band])){
        scheduled = true;
    }else{
        // A packet already scheduled has lower priority
        if((frame->getType() < lastPriority[band]) && (simTime() < sendMessageFromQueue0->getArrivalTime())){
            scheduled = true;
        }
    }
    if(scheduled){
        sendingQueue[band].emplace_back(sendingTime,sendingTime+(delta*waitTime),(MessagesTypesAndPriority)frame->getType(),frame->getReceiverAddress(),frame);
        if(sendTimer->isScheduled()){
            if(sendTimer->getArrivalTime() > sendingTime){
                cancelEvent(sendTimer);
            }
        }
        scheduleAt(sendingTime,sendTimer);
        lastPriority[band] = frame->getType();
        freeAfterCurrent[band] = sendingTime + (delta * waitTime);
    }
    return scheduled;
}

void PriorityHandlingAndScheduling::handleLowerPacket(cPacket *msg)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(msg);
    if(frame->getMsgType() == GW_HANDOFF_MESSAGE){
        send(msg,"neighbourTalkerOut");
    }else{
        cPacket *clone = msg->dup();
        if (msg->getControlInfo())
            clone->setControlInfo((msg->getControlInfo())->dup());
        send(msg,"neighbourTalkerOut");
        send(clone,"packetForwarderOut");
    }
    
}

void PriorityHandlingAndScheduling::initialize(int stage)
{
    if (stage == 0) {
        sendMessageFromQueue0 = new cMessage("Send message from band 0");
        freeAfterCurrent[0] = 0;
        lastPriority[0] = 99999;

        sendMessageFromQueue1 = new cMessage("Send message from band 1");
        freeAfterCurrent[1] = 0;
        lastPriority[1] = 99999;
    }else if (stage == INITSTAGE_APPLICATION_LAYER) {

    }
}

void PriorityHandlingAndScheduling::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        handleLowerPacket(PK(msg));
    }else if(msg->arrivedOn("packetForwarderIn")){
        bool scheduled = handleUpperPacket(PK(msg));
        if(!scheduled) send(msg,"neighbourTalkerOut");
    }else if(msg->arrivedOn("neighbourTalkerIn")){
        bool scheduled = handleUpperPacket(PK(msg));
        if(!scheduled) delete msg;
    }else if(msg->isSelfMessage()){
        if((msg == sendMessageFromQueue0) || (msg == sendMessageFromQueue1)){
            int band = 0;
            if((msg == sendMessageFromQueue0)) band = 0;
            if((msg == sendMessageFromQueue1)) band = 1;
            for(auto it=sendingQueue[band].begin();it!=sendingQueue[band].end();it++){
                if(simTime() == (*it).sendingTime){
                    LoRaMacFrame* frame = (*it).frame;
                    send(frame,"lowerLayerOut");
                    sendingQueue[band].erase(it++);
                    break;
                }
            }
            if(!sendingQueue[band].empty()) {
                auto elementinQueue = sendingQueue[band].front();
                LoRaMacFrame* frame = (elementinQueue).frame;
                int PayloadLength = frame->getPayloadLength();
                if(PayloadLength == 0)
                    PayloadLength = 20;
                double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
                freeAfterCurrent[band] = elementinQueue.sendingTime + (delta * 10);
                lastPriority[band] = frame->getType();
                scheduleAt((elementinQueue.sendingTime),msg);
            }
        }
    }else delete msg;
}

bool PriorityHandlingAndScheduling::canThisBeScehduled(int band, int priority, simtime_t sendingTime)
{
    // If a new message can be scheduled of if the new
    // one can has a higher priority than last scheduled messages
    // and is newer than the last transmitted message
    if(sendingTime < simTime()) return false;
    if(sendingTime >= freeAfterCurrent[band])
        return true;
    else{
        if(band == 0){
            if((priority < lastPriority[band]) && (simTime() < sendMessageFromQueue0->getArrivalTime()))
                return true;
        }else if(band == 1){
            if((priority < lastPriority[band]) && (simTime() < sendMessageFromQueue1->getArrivalTime()))
                return true;
        }
    }
    return false;
}

}