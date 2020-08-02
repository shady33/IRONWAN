#include "PriorityHandlingAndScheduling.h"

namespace inet {

Define_Module(PriorityHandlingAndScheduling);

PriorityHandlingAndScheduling::~PriorityHandlingAndScheduling()
{
    cancelAndDelete(sendMessageFromQueue0);
    cancelAndDelete(sendMessageFromQueue1);
}

void PriorityHandlingAndScheduling::finish()
{

}

bool PriorityHandlingAndScheduling::handleUpperPacket(cPacket *msg)
{
    bool scheduled = false;
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
    
    if(sendingTime < simTime()) return scheduled;
    // Check if the sending time is after the free time
    if((sendingTime >= freeAfterCurrent[band])){
        scheduled = true;
    }else{
        // Check if new packet is more important than last scheduled and last scheduled has not been transmitted
        if((frame->getType() < lastPriority[band]) && (simTime() < sendTimer->getArrivalTime())){
            scheduled = true;
        }
    }
    if(scheduled){
        // If a packet can be scheduled, push to queue
        // Check if a packet is scheduled
        // If it is scheduled, is its arrival time after my sending time,
        // If yes, delete and add mine
        // If no packet is scheduled, schedule a new one
        sendingQueue[band].emplace_back(sendingTime,sendingTime+(delta*waitTime),(MessagesTypesAndPriority)frame->getType(),frame->getReceiverAddress(),frame);
        if(sendTimer->isScheduled()){
            if(sendTimer->getArrivalTime() > sendingTime){
                cancelEvent(sendTimer);
                scheduleAt(sendingTime,sendTimer);
                lastPriority[band] = frame->getType();
                freeAfterCurrent[band] = sendingTime + (delta * waitTime);
            }
        }else{
            scheduleAt(sendingTime,sendTimer);
            lastPriority[band] = frame->getType();
            freeAfterCurrent[band] = sendingTime + (delta * waitTime);
        }
    }
    return scheduled;
}

void PriorityHandlingAndScheduling::handleLowerPacket(cPacket *msg)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(msg);
    if((frame->getMsgType() == GW_HANDOFF_MESSAGE)){
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
        AeseGWMode = (int)par("AeseGWMode");
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
                LoRaMacFrame* frame;
                simtime_t lastTime = SIMTIME_MAX;
                for(auto it : sendingQueue[band]){
                    if(((it).sendingTime > simTime()) && ((it).sendingTime < lastTime)){
                        frame = (it).frame;
                        lastTime = (it).sendingTime;
                    }
                }
                // auto elementinQueue = sendingQueue[band].front();
                // LoRaMacFrame* frame = (elementinQueue).frame;
                int PayloadLength = frame->getPayloadLength();
                if(PayloadLength == 0)
                    PayloadLength = 20;
                double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
                freeAfterCurrent[band] = lastTime + (delta * 10);
                lastPriority[band] = frame->getType();
                scheduleAt(lastTime,msg);
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
