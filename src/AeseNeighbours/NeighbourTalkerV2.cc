#include "NeighbourTalkerV2.h"

namespace inet {

Define_Module(NeighbourTalkerV2);

NeighbourTalkerV2::~NeighbourTalkerV2()
{
    for (auto iter = ReceivedPacketsList->begin(); iter != ReceivedPacketsList->end(); ) {
        auto cur=iter++;
        delete cur->second.frame;
        ReceivedPacketsList->erase(cur);
    }
}

void NeighbourTalkerV2::initialize(int stage)
{
    if (stage == 0) {
        AeseGWMode = (AeseGWModes)(int)par("AeseGWMode");
        currentProtocol = LORA;
    }else if (stage == INITSTAGE_APPLICATION_LAYER){
        ReceivedPacketsList = new ReceivedPacketsMap();
        DownlinkLastDropRequestList = new DownlinkLastDropRequest();
        NeighbourDropRequestList = new DownlinkLastDropRequest();

        scheduler = check_and_cast<PriorityHandlingAndScheduling *>(getParentModule()->getSubmodule("scheduler"));
        rl = check_and_cast<ReinforcementLearning *>(getParentModule()->getSubmodule("reinforcementLearning"));
        failedBids = 0;
        requestedBids = 0;
        acceptedBids = 0;
        transmittedSomeoneDownlink = 0;
        transmittedPeriodIn = 0;
        rebroadcastingAnUplink = 0;
        requestedSomeoneToForwardAck = 0;
        startUDP();
    }
}

void NeighbourTalkerV2::startUDP()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), 1007);
}

void NeighbourTalkerV2::handleMessage(cMessage *msg)
{
    // if (AeseGWMode == NO_NEIGHBOUR){
    //     delete msg;
    //     return;
    // }

    if (msg->arrivedOn("lowerLayerIn")) {
        EV << "Received message from Lower Layer" << endl;
        // LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(msg);
        handleLowerLayer(PK(msg));
    }else if(msg->arrivedOn("periodIn")){
        if(AeseGWMode > NO_NEIGHBOUR){
            handlePeriodIn(PK(msg));
        }
    }else if(msg->arrivedOn("udpIn")){
        if(AeseGWMode > NO_NEIGHBOUR){
            send(PK(msg),"periodOut");
        }else{
            delete msg;
        }
    }

    // else if(msg->arrivedOn("udpIn")){
    //     EV << "Received Request from neighbour" << endl;
    //     NeighbourTalkerMessage *request = check_and_cast<NeighbourTalkerMessage*>(PK(msg));
    //     UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(msg->getControlInfo());
    //     if(request->getMsgType() == REQUEST_HANDOFF)
    //         canIHandleThisMessage(PK(msg));
    //     else if(request->getMsgType() == CONFIRM_HANDOFF){
    //         collectBids(cInfo->getSrcAddr(),request->getDeviceAddress(),request->getRSSI());
    //         delete msg;
    //     }else if(request->getMsgType() == ACCEPT_HANDOFF){
    //         auto iter = NeighbourDropRequestList->find(request->getDeviceAddress());
    //         if(iter != NeighbourDropRequestList->end()) NeighbourDropRequestList->erase(iter);
    //         delete msg;
    //     }
    //     return;
    // }

    // if(msg->isSelfMessage()){
    //     if(msg == checkAnyUnsentMessages){
    //         EV << "Time to transmit a ping message" << endl;
    //         handleDownlinkQueue();
    //         scheduleAt(simTime() + 0.1, checkAnyUnsentMessages);
    //     }else if(!strcmp(msg->getName(),"DecideWhichNodeBidToAccept")){
    //         acceptBid(check_and_cast<DevAddrMessage*>(msg)->getAddr());
    //         delete msg;
    //     }else if(!strcmp(msg->getName(),"HaveIReceivedBidConfirmation")){
    //         checkConfirmationAndDelete(check_and_cast<DevAddrMessage*>(msg)->getAddr());
    //         delete msg;
    //     }
    // }
}

void NeighbourTalkerV2::finish()
{
    recordScalar("TransmittedSomeoneDownlink",transmittedSomeoneDownlink);
    recordScalar("TransmittedPeriodIn",transmittedPeriodIn);
    recordScalar("RebroadcastingAnUplink",rebroadcastingAnUplink);
    recordScalar("RequestedSomeoneToForwardAck",requestedSomeoneToForwardAck);

    recordScalar("RequestedBids",requestedBids);
    recordScalar("AcceptedBids",acceptedBids);
    recordScalar("FailedBids",failedBids);
    for(auto& elem: downlinkList){
        delete elem.frame;
    }
}

void NeighbourTalkerV2::handleLowerLayer(cPacket* pkt)
{
    std::string className(pkt->getClassName());
    if(className.compare("inet::LoRaMacFrame") == 0)
        handleLoRaFrame(pkt);
    else
        delete pkt;
}

void NeighbourTalkerV2::forwardToOthers(LoRaMacFrame *frame)
{
    send(frame->dup(),"rlLayerOut");
    send(frame->dup(),"periodOut");
}

void NeighbourTalkerV2::handleLoRaFrame(cPacket *pkt)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(pkt);
    if(frame->getType() == MY_ACKS){
        // need to schedule my failed acks
        //handleFailedAcks(frame);
    	if (AeseGWMode == NO_NEIGHBOUR){
	    	delete frame;
	    }else if(AeseGWMode > NO_NEIGHBOUR){
		    handleFailedAcks(frame);
	    }
    }else if(frame->getType() == FIND_NEIGHBOURS_FOR_UPLINK){
        // std::cout << "Received FIND_NEIGHBOURS_FOR_UPLINK" << std::endl;
        handleFindNeighboursForUplink(frame->decapsulate());
        delete frame;
    }else if(frame->getType() == SEND_ACCEPT_BIDS_FOR_NEIGHBOURS){
        delete frame;
    }else{
        // Message from phy layer
        if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS){
            // LAKSH: This is an uplink message from nodes for LoRa gateways
            // or its a request from another gateway
            if(frame->getMsgType() == JOIN_REQUEST){
                forwardToOthers(frame);
                physicallayer::ReceptionIndication *cInfo = check_and_cast<physicallayer::ReceptionIndication *>(pkt->getControlInfo());
                W w_rssi = cInfo->getMinRSSI();
                double rssi = w_rssi.get()*1000;
                // This gets added to received packets table
                DevAddr txAddr = frame->getTransmitterAddress();
                auto iter = ReceivedPacketsList->find(txAddr);
                if(iter == ReceivedPacketsList->end()){
                    // Not found in table, insert
                    (*ReceivedPacketsList)[txAddr] = ReceivedPacket(LORA,frame->getSequenceNumber(),frame,simTime(),rssi);
                }else{
                    // Already in table, delete old and add new
                    ReceivedPacket& rxPkt = iter->second;
                    rxPkt.insertionTime = simTime();
                    delete rxPkt.frame;
                    rxPkt.frame = frame;
                    rxPkt.lastSeqNo = frame->getSequenceNumber();
                    rxPkt.RSSI = rssi;
                }
            }else if(frame->getMsgType() == GW_HANDOFF_MESSAGE){
                LoRaMacFrame *msg = dynamic_cast<LoRaMacFrame*>(frame->decapsulate());
                msg->setType(ACCEPTED_BIDS_ACKS);

                // This is a message for gateways interested in someone to accept their bids
                DevAddr txAddr = msg->getReceiverAddress();
                auto iter = ReceivedPacketsList->find(txAddr);
                if(iter != ReceivedPacketsList->end()){
                    ReceivedPacket& rxPkt = iter->second;
                    if(((simTime() - rxPkt.insertionTime) < 2) && (scheduler->canThisBeScehduled(1,ACCEPTED_BIDS_ACKS,msg->getSendingTime()))){
                        // Decapsulate and add the correct type
                        if((AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM)){
                            if (intuniform(0,10) < 5){
                                transmittedSomeoneDownlink = transmittedSomeoneDownlink + 1;
                                send(msg->dup(),"lowerLayerOut");
                            }
                        }else{
                            transmittedSomeoneDownlink = transmittedSomeoneDownlink + 1;
                            send(msg->dup(),"lowerLayerOut");
                        }
                    }
                }
                delete msg;
                delete frame;
            }
        }else{
            // LAKSH: These are packets destined especially for me,
            // they would be gateways making bids
            // TODO: Not there in first version
        }
    }
    // frame->removeControlInfo();
}

// We check if we can transmit a period finder message, append and then send
void NeighbourTalkerV2::handlePeriodIn(cPacket *pkt)
{
    ReinforcementLearning::ActionChosen act = rl->whichSlotDoIUse();
    simtime_t sendingTime = simTime() + ((act.slot-1)*0.1);
    bool scheduled = scheduler->canThisBeScehduled(0,FIND_NEIGHBOURS_FOR_UPLINK,sendingTime);
    if(scheduled){
        LoRaMacFrame *msg = new LoRaMacFrame("FIND_NEIGHBOURS_FOR_UPLINK");
        msg->encapsulate(pkt);
        inet::units::values::Hz bw = inet::units::values::Hz(125000);
        inet::units::values::Hz freq = inet::units::values::Hz((act.channel * 200000) + 868100000);
        msg->setMsgType(GW_HANDOFF_MESSAGE);
        msg->setType(FIND_NEIGHBOURS_FOR_UPLINK);
        msg->setPayloadLength(10);
        msg->setLoRaCF(freq);
        msg->setLoRaSF(7);
        msg->setLoRaBW(bw);
        msg->setLoRaCR(1);
        msg->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
        msg->setSendingTime(sendingTime);
        transmittedPeriodIn = transmittedPeriodIn + 1;
        send(msg,"lowerLayerOut");
    }else delete pkt;
}


// LAKSH: this should check if it can be handled.
// if yes, find slot and channel and then broadcast it
void NeighbourTalkerV2::handleFailedAcks(LoRaMacFrame *frame)
{
    ReinforcementLearning::ActionChosen act = rl->whichSlotDoIUse();
    simtime_t sendingTime = simTime() + ((act.slot-1)*0.1);
    bool scheduled = scheduler->canThisBeScehduled(0,FIND_NEIGHBOURS_FOR_DOWNLINK,sendingTime);
    if(scheduled){
        LoRaMacFrame *msg = new LoRaMacFrame("GW_HANDOFF_MESSAGE");
        msg->setTransmitterAddress(frame->getTransmitterAddress());
        msg->encapsulate(frame);

        inet::units::values::Hz freq = inet::units::values::Hz((act.channel * 200000) + 868100000);
        inet::units::values::Hz bw = inet::units::values::Hz(125000);
        msg->setMsgType(GW_HANDOFF_MESSAGE);
        msg->setLoRaTP(14);
        msg->setLoRaCF(freq);
        msg->setLoRaSF(7);
        msg->setLoRaBW(bw);
        msg->setLoRaCR(1);
        msg->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
        msg->setPayloadLength(frame->getPayloadLength() + 2);
        msg->setSendingTime(sendingTime);
        requestedSomeoneToForwardAck = requestedSomeoneToForwardAck + 1;
        send(msg,"lowerLayerOut");
    }else delete frame;
}

void NeighbourTalkerV2::handleFindNeighboursForUplink(cPacket *pkt)
{
    NeighbourTalkerMessage *frame = check_and_cast<NeighbourTalkerMessage*>(pkt);
    auto iter = ReceivedPacketsList->find(frame->getDeviceAddress());
    // Do I know this node?
    if(iter != ReceivedPacketsList->end()){
        int lastSeenSeqNo  = (iter->second).lastSeqNo;
        // I have a newer packet
        if(lastSeenSeqNo > frame->getSequenceNumber()){
            auto insertionTime = (iter->second).insertionTime;
            // Have I seen the node in last 2 seconds? Or is it a non-confirmed node?
            if((simTime() - insertionTime < 2) || (!frame->getIsConfirmed())){
                ReinforcementLearning::ActionChosen act = rl->whichSlotDoIUse();
                // simtime_t sendingTime = simTime() + ((act.slot-1)*0.1);
                simtime_t sendingTime = simTime() + 0.1;
                bool scheduled = scheduler->canThisBeScehduled(0,SEND_ACCEPT_BIDS_FOR_NEIGHBOURS,sendingTime);
                if(scheduled){
                    auto f = (iter->second).frame;
                    auto frameToSend = f->dup();
                    frameToSend->setType(SEND_ACCEPT_BIDS_FOR_NEIGHBOURS);
                    inet::units::values::Hz freq = inet::units::values::Hz((act.channel * 200000) + 868100000);
                    frameToSend->setLoRaCF(freq);
                    frameToSend->setSendingTime(sendingTime);
                    rebroadcastingAnUplink = rebroadcastingAnUplink + 1;
                    send(frameToSend,"lowerLayerOut");
                }
            }
        }
    }
    delete pkt;
}


// void NeighbourTalkerV2::sendConfirmationToNeighbour(cPacket* pkt)
// {
//     UDPDataIndication *cInfo = check_and_cast<UDPDataIndication*>(pkt->getControlInfo());
//     NeighbourTalkerMessage *request = check_and_cast<NeighbourTalkerMessage*>(pkt);
//     auto iter = ReceivedPacketsList->find(request->getDeviceAddress());

//     NeighbourTalkerMessage *confirm = new NeighbourTalkerMessage("CONFIRM_HANDOFF");
//     confirm->setMsgType(CONFIRM_HANDOFF);
//     confirm->setDeviceAddress(request->getDeviceAddress());
//     confirm->setRSSI((iter->second).RSSI);
//     socket.sendTo(confirm,cInfo->getSrcAddr(),3333);

//     (*NeighbourDropRequestList)[request->getDeviceAddress()] = 1;
//     DevAddrMessage *msg = new DevAddrMessage("HaveIReceivedBidConfirmation");
//     msg->setAddr(request->getDeviceAddress());
//     scheduleAt(simTime()+0.5,msg);
// }

// void NeighbourTalkerV2::collectBids(L3Address gateway, DevAddr addr, double RSSI)
// {
//     // std::cout << getParentModule() << " received bid from " << gateway << " for" << addr << std::endl;
//     bool bidsForGWExists = false;
//     for(uint i=0;i<currentBids.size();i++){
//         if(currentBids[i].addr == addr){
//             currentBids[i].gatewaysThatBid.emplace_back(gateway,RSSI);
//             bidsForGWExists = true;
//         }
//     }
//     if(!bidsForGWExists) {
//         BidQueue bdq;
//         bdq.addr = addr;
//         bdq.gatewaysThatBid.emplace_back(gateway,RSSI);
//         currentBids.push_back(bdq);
//     }
// }

// void NeighbourTalkerV2::acceptBid(DevAddr addr)
// {
//     int idx = -1;
//     double RSSIinGW = -99999999999;
//     L3Address pickedGateway;
//     if(AeseGWMode == NEIGHBOUR_WITH_BIDS){
//         for(int i=0;i<currentBids.size();i++){
//             if(currentBids[i].addr == addr){
//                 for(auto gw: currentBids[i].gatewaysThatBid){
//                     if(RSSIinGW < std::get<1>(gw)) pickedGateway = std::get<0>(gw);
//                 }
//                 idx = i;
//                 break;
//             }
//         }
//     }else if(AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM){
//         for(int i=0;i<currentBids.size();i++){
//             if(currentBids[i].addr == addr){
//                 idx = i;
//                 if(currentBids[i].gatewaysThatBid.size() > 0){
//                     int val = intuniform(0,currentBids[i].gatewaysThatBid.size()-1);
//                     int j=0;
//                     for(auto gw: currentBids[i].gatewaysThatBid){
//                         if(j == val)
//                             pickedGateway = std::get<0>(gw);
//                         j++;
//                     }
//                 }
//                 break;
//             }
//         }
//     }
//     if(idx != -1){
//         // std::cout << getParentModule() << " Accepts bid for " << addr << " from " << pickedGateway << std::endl;
//         NeighbourTalkerMessage *accept = new NeighbourTalkerMessage("ACCEPT_HANDOFF");
//         accept->setMsgType(ACCEPT_HANDOFF);
//         accept->setDeviceAddress(addr);
//         socket.sendTo(accept,pickedGateway,3333);
//         currentBids.erase(currentBids.begin() + idx);
//         acceptedBids++;
//     }else{
//         failedBids++;
//         // std::cout << "No Bids" << std::endl;
//     }
// }

// void NeighbourTalkerV2::checkConfirmationAndDelete(DevAddr addr)
// {
//     auto iter = NeighbourDropRequestList->find(addr);
//     if(iter != NeighbourDropRequestList->end()){
//         // std::cout << getParentModule() << " Deleting " << addr << std::endl;;
//         loRaGwMac->popDevAddr(addr);
//         NeighbourDropRequestList->erase(iter);
//     }
// }

// void NeighbourTalkerV2::handleDownlinkQueue()
// {
//     // for(auto& elem : downlinkList){
//     for(std::list<DownlinkPacket>::iterator it=downlinkList.begin()++; it != downlinkList.end(); ++it){
//         // std::cout << "Added to queue:" << elem.addedToQueue << std::endl;
//         // std::cout << "Dead by time:" << elem.deadByTime << std::endl;
//         // std::cout << "Dev Addr:" << elem.addr << std::endl;
//         // std::cout << "Pkt" << elem.pkt << std::endl;
//         int sequenceNumberInList = -1;
//         auto iter = DownlinkLastDropRequestList->find((*it).addr);
//         if(iter != DownlinkLastDropRequestList->end()){
//             sequenceNumberInList = iter->second;
//         }

//         if((*it).sequenceNumber == sequenceNumberInList){
//             // The message was added to queue and broadcast by the node
//             delete (*it).frame;
//             downlinkList.erase(it++);
//          }else{
//             if(simTime() > (*it).deadByTime){
//                 std::cout << "Past Dead By time, bye bye!" << std::endl;
//                 delete (*it).frame;
//                 downlinkList.erase(it++);
//             }else{
//                 if(simTime() - (*it).addedToQueue > 0.1){
//                     EV << "Time to forward and delete message" << endl;
//                     auto frame = (*it).frame;
//                     requestedBids++;
//                     // std::cout << getParentModule() << " requests handoff" << std::endl;
//                     for(auto gw: gwAddresses){
//                         NeighbourTalkerMessage *request = new NeighbourTalkerMessage("Request_Handoff");
//                         request->encapsulate(frame->dup());
//                         request->setMsgType(REQUEST_HANDOFF);
//                         request->setSendingTime((*it).deadByTime);
//                         request->setDeviceAddress((*it).addr);
//                         socket.sendTo(request,gw,3333);
//                     }
//                     if(AeseGWMode == NEIGHBOUR_WITH_BIDS || AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM){
//                         DevAddrMessage *msg = new DevAddrMessage("DecideWhichNodeBidToAccept");
//                         msg->setAddr((*it).addr);
//                         scheduleAt(simTime() + 0.5, msg);
//                     }
//                     delete (*it).frame;
//                     downlinkList.erase(it++);
//                 }
//             }
//          }
//     }
// }

// void NeighbourTalkerV2::canIHandleThisMessage(cPacket* pkt)
// {
//     bool deletePacket = true;
//     NeighbourTalkerMessage *frame = check_and_cast<NeighbourTalkerMessage*>(pkt);
//     simtime_t freeAfter = loRaGwMac->getTimeForWhenNextMessageIsPossible();
//     if (freeAfter == 0 || freeAfter < frame->getSendingTime()){
//         // This can be scheduled
//         auto iter = ReceivedPacketsList->find(frame->getDeviceAddress());
//         // Do I know this node?
//         if(iter != ReceivedPacketsList->end()){
//             auto insertionTime = (iter->second).insertionTime;
//             // Have I seen the node in last 2 seconds?
//             if(simTime() - insertionTime < 2){
//                 if(AeseGWMode == NEIGHBOUR_WITH_BIDS || AeseGWMode == NEIGHBOUR_WITH_BIDS_RANDOM) {
//                     sendConfirmationToNeighbour(pkt);
//                 }
//                 send(pkt->decapsulate(),"lowerLayerOut");
//                 deletePacket = false;
//             }
//         }
//     }
//     if(deletePacket) {delete pkt->decapsulate();}
//     delete pkt;
// }

} //namespace inet
