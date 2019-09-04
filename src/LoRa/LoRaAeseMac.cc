//
// Copyright (C) 2016 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/common/ModuleAccess.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/UserPriority.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"

#include <LoRaAeseMac.h>

namespace inet {

Define_Module(LoRaAeseMac);

LoRaAeseMac::~LoRaAeseMac()
{
    cancelAndDelete(sendTRRequest);
    cancelAndDelete(collisionResoQueue);
    cancelAndDelete(dataQueue);
    cancelAndDelete(msgSendToMiniSlotTimer);
    cancelAndDelete(sendDataRequest);
    cancelAndDelete(endTransmission);
    cancelAndDelete(dataSuccess);
    cancelAndDelete(endReception);
    cancelAndDelete(droppedPacket);
    cancelAndDelete(endDelay_1);
    cancelAndDelete(endListening_1);
    cancelAndDelete(endDelay_2);
    cancelAndDelete(endListening_2);
    cancelAndDelete(mediumStateChange);
}

/****************************************************************
 * Initialization functions.
 */
void LoRaAeseMac::initialize(int stage)
{
    MACProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage 0\n";

        maxQueueSize = par("maxQueueSize");
        headerLength = par("headerLength");
        ackLength = par("ackLength");
        ackTimeout = par("ackTimeout");
        retryLimit = par("retryLimit");

        noOfMslots = par("noOfMslots");
        noOfNslots = par("noOfNslots");
        mSlotDuration = par("mSlotDuration"); //s
        nSlotDuration = par("nSlotDuration"); //s
        frameDuration = (noOfNslots*nSlotDuration) + (noOfMslots*mSlotDuration);
        numberOfChannels = par("numberOfChannels");
        dataOnSameChannel = par("dataOnSameChannel");

        waitDelay1Time = 1;
        listening1Time = 1;
        waitDelay2Time = 1;
        listening2Time = 1;

        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = DevAddr::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);
        registerInterface();

        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(LoRaRadio::droppedPacket, this);
        radio = check_and_cast<IRadio *>(radioModule);

        // initialize self messages
        endTransmission = new cMessage("Transmission");
        endReception = new cMessage("Reception");
        droppedPacket = new cMessage("Dropped Packet");
        endDelay_1 = new cMessage("Delay_1");
        endListening_1 = new cMessage("Listening_1");
        endDelay_2 = new cMessage("Delay_2");
        endListening_2 = new cMessage("Listening_2");
        mediumStateChange = new cMessage("MediumStateChange");

        sendTRRequest = new cMessage("SlotRequstMessage");
        sendDataRequest = new cMessage("DataMessageRequest");
        collisionResoQueue = new cMessage("CollisionResoQueue");
        dataQueue = new cMessage("DataQueue");
        msgSendToMiniSlotTimer = new cMessage("MssageSendToMiniSlotTimer");
        dataSuccess = new cMessage("SuccessfulDataTransmission");

        // set up internal queue
        transmissionQueue.setName("transmissionQueue");
        timeFromUpperLayer.setName("Upper Layer Received");
        timeDataPacketSent.setName("Data Packet Sent");
        endToEndNode.setName("EndToEndDelayAtNode");
        fsmVector.setName("LoRaMac State Machine");
        droppedPacketsVector.setName("LoRaMac State when packet dropped");

        // state variables
        fsm.setName("LoRaMac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;

        // sequence number for messages
        sequenceNumber = 0;

        // statistics
        numRetry = 0;
        numSentWithoutRetry = 0;
        numGivenUp = 0;
        numCollision = 0;
        numSent = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;
        sendDataWithMini = false;
        miniSlotNumber = 0;
        dataSlotNumber = 0;
        droppedPackets = 0;
        sendDataPackets = 0;
        dataSentInThisFrame = false;

        // initialize watches
        WATCH(fsm);
        WATCH(backoffPeriod);
        WATCH(retryCounter);
        WATCH(numRetry);
        WATCH(numSentWithoutRetry);
        WATCH(numGivenUp);
        WATCH(numCollision);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numSentBroadcast);
        WATCH(numReceivedBroadcast);
        WATCH(droppedPackets);
        WATCH(sendDataPackets);
    }
    else if (stage == INITSTAGE_LINK_LAYER){

    }
}

void LoRaAeseMac::finish()
{
    recordScalar("numRetry", numRetry);
    recordScalar("numSentWithoutRetry", numSentWithoutRetry);
    recordScalar("numGivenUp", numGivenUp);
    //recordScalar("numCollision", numCollision);
    recordScalar("numSent", numSent);
    recordScalar("numReceived", numReceived);
    recordScalar("numSentBroadcast", numSentBroadcast);
    recordScalar("numReceivedBroadcast", numReceivedBroadcast);
    recordScalar("droppedPackets", droppedPackets);
    recordScalar("sendDataPackets", sendDataPackets);
}

InterfaceEntry *LoRaAeseMac::createInterfaceEntry()
{
    InterfaceEntry *e = new InterfaceEntry(this);

    // data rate
    e->setDatarate(bitrate);

    // capabilities
    e->setMtu(par("mtu"));
    e->setMulticast(true);
    e->setBroadcast(true);
    e->setPointToPoint(false);

    return e;
}

/****************************************************************
 * Message handling functions.
 */
void LoRaAeseMac::handleSelfMessage(cMessage *msg)
{
    EV << "received self message: " << msg << endl;
    handleWithFsm(msg);
}

void LoRaAeseMac::handleUpperPacket(cPacket *msg)
{
    // std::cout << "Handle upper packet " << fsm.getState() << std::endl;
    timeFromUpperLayer.record(simTime());
    if(fsm.getState() == MSGTOSEND || fsm.getState() == MINISLOTTIMER || fsm.getState() == MINISLOTSENDING || fsm.getState() == MINISLOTREQ || fsm.getState() == DATAQUEUE || fsm.getState() == COLLISIONQUEUE)
    {
        // droppedPacketsVector.record(fsm.getState());
        // LoRaMacControlInfo *cInfo = check_and_cast<LoRaMacControlInfo *>(msg->getControlInfo());
        // if(cInfo->getNumberOfFrames() == numberOfFrames){
        //     delete transmissionQueue.pop();
            
        //     // receivedValue = simTime();
        //     LoRaMacFrame *frame = encapsulate(msg);
        //     frame->setLoRaTP(cInfo->getLoRaTP());
        //     frame->setLoRaCF(cInfo->getLoRaCF());
        //     frame->setLoRaSF(cInfo->getLoRaSF());
        //     frame->setLoRaBW(cInfo->getLoRaBW());
        //     frame->setLoRaCR(cInfo->getLoRaCR());
        //     frame->setSequenceNumber(sequenceNumber-1);
        //     frame->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
        //     frame->setLoRaUseHeader(cInfo->getLoRaUseHeader());
        //     frame->setPayloadLength(cInfo->getPayloadLength());
        //     frame->setNumberOfFrames(cInfo->getNumberOfFrames());

        //     numberOfFrames = cInfo->getNumberOfFrames();
        //     EV << "frame " << frame << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;
        //     transmissionQueue.insert(frame);
        // }else{
            droppedPackets++;
            delete msg;
        // }
    }else if(fsm.getState() == IDLE){
        receivedValue = simTime();
        
        LoRaMacControlInfo *cInfo = check_and_cast<LoRaMacControlInfo *>(msg->getControlInfo());
        LoRaMacFrame *frame = encapsulate(msg);
        frame->setLoRaTP(cInfo->getLoRaTP());
        frame->setLoRaCF(cInfo->getLoRaCF());
        frame->setLoRaSF(cInfo->getLoRaSF());
        frame->setLoRaBW(cInfo->getLoRaBW());
        frame->setLoRaCR(cInfo->getLoRaCR());
        frame->setSequenceNumber(sequenceNumber);
        frame->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
        ++sequenceNumber;
        frame->setLoRaUseHeader(cInfo->getLoRaUseHeader());
        frame->setPayloadLength(cInfo->getPayloadLength());
        frame->setNumberOfFrames(cInfo->getNumberOfFrames());

        numberOfFrames = cInfo->getNumberOfFrames();
        EV << "frame " << frame << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;
        transmissionQueue.insert(frame);
        handleWithFsm(frame);
    }else{
        droppedPackets++;
        delete msg;
    }
}

void LoRaAeseMac::handleLowerPacket(cPacket *msg)
{   
    turnOnReceiver();
    if(!isSensor){
        parseActuationMessage(msg);
    }else if(isSensor){
        if( (fsm.getState() == IDLE)) delete msg;
        else if(fsm.getState() == MSGTOSEND || fsm.getState() == MINISLOTREQ || fsm.getState() == DATAQUEUE || fsm.getState() == COLLISIONQUEUE){parseFeedbackMessage(msg);delete msg;}
        else {handleWithFsm(msg);delete msg;}
    }
}

void LoRaAeseMac::handleWithFsm(cMessage *msg)
{
    // std::cout << address << " State when entering:" << fsm.getStateName() << std::endl;
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(msg);
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Idle-MsgToSend,
                                  isUpperMessage(msg),
                                  MSGTOSEND,turnOnAndSwitch();
            );
        }
        FSMA_State(MSGTOSEND)
        {
            FSMA_Enter(turnOnAndSwitch());
            FSMA_Event_Transition(MsgToSend-MiniSlotTimer,
                                  msg == msgSendToMiniSlotTimer,
                                  MINISLOTTIMER,
                {scheduleMinislot();if(sendDataWithMini)scheduleDataFrame();}
                numSent++;
            );
        }
        FSMA_State(MINISLOTTIMER)
        {
            FSMA_Event_Transition(MiniSlotTimer-MiniSlotSending,
                                  msg == sendTRRequest,
                                  MINISLOTSENDING,sendMinislot()
            );
        }
        FSMA_State(MINISLOTSENDING)
        {
            FSMA_Event_Transition(MiniSlotSending-MiniSlotReq,
                                    msg == endTransmission,
                                    MINISLOTREQ,
            );
        }
        FSMA_State(MINISLOTREQ)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(MiniSlotReq-DataMessage,
                                  msg == sendDataRequest && sendDataWithMini,
                                  TRANMSITTING,
                                  sendDataFrame(getCurrentTransmission());
            );
            FSMA_Event_Transition(Transmit-Idle,
                                  msg == dataSuccess,
                                  IDLE,finishCurrentTransmission();
            );
            if(dataOnSameChannel){
                FSMA_Event_Transition(MiniSlotReq-DataQueue,
                                      msg == dataQueue,
                                      DATAQUEUE,
                );
            }else{
                FSMA_Event_Transition(MiniSlotReq-DataSlot,
                                      msg == dataQueue,
                                      DATASLOT,{turnOffReceiver();scheduleDataFrame();}
                );
            }
            FSMA_Event_Transition(MiniSlotReq-CollisionResoQueue,
                                  msg == collisionResoQueue,
                                  COLLISIONQUEUE,
            );
        }
        FSMA_State(DATAQUEUE)
        {
            FSMA_Enter(checkIfDataSlot(msg));
            FSMA_Event_Transition(DataQueue-DataSlot,
                                  pDQ < noOfNslots,
                                  DATASLOT,scheduleDataFrame();
            );
        }
        FSMA_State(DATASLOT)
        {
            FSMA_Event_Transition(DataSlot-Transmit,
                                  msg == sendDataRequest,
                                  TRANMSITTING,sendDataFrame(getCurrentTransmission());
            );
        }
        FSMA_State(TRANMSITTING)
        {
            // FSMA_Enter(logTimeSent());
            FSMA_Event_Transition(Transmit-Idle,
                                  msg == endTransmission && sendDataWithMini,
                                  MINISLOTREQ,{sendDataWithMini=false;}
            );
            FSMA_Event_Transition(Transmit-Idle,
                                  msg == endTransmission && !sendDataWithMini,
                                  IDLE,
                                  finishCurrentTransmission();
            );
        }
        FSMA_State(COLLISIONQUEUE)
        {
            FSMA_Enter(checkIfZero(msg));
            FSMA_Event_Transition(CollisionQueue-MiniSlotTimer,
                                  pCQ == 0,
                                  MINISLOTTIMER,scheduleMinislot()
            );   
        }
    }
    // fsmVector.record(fsm.getState());
    // std::cout << address << " State when exiting:" << fsm.getStateName() << std::endl;
}

void LoRaAeseMac::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::receptionStateChangedSignal) {
        IRadio::ReceptionState newRadioReceptionState = (IRadio::ReceptionState)value;
        if (receptionState == IRadio::RECEPTION_STATE_RECEIVING) {
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        }
        receptionState = newRadioReceptionState;
        // handleWithFsm(mediumStateChange);
    }
    else if (signalID == LoRaRadio::droppedPacket) {
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        // handleWithFsm(droppedPacket);
        turnOnReceiver();
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
            handleWithFsm(endTransmission);
        }
        transmissionState = newRadioTransmissionState;
    }
}

LoRaMacFrame *LoRaAeseMac::encapsulate(cPacket *msg)
{
    LoRaMacFrame *frame = new LoRaMacFrame(msg->getName());

    frame->setByteLength(headerLength);
    frame->setArrival(msg->getArrivalModuleId(), msg->getArrivalGateId());

    frame->setTransmitterAddress(address);

    frame->encapsulate(msg);

    return frame;
}

cPacket *LoRaAeseMac::decapsulate(LoRaMacFrame *frame)
{
    cPacket *payload = frame->decapsulate();

    delete frame;
    return payload;
}

/****************************************************************
 * Frame sender functions.
 */
void LoRaAeseMac::sendDataFrame(LoRaMacFrame *frameToSend)
{
    EV << "sending Data frame\n";
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    LoRaMacFrame *frameCopy = frameToSend->dup();
    frameCopy->setMsgType(UPLINKDATASLOT);
    frameCopy->setMyMiniSlot(miniSlotNumber);
    frameCopy->setMyDataSlot(dataSlotNumber);
    frameCopy->setLoRaCF(transmissionFrequency);

    LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
    ctrl->setSrc(frameCopy->getTransmitterAddress());
    ctrl->setDest(frameCopy->getReceiverAddress());
    ctrl->setLoRaCF(transmissionFrequency);

    frameCopy->setControlInfo(ctrl);
    if(sendDataWithMini) frameCopy->setDataInThisFrame(true);
    else frameCopy->setDataInThisFrame(false);
 
    dataSentInThisFrame = true;   
    sendDataPackets++;   
    // std::cout << address << ":Sending data frame at" << dataSlotNumber << std::endl;
    sendDown(frameCopy);
}

void LoRaAeseMac::sendAckFrame()
{
    EV << "sending Ack frame\n";
    auto ackFrame = new CsmaCaMacAckFrame("CsmaAck");
    ackFrame->setByteLength(ackLength);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    sendDown(ackFrame);
}

/****************************************************************
 * Helper functions.
 */
void LoRaAeseMac::finishCurrentTransmission()
{
    // scheduleAt(simTime() + waitDelay1Time, endDelay_1);
    // scheduleAt(simTime() + waitDelay1Time + listening1Time, endListening_1);
    // scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time, endDelay_2);
    // scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time + listening2Time, endListening_2);
    numberOfFrames = 0;
    dataSlotNumber = 0;
    miniSlotNumber = 0;
    popTransmissionQueue();
    sendDataWithMini = false;
    dataSentInThisFrame = false;
    logTimeSent();
}

LoRaMacFrame *LoRaAeseMac::getCurrentTransmission()
{
    return static_cast<LoRaMacFrame*>(transmissionQueue.front());
}

void LoRaAeseMac::popTransmissionQueue()
{
    EV << "dropping frame from transmission queue\n";
    delete transmissionQueue.pop();
    if (queueModule) {
        // tell queue module that we've become idle
        EV << "requesting another frame from queue module\n";
        queueModule->requestPacket();
    }
}

bool LoRaAeseMac::isReceiving()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_RECEIVING;
}

bool LoRaAeseMac::isAck(LoRaMacFrame *frame)
{
    return dynamic_cast<LoRaMacFrame *>(frame);
}

bool LoRaAeseMac::isFeedback(LoRaMacFrame *frame)
{
    if(frame->getMsgType() == FEEDBACK)
        return true;
    return false;
}

bool LoRaAeseMac::isActuation(LoRaMacFrame *frame)
{
    if(frame->getMsgType() == ACTUATION)
        return true;
    return false;
}

bool LoRaAeseMac::isBroadcast(LoRaMacFrame *frame)
{
    return frame->getReceiverAddress().isBroadcast();
}

bool LoRaAeseMac::isForUs(LoRaMacFrame *frame)
{
    return frame->getReceiverAddress() == address;
}

void LoRaAeseMac::turnOnAndSwitch()
{
    turnOnReceiver();
    setCarrierFrequency(initCarrierFrequency);
}

void LoRaAeseMac::turnOnReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
}

void LoRaAeseMac::turnOffReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

DevAddr LoRaAeseMac::getAddress()
{
    return address;
}

void LoRaAeseMac::scheduleMinislot()
{
    miniSlotNumber = intuniform(0,(noOfMslots-1));
    // std::cout << "Scheduling at minislot: " << miniSlotNumber << std::endl;
    EV << "Scheduling at minislot: " << miniSlotNumber << "\n";
    scheduleAt(simTime() + (miniSlotNumber * mSlotDuration) + uniform(0.0001,0.005),sendTRRequest);
}

void LoRaAeseMac::sendMinislot()
{
    EV << "sending Minislot frame\n";
    // std::cout << address << ":Sending minislot frame at" << miniSlotNumber << std::endl;

    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    LoRaMacFrame *frameToSend = new LoRaMacFrame("MiniSlotFrame");
    // frameToSend->encapsulate(request);
    frameToSend->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
    frameToSend->setNumberOfFrames(numberOfFrames);

    frameToSend->setLoRaTP(getCurrentTransmission()->getLoRaTP());
    frameToSend->setLoRaCF(getCurrentTransmission()->getLoRaCF());
    frameToSend->setLoRaSF(getCurrentTransmission()->getLoRaSF());
    frameToSend->setLoRaBW(getCurrentTransmission()->getLoRaBW());
    frameToSend->setLoRaCR(getCurrentTransmission()->getLoRaCR());
    frameToSend->setPayloadLength(5);

    frameToSend->setMsgType(UPLINKMINISLOT);
    frameToSend->setMyMiniSlot(miniSlotNumber);
    if(sendDataWithMini) frameToSend->setDataInThisFrame(true);
    else frameToSend->setDataInThisFrame(false);

    sendDown(frameToSend);
}

void LoRaAeseMac::parseFeedbackMessage(cPacket *packet)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame*>(packet);
    if(isFeedback(frame)){
        if(fsm.getState() == COLLISIONQUEUE){
            // std::cout << "Feedback processing in CollisionQueue" << pCQ << std::endl;
            pCQ = pCQ - 1;
            checkIfZero(collisionResoQueue);
            // handleWithFsm(collisionResoQueue);
        }else if(fsm.getState() == DATAQUEUE){
            // std::cout << "Feedback processing in DataQueue" << pDQ << std::endl;
            // pDQ = pDQ - 1;
            // checkIfZero(dataQueue);
            if(dataOnSameChannel){
                checkIfDataSlot(dataQueue);
            }
            // handleWithFsm(dataQueue);
        }else if(fsm.getState() == MINISLOTREQ){
            if(dataOnSameChannel){
                int responseMini = frame->getM(miniSlotNumber);
                int responseData = frame->getN(dataSlotNumber);
                int CRQ = frame->getCRQ();
                int DTQ = frame->getDTQ();
                // std::cout << "Feedback processing in Minislotreq" << responseMini << DTQ << CRQ << std::endl;
                pCQ = CRQ;
                pDQ = DTQ;
                if(responseData == SUCCESS && dataSentInThisFrame){
                    dataSentInThisFrame = false;
                    handleWithFsm(dataSuccess);
                }else{
                    dataSentInThisFrame = false;
                    for(int i = (noOfMslots-1);i > (miniSlotNumber-1);i--){
                        if(frame->getM(i) == COLLISION){
                            pCQ = pCQ - 1;
                        }
                        else if(frame->getM(i) == SUCCESS){
                            pDQ = pDQ - 1;
                        }
                    }
                    if(responseMini == COLLISION){
                        // std::cout << "Handling Collison " << pCQ << std::endl;
                        handleWithFsm(collisionResoQueue);
                    }
                    else if(responseMini == SUCCESS){
                        // std::cout << "Handling Data " << pDQ << std::endl;
                        handleWithFsm(dataQueue);
                    }
                }
            }else{
                int responseMini = frame->getM(miniSlotNumber);
                int CRQ = frame->getCRQ();
                
                pCQ = CRQ;
                dataSentInThisFrame = false;
                if(responseMini == COLLISION){
                    for(int i = (noOfMslots-1); i > (miniSlotNumber-1); i--){
                        if(frame->getM(i) == COLLISION){
                            pCQ = pCQ - 1;
                        }
                    }
                    // std::cout << "Handling Collison " << pCQ << std::endl;
                    handleWithFsm(collisionResoQueue);
                }
                else if(responseMini == SUCCESS){
                    // std::cout << address <<":Handling Data " << frame->getChannelNumber(miniSlotNumber) << " " << frame->getPosition(miniSlotNumber) << std::endl;
                    transmissionFrequency = inet::units::values::Hz(frame->getChannelNumber(miniSlotNumber) * 200000 + 867100000);
                    dataSlotNumber = frame->getPosition(miniSlotNumber);
                    handleWithFsm(dataQueue);
                }
            }
            // std::cout << address <<":Feedback processing in Minislotreq" << responseMini << pDQ << " " << pCQ << std::endl;
        }else if(fsm.getState() == MSGTOSEND){
            // if( frame->getDTQ() < noOfNslots ){
            //     dataSlotNumber = intuniform(frame->getDTQ(),(noOfNslots-1));
            //     sendDataWithMini = true;
            // }
            handleWithFsm(msgSendToMiniSlotTimer);
        }
    }
}

void LoRaAeseMac::scheduleDataFrame()
{
    // dataSlotNumber = intuniform(0,(noOfNslots-1));
    if(dataOnSameChannel){
        scheduleAt(simTime()+(uniform(0.0001,0.005))+(noOfMslots * mSlotDuration)+(dataSlotNumber * nSlotDuration), sendDataRequest);
    }else{
        scheduleAt(simTime()+(uniform(0.023,0.028))+(dataSlotNumber * nSlotDuration), sendDataRequest);
    }
}

void LoRaAeseMac::checkIfZero(cMessage *msg)
{
    if((pDQ == 0 && msg == dataQueue) || (pCQ == 0 && msg == collisionResoQueue)) handleWithFsm(msg);
    else turnOnReceiver();
}

void LoRaAeseMac::logTimeSent()
{
    //timeDataPacketSent.record(simTime());
    endToEndNode.record(simTime()-receivedValue);
    // std::cout << simTime()-receivedValue << std::endl;
}

void LoRaAeseMac::checkIfDataSlot(cMessage *msg)
{
    if(pDQ > (noOfNslots-1)){
        pDQ = pDQ - noOfNslots;
    }else{
        dataSlotNumber = pDQ;
        handleWithFsm(msg);
    }
}

void LoRaAeseMac::setCarrierFrequency(Hz carrierFrequency)
{
    initCarrierFrequency = carrierFrequency;
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
    loraRadio->setCarrierFrequency(carrierFrequency);    
}

void LoRaAeseMac::setNodeType(NodeType nd)
{
    node = nd;
}

void LoRaAeseMac::setSensor(bool sensor)
{
    isSensor = sensor;
}

void LoRaAeseMac::parseActuationMessage(cPacket *packet)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame*>(packet);
    if(isActuation(frame)){
        sendUp(decapsulate(frame));
    }else{
        delete packet;
    }
}

std::string LoRaAeseMac::str() const
{
    return "LoRaAeseMac";
}

} // namespace inet
