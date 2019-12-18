//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "LoRaGWMac.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "../AeseNeighbours/timeOnAir.h"

namespace inet {

Define_Module(LoRaGWMac);

LoRaGWMac::~LoRaGWMac()
{
    cancelAndDelete(dutyCycleTimer);
    cancelAndDelete(sendMessageFromQueue);
}

void LoRaGWMac::initialize(int stage)
{
    MACProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        //radioModule->subscribe(IRadio::radioModeChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radio = check_and_cast<IRadio *>(radioModule);
        waitingForDC = false;
        dutyCycleTimer = new cMessage("Duty Cycle Timer");
        const char *addressString = par("address");
        GW_forwardedDown = 0;
        GW_droppedDC = 0;
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = DevAddr::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);
        GW_USED_TIME = registerSignal("GW_USED_TIME");
        GW_TRANSMITTED_PACKET = registerSignal("GW_TRANSMITTED_PACKET");
        sendMessageFromQueue = new cMessage("Send message from queue");
        freeAfterCurrent = 0;
        freeAfterLast = 0;
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        radio->setRadioMode(IRadio::RADIO_MODE_TRANSCEIVER);
    }
}

void LoRaGWMac::finish()
{
    recordScalar("GW_forwardedDown", GW_forwardedDown);
    recordScalar("GW_droppedDC", GW_droppedDC);

    recordScalar("Channel_0_used_time", usedTimes[0]);
    recordScalar("Channel_1_used_time", usedTimes[1]);
    recordScalar("Channel_2_used_time", usedTimes[2]);
    recordScalar("Channel_3_used_time", usedTimes[3]);
}


InterfaceEntry *LoRaGWMac::createInterfaceEntry()
{
    InterfaceEntry *e = new InterfaceEntry(this);

    // data rate
    //e->setDatarate(bitrate);

    // generate a link-layer address to be used as interface token for IPv6
    //e->setMACAddress(address);
    //e->setInterfaceToken(address.formInterfaceIdentifier());

    // capabilities
    //e->setMtu(par("mtu"));
    //e->setMulticast(true);
    //e->setBroadcast(true);
    //e->setPointToPoint(false);

    return e;
}

void LoRaGWMac::handleSelfMessage(cMessage *msg)
{
    if(msg == dutyCycleTimer) waitingForDC = false;
    if(msg == sendMessageFromQueue){
        if(simTime() == sendingQueue.front().sendingTime){
            LoRaMacFrame* frame = sendingQueue.front().frame;
            int PayloadLength = frame->getPayloadLength();
            if(PayloadLength == 0)
                PayloadLength = 20;
            double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
            emit(GW_USED_TIME,delta);
            GW_forwardedDown++;
            usedTimes[3] = usedTimes[3] + delta;

            sendDown(frame);
            sendingQueue.pop_front();
        }
        if(!sendingQueue.empty()) scheduleAt((sendingQueue.front()).sendingTime,sendMessageFromQueue);
    }
}

void LoRaGWMac::handleUpperPacket(cPacket *msg)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
    frame->removeControlInfo();
    simtime_t sendingTime = frame->getSendingTime();

    if(sendingTime >= freeAfterCurrent){
        int PayloadLength = frame->getPayloadLength();
        if(PayloadLength == 0)
            PayloadLength = 20;
        LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
        ctrl->setSrc(address);
        ctrl->setDest(frame->getReceiverAddress());
        frame->setControlInfo(ctrl);
        double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
        sendingQueue.emplace_back(sendingTime,sendingTime+(delta*10),frame->getReceiverAddress(),frame);
        scheduleAt(sendingTime,sendMessageFromQueue);
        freeAfterLast = freeAfterCurrent;
        freeAfterCurrent = sendingTime + (delta * 10);

        std::string addrStrwithId = (frame->getReceiverAddress()).str();
        addrStrwithId += ":";
        addrStrwithId += std::to_string(frame->getSequenceNumber());
        emit(GW_TRANSMITTED_PACKET,addrStrwithId.c_str());
    } else delete msg;
}

// LAKSH: Filtering of packets has been stopped
void LoRaGWMac::handleLowerPacket(cPacket *msg)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
    if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS){
        int PayloadLength = frame->getPayloadLength();
        double delta = timeOnAir(frame->getLoRaSF(),frame->getLoRaBW(), PayloadLength, frame->getLoRaCR());
        int channelNumber = ((frame->getLoRaCF() - inet::units::values::Hz(868100000))/inet::units::values::Hz(200000)).get();
        usedTimes[channelNumber] = usedTimes[channelNumber] + delta;
    }

    // sendUp(frame);
    if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS)
        sendUp(frame);
    else
        delete frame;
}

void LoRaGWMac::sendPacketBack(LoRaMacFrame *receivedFrame)
{
    EV << "sending Data frame back" << endl;
    LoRaMacFrame *frameToSend = new LoRaMacFrame("BackPacket");
    frameToSend->setReceiverAddress(receivedFrame->getTransmitterAddress());
    sendDown(frameToSend);
}

void LoRaGWMac::createFakeLoRaMacFrame()
{

}

void LoRaGWMac::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            //transmissin is finished
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
        }
        transmissionState = newRadioTransmissionState;
    }
}

DevAddr LoRaGWMac::getAddress() const
{
    return address;
}

std::string LoRaGWMac::str() const
{
    return "LoRaGWMac" + address.str();
}

simtime_t LoRaGWMac::getTimeForWhenNextMessageIsPossible() const
{
    if(!sendingQueue.empty())
        return (sendingQueue.back()).freeAfter;
    else
        return 0;
}

void LoRaGWMac::popDevAddr(DevAddr addr)
{
    for(auto it=sendingQueue.begin();it!=sendingQueue.end();it++){
        if((*it).addr == addr) {
            delete (*it).frame;
            if((*it).freeAfter == freeAfterCurrent){
                freeAfterCurrent = freeAfterLast;
            }
            sendingQueue.erase(it++);
        }
    }
}

}
