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

#include <LoRaReceiver.h>
#include "inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h"

namespace inet {

namespace physicallayer {

Define_Module(LoRaReceiver);

LoRaReceiver::LoRaReceiver() :
    snirThreshold(NaN)
{
}

void LoRaReceiver::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {
        snirThreshold = math::dB2fraction(par("snirThreshold"));
        if(strcmp(getParentModule()->getClassName(), "inet::physicallayer::LoRaGWRadio") == 0)
        {
            iAmGateway = true;
        } else iAmGateway = false;
        alohaChannelModel = par("alohaChannelModel");
        LoRaReceptionCollision = registerSignal("LoRaReceptionCollision");
        LoRaMiniSlotCollision = registerSignal("LoRaMiniSlotCollision");
        LoRaDataSlotCollision = registerSignal("LoRaDataSlotCollision");
        numCollisions = 0;
        rcvBelowSensitivity = 0;
        dataChecked = false;
        if(iAmGateway == false){
            // Laksh: These are parameters for downlink to nodes, change based on system
            std::string appLayer = getContainingNode(this)->getSubmodule("LoRaApp")->str();
             if(appLayer.compare("SimpleLoRaApp") == 0){
                 SimpleLoRaApp *loRaApp = check_and_cast<SimpleLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
                 LoRaMac *loRamac = check_and_cast<LoRaMac *>(getContainingNode(this)->getSubmodule("LoRaNic")->getSubmodule("mac"));
                 // loRaCF = loRamac->carrierFrequency;
                 loRaCF = units::values::Hz(869460500);
                 loRaSF = 9;
                 loRaBW = units::values::Hz(125000);
             }else if(appLayer.compare("AeseLoRaApp") == 0){
                 // FIXME: Laksh: this is hardcoded
                 //AeseLoRaApp *loRaApp = check_and_cast<AeseLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//                 loRaCF = loRaApp->loRaCF;
//                 loRaSF = loRaApp->loRaSF;
//                 loRaBW = loRaApp->loRaBW;
                 loRaCF = units::values::Hz(869460500);
                 loRaSF = 7;
                 loRaBW = units::values::Hz(125000);
             }else if(appLayer.compare("AeseLoRaActuatorApp") == 0){
//                 AeseLoRaActuatorApp *loRaApp = check_and_cast<AeseLoRaActuatorApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
                 loRaCF = units::values::Hz(869587500);
                 loRaSF = 7;
                 loRaBW = units::values::Hz(125000);
             }
        }
    }
}

void LoRaReceiver::finish()
{
    recordScalar("numCollisions", numCollisions);
    recordScalar("rcvBelowSensitivity", rcvBelowSensitivity);
}

bool LoRaReceiver::computeIsReceptionPossible(const IListening *listening, const ITransmission *transmission) const
{
    //here we can check compatibility of LoRaTx parameters (or beeing a gateway)
    const LoRaTransmission *loRaTransmission = check_and_cast<const LoRaTransmission *>(transmission);

//    std::string s = getContainingNode(this)->getSubmodule("LoRaApp")->str();
//
//    if(s.compare("SimpleLoRaApp") == 0){
//        SimpleLoRaApp *loRaApp = check_and_cast<SimpleLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//        LoRaMac *loRamac = check_and_cast<LoRaMac *>(getContainingNode(this)->getSubmodule("LoRaNic")->getSubmodule("mac"));
//        loRaCF = loRamac->carrierFrequency;
//        loRaSF = loRaApp->loRaSF;
//        loRaBW = loRaApp->loRaBW;
//    }else if(s.compare("AeseLoRaApp") == 0){
//        AeseLoRaApp *loRaApp = check_and_cast<AeseLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//        loRaCF = loRaApp->loRaCF;
//        loRaSF = loRaApp->loRaSF;
//        loRaBW = loRaApp->loRaBW;
//    }else if(s.compare("AeseLoRaActuatorApp") == 0){
//        AeseLoRaActuatorApp *loRaApp = check_and_cast<AeseLoRaActuatorApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//        loRaCF = loRaApp->loRaCF;
//        loRaSF = loRaApp->loRaSF;
//        loRaBW = loRaApp->loRaBW;
//    }

    if(iAmGateway || (loRaTransmission->getLoRaCF() == loRaCF && loRaTransmission->getLoRaBW() == loRaBW && loRaTransmission->getLoRaSF() == loRaSF))
        return true;
    else
        return false;
}

bool LoRaReceiver::computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const
{
    //here we can check compatibility of LoRaTx parameters (or beeing a gateway) and reception above sensitivity level
    const LoRaBandListening *loRaListening = check_and_cast<const LoRaBandListening *>(listening);
    const LoRaReception *loRaReception = check_and_cast<const LoRaReception *>(reception);
    if (iAmGateway == false && (loRaListening->getLoRaCF() != loRaReception->getLoRaCF() || loRaListening->getLoRaBW() != loRaReception->getLoRaBW() || loRaListening->getLoRaSF() != loRaReception->getLoRaSF())) {
        return false;
    } else {
        W minReceptionPower = loRaReception->computeMinPower(reception->getStartTime(part), reception->getEndTime(part));
        W sensitivity = getSensitivity(loRaReception);
        bool isReceptionPossible = minReceptionPower >= sensitivity;
        EV_DEBUG << "Computing whether reception is possible: minimum reception power = " << minReceptionPower << ", sensitivity = " << sensitivity << " -> reception is " << (isReceptionPossible ? "possible" : "impossible") << endl;
        if(isReceptionPossible == false) {
           const_cast<LoRaReceiver* >(this)->rcvBelowSensitivity++;
        }
        return isReceptionPossible;
    }
}

bool LoRaReceiver::computeIsReceptionAttempted(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const
{
    if(isPacketCollided(reception, part, interference))
    {
        auto macFrame = const_cast<cPacket *>(reception->getTransmission()->getMacFrame());
        LoRaMacFrame *loraMacFrame = check_and_cast<LoRaMacFrame *>(macFrame);

        if (iAmGateway == false) {
            std::string s = getParentModule()->getParentModule()->getSubmodule("mac")->str();
            DevAddr addr;
            if(s.compare("LoRaMac") == 0){
                LoRaMac *macLayer = check_and_cast<LoRaMac *>(getParentModule()->getParentModule()->getSubmodule("mac"));    
                addr = macLayer->getAddress();
            }else if(s.compare("LoRaAeseMac") == 0){
                LoRaAeseMac *macLayer = check_and_cast<LoRaAeseMac *>(getParentModule()->getParentModule()->getSubmodule("mac"));
                addr = macLayer->getAddress();
            }
            if (loraMacFrame->getReceiverAddress() == addr) {
                const_cast<LoRaReceiver* >(this)->numCollisions++;
            }
            //EV << "Node: Extracted macFrame = " << loraMacFrame->getReceiverAddress() << ", node address = " << macLayer->getAddress() << std::endl;
        } else {
            LoRaGWMac *gwMacLayer = check_and_cast<LoRaGWMac *>(getParentModule()->getParentModule()->getSubmodule("mac"));
            EV << "GW: Extracted macFrame = " << loraMacFrame->getReceiverAddress() << ", node address = " << gwMacLayer->getAddress() << std::endl;
            if (loraMacFrame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS) {
                const_cast<LoRaReceiver* >(this)->numCollisions++;
            }
        }
        return false;
    } else {
        return true;
    }
}

bool LoRaReceiver::isPacketCollided(const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const
{
    auto radio = reception->getReceiver();
    //auto radioMedium = radio->getMedium();
    auto interferingReceptions = interference->getInterferingReceptions();
    const LoRaReception *loRaReception = check_and_cast<const LoRaReception *>(reception);
    simtime_t m_x = (loRaReception->getStartTime() + loRaReception->getEndTime())/2;
    simtime_t d_x = (loRaReception->getEndTime() - loRaReception->getStartTime())/2;
    // double P_threshold = 6;
    W signalRSSI_w = loRaReception->getPower();
    double signalRSSI_mw = signalRSSI_w.get()*1000;
    double signalRSSI_dBm = math::mW2dBm(signalRSSI_mw);
    for (auto interferingReception : *interferingReceptions) {
        bool overlap = false;
        bool frequencyColision = false;
        // bool spreadingFactorColision = false;
        bool captureEffect = false;
        // bool timingCollison = false; //Collision is acceptable in first part of preambles
        const LoRaReception *loRaInterference = check_and_cast<const LoRaReception *>(interferingReception);

        simtime_t m_y = (loRaInterference->getStartTime() + loRaInterference->getEndTime())/2;
        simtime_t d_y = (loRaInterference->getEndTime() - loRaInterference->getStartTime())/2;
        if(omnetpp::fabs(m_x - m_y) < d_x + d_y)
        {
            overlap = true;
        }

        if(loRaReception->getLoRaCF() == loRaInterference->getLoRaCF())
        {
            frequencyColision = true;
        }

        // if(loRaReception->getLoRaSF() == loRaInterference->getLoRaSF())
        // {
        //     spreadingFactorColision = true;
        // }

        W interferenceRSSI_w = loRaInterference->getPower();
        double interferenceRSSI_mw = interferenceRSSI_w.get()*1000;
        double interferenceRSSI_dBm = math::mW2dBm(interferenceRSSI_mw);

        if(signalRSSI_dBm - interferenceRSSI_dBm >= nonOrthDelta[loRaReception->getLoRaSF()-7][loRaInterference->getLoRaSF()-7])
        {
            captureEffect = true;
        }

        // double nPreamble = 8; //from the paper "Does Lora networks..."
        //double Npream = nPreamble + 4.25; //4.25 is a constant added by Lora Transceiver
        
        auto macFrame = const_cast<cPacket *>(reception->getTransmission()->getMacFrame());
        LoRaMacFrame *loraMacFrame = check_and_cast<LoRaMacFrame *>(macFrame);

        // simtime_t Tsym = (pow(2, loRaReception->getLoRaSF()))/(loRaReception->getLoRaBW().get()/1000)/1000;
        // simtime_t csBegin = loRaReception->getPreambleStartTime() + Tsym * (nPreamble - 5);
        // if(csBegin < loRaInterference->getEndTime())
        // {
        //     timingCollison = true;
        // }
        if (overlap && frequencyColision)// && spreadingFactorColision) // && captureEffect && timingCollison)
        {
            if(alohaChannelModel == true)
            {
                if(iAmGateway && (part == IRadioSignal::SIGNAL_PART_DATA || part == IRadioSignal::SIGNAL_PART_WHOLE)){ 
                    const_cast<LoRaReceiver* >(this)->emit(LoRaReceptionCollision, loraMacFrame->getMyMiniSlot() + loraMacFrame->getMyDataSlot());
                    if(loraMacFrame->getMsgType() == UPLINKMINISLOT) const_cast<LoRaReceiver* >(this)->emit(LoRaMiniSlotCollision, loraMacFrame->getMyMiniSlot());
                    else if(loraMacFrame->getMsgType() == UPLINKDATASLOT) const_cast<LoRaReceiver* >(this)->emit(LoRaDataSlotCollision, loraMacFrame->getMyDataSlot());
                }
                return true;
            }
            if(alohaChannelModel == false)
            {
                if(captureEffect == false)// && timingCollison)
                {
                    if(iAmGateway && (part == IRadioSignal::SIGNAL_PART_DATA || part == IRadioSignal::SIGNAL_PART_WHOLE)){
                        const_cast<LoRaReceiver* >(this)->emit(LoRaReceptionCollision, loraMacFrame->getMyMiniSlot() + loraMacFrame->getMyDataSlot());
                        if(loraMacFrame->getMsgType() == UPLINKMINISLOT) const_cast<LoRaReceiver* >(this)->emit(LoRaMiniSlotCollision, loraMacFrame->getMyMiniSlot());
                        else if(loraMacFrame->getMsgType() == UPLINKDATASLOT) const_cast<LoRaReceiver* >(this)->emit(LoRaDataSlotCollision, loraMacFrame->getMyDataSlot());
                    }
                    return true;
                }
            }

        }
    }
    return false;
}

const ReceptionIndication *LoRaReceiver::computeReceptionIndication(const ISNIR *snir) const
{
    const ScalarSNIR *scalarSNIR = check_and_cast<const ScalarSNIR *>(snir);
    auto indication = createReceptionIndication();
    //indication->setMinSNIR(snir->getMin());
    indication->setMinSNIR(scalarSNIR->getMin());

    const LoRaReception *loRaReception = check_and_cast<const LoRaReception *>(scalarSNIR->getReception());
    indication->setMinRSSI(loRaReception->getPower());
    return indication;
}

ReceptionIndication *LoRaReceiver::createReceptionIndication() const
{
    return new ReceptionIndication();
}
const IReceptionDecision *LoRaReceiver::computeReceptionDecision(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISNIR *snir) const
{
    auto isReceptionPossible = computeIsReceptionPossible(listening, reception, part);
    auto isReceptionAttempted = isReceptionPossible && computeIsReceptionAttempted(listening, reception, part, interference);
    auto isReceptionSuccessful = isReceptionAttempted && computeIsReceptionSuccessful(listening, reception, part, interference, snir);
    return new ReceptionDecision(reception, part, isReceptionPossible, isReceptionAttempted, isReceptionSuccessful);
}

const IReceptionResult *LoRaReceiver::computeReceptionResult(const IListening *listening, const IReception *reception, const IInterference *interference, const ISNIR *snir) const
{
    auto radio = reception->getReceiver();
    auto radioMedium = radio->getMedium();
    auto transmission = reception->getTransmission();
    //const LoRaReception *loRaReception = check_and_cast<const LoRaReception *>(reception);
    //W RSSI = loRaReception->computeMinPower(reception->getStartTime(part), reception->getEndTime(part));
    auto indication = computeReceptionIndication(snir);
    // TODO: add all cached decisions?
    auto decisions = new std::vector<const IReceptionDecision *>();
    decisions->push_back(radioMedium->getReceptionDecision(radio, listening, transmission, IRadioSignal::SIGNAL_PART_WHOLE));
    return new ReceptionResult(reception, decisions, indication);
}

bool LoRaReceiver::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISNIR *snir) const
{
    return true;
    //we don't check the SINR level, it is done in collision checking by P_threshold level evaluation
}

const IListening *LoRaReceiver::createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition) const
{
    if(iAmGateway == false)
    {
//        std::string s = getContainingNode(this)->getSubmodule("LoRaApp")->str();
//
//        units::values::Hz loRaCF;
//        int loRaSF;
//        units::values::Hz loRaBW;

//        if(s.compare("SimpleLoRaApp") == 0){
//            SimpleLoRaApp *loRaApp = check_and_cast<SimpleLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//            LoRaMac *loRamac = check_and_cast<LoRaMac *>(getContainingNode(this)->getSubmodule("LoRaNic")->getSubmodule("mac"));
//            loRaCF = loRamac->carrierFrequency;
//            loRaSF = loRaApp->loRaSF;
//            loRaBW = loRaApp->loRaBW;
//        }else if(s.compare("AeseLoRaApp") == 0){
//            AeseLoRaApp *loRaApp = check_and_c§ast<AeseLoRaApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//            loRaCF = loRaApp->loRaCF;
//            loRaSF = loRaApp->loRaSF;
//            loRaBW = loRaApp->loRaBW;
//        }else if(s.compare("AeseLoRaActuatorApp") == 0){
//            AeseLoRaActuatorApp *loRaApp = check_and_cast<AeseLoRaActuatorApp *>(getContainingNode(this)->getSubmodule("LoRaApp"));
//            loRaCF = loRaApp->loRaCF;
//            loRaSF = loRaApp->loRaSF;
//            loRaBW = loRaApp->loRaBW;
//        }

        return new LoRaBandListening(radio, startTime, endTime, startPosition, endPosition, loRaCF, loRaSF, loRaBW);
    }
    else return new LoRaBandListening(radio, startTime, endTime, startPosition, endPosition, LoRaCF, LoRaSF, LoRaBW);
}

const IListeningDecision *LoRaReceiver::computeListeningDecision(const IListening *listening, const IInterference *interference) const
{
    const IRadio *receiver = listening->getReceiver();
    const IRadioMedium *radioMedium = receiver->getMedium();
    const IAnalogModel *analogModel = radioMedium->getAnalogModel();
    const INoise *noise = analogModel->computeNoise(listening, interference);
    const ScalarNoise *loRaNoise = check_and_cast<const ScalarNoise *>(noise);
    W maxPower = loRaNoise->computeMaxPower(listening->getStartTime(), listening->getEndTime());
    bool isListeningPossible = maxPower >= energyDetection;
    delete noise;
    EV_DEBUG << "Computing whether listening is possible: maximum power = " << maxPower << ", energy detection = " << energyDetection << " -> listening is " << (isListeningPossible ? "possible" : "impossible") << endl;
    return new ListeningDecision(listening, isListeningPossible);
}

W LoRaReceiver::getSensitivity(const LoRaReception *reception) const
{
    //function returns sensitivity -- according to LoRa documentation, it changes with LoRa parameters
    //Sensitivity values from Semtech SX1272/73 datasheet, table 10, Rev 3.1, March 2017
    W sensitivity = W(math::dBm2mW(-126.5) / 1000);
    if(reception->getLoRaSF() == 6)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-121) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-118) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-111) / 1000);
    }

    if (reception->getLoRaSF() == 7)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-124) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-122) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-116) / 1000);
    }

    if(reception->getLoRaSF() == 8)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-127) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-125) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-119) / 1000);
    }
    if(reception->getLoRaSF() == 9)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-130) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-128) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-122) / 1000);
    }
    if(reception->getLoRaSF() == 10)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-133) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-130) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-125) / 1000);
    }
    if(reception->getLoRaSF() == 11)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-135) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-132) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-128) / 1000);
    }
    if(reception->getLoRaSF() == 12)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBm2mW(-137) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBm2mW(-135) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBm2mW(-129) / 1000);
    }
    return sensitivity;
}

}
}
