#include "timeOnAir.h"

double timeOnAir(int LoRaSF, inet::units::values::Hz LoRaBW, int PayloadLength, int LoRaCR)
{
    int nPreamble = 8;
    omnetpp::simtime_t Tsym = (pow(2, LoRaSF))/(LoRaBW.get()/1000);
    omnetpp::simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    //preambleDuration = Tpreamble;
    int payloadSymbNb = 8 + inet::math::max(ceil((8*PayloadLength - 4*LoRaSF + 28 + 16 - 20*0)/(4*(LoRaSF-2*0)))*(LoRaCR + 4), 0);

    omnetpp::simtime_t Theader = 0.5 * (payloadSymbNb) * Tsym / 1000;
    omnetpp::simtime_t Tpayload = 0.5 * (payloadSymbNb) * Tsym / 1000;
    
    omnetpp::simtime_t duration = Tpreamble + Theader + Tpayload;
    return duration.dbl();
}