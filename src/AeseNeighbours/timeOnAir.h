#ifndef _AESE_TIME_ON_AIR_
#define _AESE_TIME_ON_AIR_

#include <omnetpp.h>
#include "inet/common/Units.h"
#include "inet/common/INETMath.h"

double timeOnAir(int LoRaSF, inet::units::values::Hz LoRaBW, int PayloadLength, int LoRaCR);

#endif // _AESE_TIME_ON_AIR_