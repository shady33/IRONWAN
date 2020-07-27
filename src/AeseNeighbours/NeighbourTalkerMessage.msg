cplusplus {{
#include "../misc/DevAddr.h"
#include <inet/common/Units.h>
#include <inet/networklayer/common/L3Address.h>
}}

class noncobject DevAddr;
class noncobject inet::units::values::W;
class noncobject inet::units::values::Hz;
class noncobject inet::L3Address;

namespace inet;

enum PacketType{
    REQUEST_HANDOFF = 0;
    CONFIRM_HANDOFF = 1;
    ACCEPT_HANDOFF  = 2;
}

packet NeighbourTalkerMessage {
    int msgType @enum(PacketType);

    L3Address sendingGWAddress;
    L3Address intendedGWAddress;
    simtime_t sendingTime;
    double RSSI;

    DevAddr deviceAddress;
    int sequenceNumber;
}
