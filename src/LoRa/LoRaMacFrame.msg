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

//
// TODO generated message class
//

cplusplus {{
#include "../misc/DevAddr.h"
#include <inet/common/Units.h>
}}

class noncobject DevAddr;
class noncobject inet::units::values::W;
class noncobject inet::units::values::Hz;

namespace inet;

enum MessagesTypesAndPriority
{
    MY_ACKS = 1;
    FIND_NEIGHBOURS_FOR_UPLINK = 2;
    SEND_ACCEPT_BIDS_FOR_NEIGHBOURS = 3;
    ACCEPTED_BIDS_ACKS = 4;
    FIND_NEIGHBOURS_FOR_DOWNLINK = 5;
}

enum MacPacketType
{
    JOIN_REQUEST = 0;
    FEEDBACK = 1;
    UPLINKMINISLOT = 2;
    UPLINKDATASLOT = 3;
    ACTUATION = 4;
    GW_PING_MESSAGE = 5;
    ACK_ADR_PACKET = 6;
    GW_HANDOFF_MESSAGE = 7;
}

enum CollisionInfo
{
    EMPTY = 0;
    COLLISION = 1;
    SUCCESS = 2;
}

packet LoRaMacFrame {
    DevAddr transmitterAddress;
    DevAddr receiverAddress;

    int sequenceNumber;
    double LoRaTP;
    inet::units::values::Hz LoRaCF;
    int LoRaSF;
    inet::units::values::Hz LoRaBW;
    int LoRaCR;
    bool LoRaUseHeader;
    double RSSI;
    double SNIR;
    int payloadLength;

    int msgType @enum(MacPacketType);
    int CRQ;
    int DTQ;
    long myMiniSlot;
    long myDataSlot;
    bool DataInThisFrame;
    int m[64] @enum(CollisionInfo);
    int n[64] @enum(CollisionInfo);
    int channelNumber[64];
    int position[64];
    int numberOfFrames;
    bool confirmedMessage;

    simtime_t sendingTime;
    simtime_t generatedTime;
    int type @enum(MessagesTypesAndPriority);
}
