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

enum AppPacketType
{
    JOIN_REQUEST = 1;
    JOIN_REPLY = 2;
    DATA = 3;
    TXCONFIG = 4;
    DATADOWN = 5;
    DATANOSOCKET = 6;
    FEEDBACK = 7;
    UPLINK = 8;
    ACTUATION = 9;
}

class LoRaOptions {
    double LoRaTP = -1;
    double LoRaCF = -1;
    int LoRaSF = -1;
    double LoRaBW = -1;
    int LoRaCR = -1;
    bool UseHeader = true;
    bool ADRACKReq = false;
}

packet AeseAppPacket {
    int msgType @enum(AppPacketType);
    double sampleMeasurement;
    double actuationSignal[250];
    LoRaOptions options;
    double systemNumber;
    int numberOfFrames;
    simtime_t packetGeneratedTime[128];
    int actuatorNumber[128];
    int actuatorSequenceNumbers[128];
    int sensorNumber;
}
