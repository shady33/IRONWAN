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

package loranetwork.LoRa;


import inet.applications.contract.IUDPApp;

simple PacketForwarder like IUDPApp
{
    @class(inet::PacketForwarder);
    @signal[LoRa_GWPacketReceived](type=long); // optional
    @statistic[LoRa_GWPacketReceived](source=LoRa_GWPacketReceived; record=count);

    int localPort = default(-1);  // local port (-1: use ephemeral port)
    string destAddresses = default(""); // list of IP addresses, separated by spaces ("": don't send)
    string localAddress = default("");
    int networkServerNumber = default(0);
    int numberOfNS = default(0);
    int destPort;
	double timeToStartSignal @unit(s) = default(10s);
    double actuationPeriod @unit(s) = default(1s);
    
    int noOfMslots = default(3);
    int noOfNslots = default(1);
    double mSlotDuration @unit(s) = default(0.1s);
    double nSlotDuration @unit(s) = default(0.1s);
    bool dataOnSameChannel = default(true);
    int numberOfChannels = default(0);

    int numberOfNodes = default(0);
    int numberOfAeseNodes = default(0);
    int numberOfAeseActuatorNodes = default(0);
    int numberOfSubSystems = default(0);
    
    bool sendImmediateActuation = default(false);

    bool enableDQ = default(false);
    bool enableActuation = default(false);

    int numberOfGateways = default(1);
    gates:
		input lowerLayerIn @labels(PacketForwarder/up);
        output lowerLayerOut @labels(PacketForwarder/down);
        input udpIn @labels(PacketForwarder/udpIn);
        output udpOut @labels(PacketForwarder/udpOut);

}
