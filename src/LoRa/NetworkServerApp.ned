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

simple NetworkServerApp like IUDPApp
{
    @class(inet::NetworkServerApp);
    @signal[LoRa_ServerPacketReceived](type=bool); // optional
    @statistic[LoRa_ServerPacketReceived](source=LoRa_ServerPacketReceived; record=count);
    int localPort = default(-1);  // local port (-1: use ephemeral port)
    string destAddresses = default(""); // list of IP addresses, separated by spaces ("": don't send)
    string localAddress = default("");
    int destPort = default(-1);
    int AeseGWMode = default(0);
    bool evaluateADRinServer = default(false);
	int networkServerNumber = default(0);
	string adrMethod = default("max");
	
    gates:
    output udpOut;
    input udpIn;

}