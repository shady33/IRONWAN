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

package loranetwork.AeseNeighbours;

import inet.applications.contract.IUDPApp;

simple NeighbourTalker like IUDPApp
{
    @class(inet::NeighbourTalker);
    int AeseGWMode = default(0);
    double periodicPingInterval @unit(s)= default(100s);
    int numberOfGateways = default(1);
    string localAddress = default("");
    gates:
		input lowerLayerIn @labels(NeighbourTalker/up);
        output lowerLayerOut @labels(NeighbourTalker/down);
		input udpIn @labels(PacketForwarder/udpIn);
        output udpOut @labels(PacketForwarder/udpOut);
}