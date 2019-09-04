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

package loranetwork.LoraNode;

import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.AeseLoRaNic;
import loranetwork.LoRaApp.AeseLoRaActuatorApp;

module AeseLoRaActuatorNode
{
    parameters:
        string deploymentType = default("");
        double maxGatewayDistance = default(320.0);
        double gatewayX = default(320.0);
        double gatewayY = default(320.0);
        string mobilityType = default("StationaryMobility");
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=297,313;i=device/accesspoint;is=vs");
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=30,26");
        }
        mobility: <mobilityType> like IMobility {
            @display("p=24,88");
        }
        LoRaNic: AeseLoRaNic {
            @display("p=137,239");
        }
        LoRaApp: AeseLoRaActuatorApp {
            @display("p=137,32");
        }
    connections allowunconnected:

        LoRaApp.appOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> LoRaApp.appIn;
}
