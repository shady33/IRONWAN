package loranetwork.simulations;

import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import inet.node.internetcloud.InternetCloud;
import loranetwork.LoRaPhy.LoRaMedium;
import loranetwork.LoraNode.AeseLoRaActuatorNode;
import loranetwork.LoraNode.AeseLoRaNode;
import loranetwork.LoraNode.LoRaGW;
import loranetwork.LoraNode.LoRaNode;
import loranetwork.AeseNeighbours.PacketDuplicator;
import loranetwork.AeseNeighbours.LoRaAeseGW;

@license(LGPL);
network LoRaNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfAeseNodes = default(1);
        int numberOfAeseActuatorNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=355.824,356.832");
    submodules:
        AeseloRaNodes[numberOfAeseNodes]: AeseLoRaNode {
            @display("p=83.664,233.856");
        }
        AeseloRaActuatorNodes[numberOfAeseActuatorNodes]: AeseLoRaActuatorNode {
            @display("p=83.664,233.856");
        }
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=167.328,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=322,27");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=258,109");
        }
        internetCloud: InternetCloud {
            @display("p=138,27");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=24.192001,27.216002");
        }
        nsRouter: Router {
            @display("p=227,27");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

@license(LGPL);
network LoRaNetworkMultipleVendors
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfAeseNodes = default(1);
        int numberOfAeseActuatorNodes = default(1);
        int numberOfGatewaysPerNS = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        int numberOfNS = default(1);
        @display("bgb=355.824,356.832");
    submodules:
        AeseloRaNodes[numberOfAeseNodes]: AeseLoRaNode {
            @display("p=83.664,233.856");
        }
        AeseloRaActuatorNodes[numberOfAeseActuatorNodes]: AeseLoRaActuatorNode {
            @display("p=83.664,233.856");
        }
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGWNS0[numberOfGatewaysPerNS]: LoRaAeseGW {
            @display("p=83.664,150.192;is=s");
        }
        loRaGWNS1[numberOfGatewaysPerNS]: LoRaAeseGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=167.328,88.704");
        }
        networkServer[numberOfNS]: StandardHost;
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=258,109");
        }
        internetCloud: InternetCloud {
            @display("p=138,27");
        }
        gwRouter[numberOfGatewaysPerNS*2]: Router;
        nsRouter[numberOfNS]: Router;
    connections:
        for i=0..numberOfNS-1 {
            networkServer[i].ethg++ <--> Eth1G <--> nsRouter[i].ethg++;
            nsRouter[i].pppg++ <--> Eth1G <--> internetCloud.pppg++;
        }
        for i=0..numberOfGatewaysPerNS-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[numberOfGatewaysPerNS+i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGWNS0[i].ethg++;
            gwRouter[numberOfGatewaysPerNS+i].ethg++ <--> Eth1G <--> loRaGWNS1[i].ethg++;
        }
}
