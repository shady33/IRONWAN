package loranetwork.AeseNeighbours;

simple PriorityHandlingAndScheduling
{
	@class(inet::PriorityHandlingAndScheduling);
    @display("i=block/switch");

    gates:
        input lowerLayerIn;
        output lowerLayerOut;
        input packetForwarderIn;
        output packetForwarderOut;
        input neighbourTalkerIn;
        output neighbourTalkerOut;
}