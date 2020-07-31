package loranetwork.AeseNeighbours;

simple PriorityHandlingAndScheduling
{
	@class(inet::PriorityHandlingAndScheduling);
    @display("i=block/switch");
    int AeseGWMode = default(0);
    gates:
        input lowerLayerIn;
        output lowerLayerOut;
        input packetForwarderIn;
        output packetForwarderOut;
        input neighbourTalkerIn;
        output neighbourTalkerOut;
}