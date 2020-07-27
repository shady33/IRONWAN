package loranetwork.AeseNeighbours;

simple ReinforcementLearning
{
    int numberOfPastSlots = default(8);
    int numberOfFutureSlots = default(8);
    double alphaRL = default(0.8);
    double discountFactor = default(0.1);
    double epsilon = default(0.2);
    
    @class(inet::ReinforcementLearning);
    gates:
	    input lowerLayerIn @labels(ReinforcementLearning/up);
        output lowerLayerOut @labels(ReinforcementLearning/down);
}
