package loranetwork.AeseNeighbours;

simple ReinforcementLearning
{
    int numberOfPastSlots = default(2);
    int numberOfFutureSlots = default(2);
    double alphaRL = default(0.9);
    double discountFactor = default(0.1);
    double epsilon = default(0.1);
    
    @class(inet::ReinforcementLearning);
    gates:
	    input lowerLayerIn @labels(ReinforcementLearning/up);
        output lowerLayerOut @labels(ReinforcementLearning/down);
}