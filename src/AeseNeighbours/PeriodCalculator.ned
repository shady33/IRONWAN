package loranetwork.AeseNeighbours;

simple PeriodCalculator
{
    double tValue = default(0.703);
    @class(inet::PeriodCalculator);
    int AeseGWMode = default(0);
    gates:
	    input lowerLayerIn @labels(PeriodCalculator/up);
        output lowerLayerOut @labels(PeriodCalculator/down);
}