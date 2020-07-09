package loranetwork.AeseNeighbours;

simple PeriodCalculator
{
    double tValue = default(0.703);
    @class(inet::PeriodCalculator);
    gates:
		  input lowerLayerIn @labels(PeriodCalculator/up);
      output lowerLayerOut @labels(PeriodCalculator/down);
}