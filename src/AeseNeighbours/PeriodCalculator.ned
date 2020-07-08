package loranetwork.AeseNeighbours;

simple PeriodCalculator
{
    @class(inet::PeriodCalculator);
    gates:
		  input lowerLayerIn @labels(PeriodCalculator/up);
      output lowerLayerOut @labels(PeriodCalculator/down);
}