package loranetwork.AeseNeighbours;

simple NodeParams
{
    parameters:
        @class(inet::NodeParams);
        int numberOfNodes = default(0);
        int ratioOfAckToTotal = default(100);
	int nodeParamsRetryLimit = default(8);
}
