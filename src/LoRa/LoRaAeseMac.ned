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

//based on Ieee802154NarrowbandMac

package loranetwork.LoRa;

import inet.linklayer.csmaca.CsmaCaMac; 

simple LoRaAeseMac extends CsmaCaMac
{
    parameters:
        bitrate = 250bps;
        @class(inet::LoRaAeseMac);
	    int noOfMslots = default(3);
		int noOfNslots = default(1);
		double mSlotDuration @unit(s) = default(0.1s);
	    double nSlotDuration @unit(s) = default(0.1s);
        bool dataOnSameChannel = default(true);
        int numberOfChannels = default(0);
        gates:
        	input upperMgmtIn;
        	output upperMgmtOut;
}
