#ifndef __AESE_SUPPORTEDPROTOCOLS_H
#define __AESE_SUPPORTEDPROTOCOLS_H

// Only LoRa supported (as of September 2019)

enum SupportedProtocols {
	LORA, 
	SIGFOX,
	NBIoT,
	WiFi,
	BLE
};

#endif // __AESE_SUPPORTEDPROTOCOLS_H
