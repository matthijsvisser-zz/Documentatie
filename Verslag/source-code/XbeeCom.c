#include "XbeeCom.h"

// List of Rxpackets
XBee_packet *RxPackets;
unsigned char RxTail;
unsigned char RxHead;

// List of Txpackets
XBee_packet *TxPackets;
unsigned char TxTail;
unsigned char TxHead;

// Temporary packet being received or transmitted
XBee_packet RxPacket;
XBee_packet TxPacket;
unsigned char RxState;
unsigned char RxChecksum;
unsigned char RxDataCount;

void XBee_communication_init(void) {
	RxState = 0;
	RxChecksum = 0;
	RxDataCount = 0;
	
	RxPackets = (XBee_packet *) malloc(sizeof(XBee_packet) * MAX_PACKETS_RX);
	RxTail = 0;
	RxHead = 0;
	
	TxPackets = (XBee_packet *) malloc(sizeof(XBee_packet) * MAX_PACKETS_TX);
	TxTail = 0;
	TxHead = 0;
}



// Return number of RxPackets in buffer
unsigned char XBee_communication_RxPacketsAvailable(void) {
	if(RxHead >= RxTail)
	return RxHead - RxTail;
	else
	return MAX_PACKETS_RX - (RxTail - RxHead);
}

XBee_packet *XBee_communication_getRxPacket(void) {
	if(XBee_communication_RxPacketsAvailable() > 0) {
		unsigned char ptr = RxTail;
		
		if((RxTail + 1) > MAX_PACKETS_RX) {
			RxTail = 0;
			} else {
			RxTail++;
		}
		
		return &RxPackets[ptr];
		} else {
		return NULL;
	}
}

void XBee_communication_saveRxPacket(XBee_packet *packet) {
	
	memcpy(&RxPackets[RxHead], packet, sizeof(XBee_packet));
	
	if(RxHead >= MAX_PACKETS_RX)
	RxHead = 0;
	else
	RxHead++;
	
	if(RxTail == RxHead) {
		if((RxTail + 1) > MAX_PACKETS_RX)
		RxTail = 0;
		else
		RxTail = RxTail + 1;
	}
}

unsigned char XBee_communication_processSerialData(char *data, unsigned char length) {
	unsigned char countNewPackets = 0;
	
	for(unsigned char i = 0; i < length; i++) {
		unsigned char c = data[i];
		//printf("State: %d (0x%x)\r\n", RxState, c);
		
		if(RxState == 0) {
				if (c == PACKET_START_BYTE_1) {
					RxState = 1;
					} else {
					RxState = 0;
				}
			} else if(RxState == 1) {
				if(c == PACKET_START_BYTE_2) {
					RxState = 2;
					} else {
					RxState = 0;
				}
			} else if(RxState == 2) {
				if(c == PACKET_START_BYTE_3) {
					RxState = 3;
					} else {
					RxState = 0;
				}
			} else if(RxState == 3) {
				if(c == PACKET_START_BYTE_4) {
					RxState = 4;
					} else {
					RxState = 0;
				}
			} else if(RxState == 4) {
				RxPacket.length = c;
				RxChecksum |= c;
				if(RxPacket.length > MAX_DATA_SIZE) {
					RxState = 0;
				} else {
					RxState = 5;
				}
			} else if(RxState == 5) {
				RxPacket.command = c;
				RxChecksum |= c;
			
				RxState = 6;
			} else if(RxState == 6) {
				if(RxDataCount < RxPacket.length) {
					RxPacket.data[RxDataCount++] = c;
					RxChecksum |= c;
				}
				if(RxDataCount >= RxPacket.length){
					RxState = 7;
				}
			} else if(RxState == 7) {
				RxPacket.checksum = c;
			
				if(RxPacket.checksum == RxChecksum) {
					XBee_communication_saveRxPacket(&RxPacket);
					countNewPackets++;
				} else {
					printf("Checksum failed: 0x%x vs 0x%x\r\n", RxPacket.checksum, RxChecksum);
				}
			
				RxState = 0, RxChecksum = 0, RxDataCount = 0;
			
		}
		
	}
	
	return countNewPackets;
}
