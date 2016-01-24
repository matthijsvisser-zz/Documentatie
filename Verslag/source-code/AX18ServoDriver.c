#include "AX18ServoDriver.h"

/**
 * Generate checksum for Writing
 * @param  id      Servo identifier
 * @param  address Servo memory write address
 * @param  data    Data to write to memory
 * @param  length  Length of data
 * @return         Checksum
 */
unsigned char generateTxChecksum(unsigned char id, unsigned char address, unsigned char *data, unsigned length) {

	unsigned char checksum = id + (length + 3) + AX_WRITE_DATA + address;
	for(unsigned char i = 0; i < length; i++) {
		checksum += data[i];
	}
	return ~checksum;
}

unsigned char generateRxRequestChecksum(unsigned char id, unsigned char length, unsigned char address) {

	unsigned char checksum = id + 4 + AX_READ_DATA + length +address;

	return ~checksum;
}

/**
 * Generate checksum for Reading
 * @param  id      Servo identifier
 * @param  error   Error returned from Servo
 * @param  address Servo memory read address
 * @param  data    received data
 * @param  length  length of received data
 * @return         checksum
 */
unsigned char generateRxChecksum(unsigned char id, unsigned char error, unsigned char address, unsigned char *data, unsigned char length) {

	unsigned char checksum = id + (length + 3) + error + AX_READ_DATA + address;
	for(unsigned char i = 0; i < length; i++) {
		checksum += data[i];
	}
	return ~checksum;
}


/**
 * Writes to the Servo
 * @param id      Servo identifier
 * @param address Servo memory write address
 * @param data    Data to write to memory
 * @param length  Length of data
 */
void AX18FWrite(unsigned char id, unsigned char address, unsigned char *data, unsigned char length) {

	// Enable Uart Tx so we can send
	uart1_RxDisable();
	uart1_clearTxBuffer();
	uart1_TxEnable();

	uart1_putc(AX_START);
	uart1_putc(AX_START);
	uart1_putc(id);
	uart1_putc(length + 3);
	uart1_putc(AX_WRITE_DATA);
	uart1_putc(address);

	for(unsigned char i = 0; i < length; i++) {
		uart1_putc(data[i]);
	}

	uart1_putc(generateTxChecksum(id, address, data, length));
	//_delay_us(500);
	uart1_TxWaitDisable();
}

void AX18FSyncWrite(unsigned char *ids, unsigned char numIds, unsigned char startAddress, unsigned char *data, unsigned char length) {

	unsigned char checksum = 0; // = ~ ( ID + Length + Instruction + Parameter1 + � Parameter N )
	// Enable Uart Tx so we can send
	uart1_RxDisable();
	uart1_clearTxBuffer();
	uart1_TxEnable();

	uart1_putc(AX_START);
	uart1_putc(AX_START);

	uart1_putc(BROADCAST_ID);
	checksum += BROADCAST_ID;

	unsigned char totalLength = (length+1) * numIds + 4; // (L+1) X N + 4   (L: Data Length per RX-64, N: the number of RX-64s)
	uart1_putc(totalLength);
	checksum += totalLength;


	uart1_putc(AX_SYNC_WRITE);
	checksum += AX_SYNC_WRITE;

	uart1_putc(startAddress);
	checksum += startAddress;

	uart1_putc(length);
	checksum += length;


	for(int i = 0; i < numIds;i++) {
		uart1_putc(ids[i]);
		checksum += ids[i];
		for(int b = 0; b < length; b++) {
			uart1_putc(data[b]);
			checksum += data[b];
		}
	}
	checksum = ~checksum;
	uart1_putc(checksum);

	uart1_TxWaitDisable();
}

/**
 * Reads from the servo
 * @param id      Servo identifier
 * @param address Servo memory read address
 * @param buffer  Buffer to save data
 * @param length  Length to read
 * @return        Error occurred, return 0 on success, 1 on error
 */
unsigned char AX18FRead(unsigned char id, unsigned char address, unsigned char *buffer, unsigned char length) {

	unsigned char checksum = generateRxRequestChecksum(id, length, address);

	// Enable Uart Tx so we can send
	uart1_clearTxBuffer();
	uart1_TxEnable();
	uart1_RxDisable();

	uart1_putc(AX_START);
	uart1_putc(AX_START);
	uart1_putc(id);
	uart1_putc(4);
	uart1_putc(AX_READ_DATA);
	uart1_putc(address);
	uart1_putc(length);
	uart1_putc(checksum);

	uart1_TxWaitDisable();
	uart1_RxEnable(); // TEMP

	// Wait for response
	while(uart1_canRead() <= 0);

	//_delay_us(TX_READ_DELAY_TIME);
	unsigned char RxState = 0, RxDataCount = 0, RxServoId = 0, RxLength = 0, RxError = 0, RxChecksum = 0, Error = 0;

	// Wait a couple of micro seconds to receive some data
	// Loop trough all received bytes
	while(uart1_canRead() > 0) {
		//printf("Buffer size: %d\r\n", uart1_canRead())
		char c = uart1_getc();
	printf("(%d): 0x%x\r\n", RxState, c);
		switch(RxState) {

			// 1) First Start byte
			case 0:
				if(c == AX_START) {
					RxState = 1;
				}
			break;

			// 2) Second start byte
			case 1:
				if(c == AX_START) {
					RxState = 2;
				} else {
					RxState = 0;
				}
			break;

			// 3) Id byte
			case 2:
				if(c != AX_START) {
					RxServoId = c;
					RxState = 3;
				} else {
					RxState = 0;
					Error = 1;
				}
			break;

			// 4) Length byte
			case 3:
				RxLength = c;
				RxState = 4;
			break;

			// 5) Error byte
			case 4:
				RxError = c;
				RxState = 5;
			break;

			// Data bytes and checksum byte
			case 5:
				if(RxDataCount >= RxLength - 2) {
					RxChecksum = c;
					RxState = 6;
					break;
				}
				printf("Data (%d): 0x%x\r\n", RxDataCount, c);
				buffer[RxDataCount++] = c;
			break;

			// There is no state 6 unless we got more data then expected...
			case 6:
				Error = 1;
			break;

		}
	}

	// Check if packet is correct by comparing the checksum
	if(generateRxChecksum(RxServoId, RxError, address, buffer, length) != RxChecksum) {
		Error = 1;
	}

	printf("error: 0x%x\r\n", RxError);

	return Error;

}

/**
 * Set goal position of servo. Range of position is 0 - 1023.
 * @param id  Servo identifier
 * @param pos Goal position
 */
void AX18SetPosition(unsigned char id, unsigned long pos) {

	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(pos),
		unsigned16ToUnsigned8Higher(pos)};
printf("p(%d):%d\r\n", id, pos);
	AX18FWrite(id, AX_GOAL_POSITION_L, buffer, 2);
}

/**
 * Set speed of servo. This speed will be used in both JOIN and WHEEL mode.
 * @param id    Servo identifier
 * @param speed Speed
 */
void AX18SetSpeed(unsigned char id, unsigned long speed) {
	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(speed),
		unsigned16ToUnsigned8Higher(speed)};

	AX18FWrite(id, AX_GOAL_SPEED_L, buffer, 2);
}

unsigned long AX18GetSpeed(unsigned char id) {

	unsigned char buffer[2];
	if(AX18FRead(id, AX_GOAL_SPEED_L, buffer, 2) == 0)
		return 0;
	return unsigned8ToUnsigned16(buffer[0], buffer[1]);

}

void AX18SetTorque(unsigned char id, unsigned long torque) {
	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(torque),
	unsigned16ToUnsigned8Higher(torque)};

	printf("Torque set to: %d (0x%x, 0x%x)\r\n", (int) torque, buffer[0], buffer[1]);

	AX18FWrite(id, AX_MAX_TORQUE_L, buffer, 2);
}

void AX18SetTorqueLimit(unsigned char id, unsigned long torque) {
	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(torque),
		unsigned16ToUnsigned8Higher(torque)};


	printf("Torque limit set to: %d (0x%x, 0x%x)\r\n", (int) torque, buffer[0], buffer[1]);

	AX18FWrite(id, AX_TORQUE_LIMIT_L, buffer, 2);
}

unsigned long AX18GetTorque(unsigned char id) {

	unsigned char buffer[2];
	if(AX18FRead(id, AX_MAX_TORQUE_L, buffer, 2) == 0)
		return 0;
	return unsigned8ToUnsigned16(buffer[0], buffer[1]);

}

void AX18SetLed(unsigned char id, unsigned char on) {
	if(on == 0 || on == 1)
		AX18FWrite(id, AX_LED, &on, 1);
}

unsigned char AX18GetLed(unsigned char id) {
	unsigned char buffer[1];
	if(AX18FRead(id, AX_LED, buffer, 1) == 0)
		return 0;
	return buffer[0];
}

void AX18SetTorqueEnable(unsigned char id, unsigned char torqueEnable) {
	if(torqueEnable == 0 || torqueEnable == 1)
		AX18FWrite(id, AX_TORQUE_ENABLE, &torqueEnable, 1);
}

unsigned char AX18GetTorqueEnable(unsigned char id) {
	unsigned char buffer[1];
	if(AX18FRead(id, AX_TORQUE_ENABLE, buffer, 1) == 0)
		return 0;
	return buffer[0];
}

void AX18SetID(unsigned char id, unsigned char newId) {
		AX18FWrite(id, AX_ID, &newId, 1);
}

unsigned char AX18GetId(unsigned char id) {
	unsigned char buffer[1];
	if(AX18FRead(id, AX_ID, buffer, 1) == 0)
		return 0;
	return buffer[0];
}

void AX18SetReturnDelayTime(unsigned char id, unsigned long delay) {
	unsigned char data = delay / 2;
	AX18FWrite(id, AX_RETURN_DELAY_TIME, &data, 1);
}

void AX18SetBaudRate(unsigned char id, unsigned long baudRate) {
	unsigned char data = 2000000/baudRate-1;
	AX18FWrite(id, AX_BAUD_RATE, &data, 1);
}

unsigned long AX18GetBaudRate(unsigned char id) {
	unsigned char buffer[1];
	if(AX18FRead(id, AX_BAUD_RATE, buffer, 1) == 0)
		return 0;
	return 2000000/(buffer[0] + 1);
}

void AX18SetAngleLimitCW(unsigned char id, unsigned long limit) {
	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(limit),
		unsigned16ToUnsigned8Higher(limit)};

	AX18FWrite(id, AX_CW_ANGLE_LIMIT_L, buffer, 2);
}

void AX18SetAngleLimitCCW(unsigned char id, unsigned long limit) {
	unsigned char buffer[2] = {
		unsigned16ToUnsigned8Lower(limit),
		unsigned16ToUnsigned8Higher(limit)};

	AX18FWrite(id, AX_CCW_ANGLE_LIMIT_L, buffer, 2);
}

/**
 * Returns lower byte of signed 16 bit value
 * @param  data 16 bit signed value
 * @return      lower byte
 */
char signed16ToSigned8Lower(long data) {
	return data & 0xFF;
}

/**
 * Returns higher byte of signed 16 bit value
 * @param  data 16 bit signed value
 * @return      higher byte
 */
char signed16ToSigned8Higher(long data) {
	return (data >> 8) & 0xFF;
}

/**
 * Returns lower byte of unsigned 16 bit value
 * @param  data 16 bit unsigned value
 * @return      lower byte
 */
unsigned char unsigned16ToUnsigned8Lower(unsigned long data) {
	return data & 0xFF;
}

/**
 * Returns higher byte of unsigned 16 bit value
 * @param  data 16 bit unsigned value
 * @return      higher byte
 */
unsigned char unsigned16ToUnsigned8Higher(unsigned long data) {
	return (data >> 8) & 0xFF;
}

unsigned long unsigned8ToUnsigned16(unsigned char lower, unsigned char higher) {
	return (higher << 8) | lower;
}

signed long signed8Tosigned16(unsigned char lower, unsigned char higher) {
	return (higher << 8) | lower;
}
