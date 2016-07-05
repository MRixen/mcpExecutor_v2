#include <SPI.h>


const int CS_PIN_ADXL = 7;
const int CS_PIN_MCP2515 = 8;
const byte EXECUTOR_ID = 0x00;

// CONTROL REGISTER
const byte CONTROL_REGISTER_BFPCTRL = 0x0C;
const byte CONTROL_REGISTER_TXRTSCTRL = 0x0D;
const byte CONTROL_REGISTER_CANSTAT = 0x0E;
const byte CONTROL_REGISTER_CANCTRL = 0x0F;
const byte CONTROL_REGISTER_TEC = 0x1C;
const byte CONTROL_REGISTER_REC = 0x1D;
const byte CONTROL_REGISTER_CNF1 = 0x2A;
const byte CONTROL_REGISTER_CNF2 = 0x29;
const byte CONTROL_REGISTER_CNF3 = 0x28;
const byte CONTROL_REGISTER_CANINTE = 0x2B;
const byte CONTROL_REGISTER_CANINTF = 0x2C;
const byte CONTROL_REGISTER_EFLG = 0x2D;
const byte CONTROL_REGISTER_TXB0CTRL = 0x30;
const byte CONTROL_REGISTER_TXB1CTRL = 0x40;
const byte CONTROL_REGISTER_TXB2CTRL = 0x50;
const byte CONTROL_REGISTER_RXB0CTRL = 0x60;
const byte CONTROL_REGISTER_RXB1CTRL = 0x70;

// REGISTER MCP2515
const byte REGISTER_TXB0SIDH = 0x31;
const byte REGISTER_TXB0SIDL = 0x32;
const byte REGISTER_TXB0DLC = 0x35;
const byte REGISTER_TXB0D0 = 0x36;
const byte REGISTER_TXB0D1 = 0x37;
const byte REGISTER_TXB0D2 = 0x38;
const byte REGISTER_TXB0D3 = 0x39;
const byte REGISTER_TXB0D4 = 0x3A;
const byte REGISTER_TXB0D5 = 0x3B;
const byte REGISTER_TXB0D6 = 0x3C;
const byte REGISTER_TXB0D7 = 0x3D;
const byte REGISTER_TXB0Dx[] = { REGISTER_TXB0D0, REGISTER_TXB0D1, REGISTER_TXB0D2, REGISTER_TXB0D3, REGISTER_TXB0D4, REGISTER_TXB0D5, REGISTER_TXB0D6, REGISTER_TXB0D7 };

const byte REGISTER_TXB1SIDH = 0x41;
const byte REGISTER_TXB1SIDL = 0x42;
const byte REGISTER_TXB1DLC = 0x45;
const byte REGISTER_TXB1D0 = 0x46;
const byte REGISTER_TXB1D1 = 0x47;
const byte REGISTER_TXB1D2 = 0x48;
const byte REGISTER_TXB1D3 = 0x49;
const byte REGISTER_TXB1D4 = 0x4A;
const byte REGISTER_TXB1D5 = 0x4B;
const byte REGISTER_TXB1D6 = 0x4C;
const byte REGISTER_TXB1D7 = 0x4D;
const byte REGISTER_TXB1Dx[] = { REGISTER_TXB1D0, REGISTER_TXB1D1, REGISTER_TXB1D2, REGISTER_TXB1D3, REGISTER_TXB1D4, REGISTER_TXB1D5, REGISTER_TXB1D6, REGISTER_TXB1D7 };

const byte REGISTER_TXB2SIDH = 0x51;
const byte REGISTER_TXB2SIDL = 0x52;
const byte REGISTER_TXB2DLC = 0x55;
const byte REGISTER_TXB2D0 = 0x56;
const byte REGISTER_TXB2D1 = 0x57;
const byte REGISTER_TXB2D2 = 0x58;
const byte REGISTER_TXB2D3 = 0x59;
const byte REGISTER_TXB2D4 = 0x5A;
const byte REGISTER_TXB2D5 = 0x5B;
const byte REGISTER_TXB2D6 = 0x5C;
const byte REGISTER_TXB2D7 = 0x5D;
const byte REGISTER_TXB2Dx[] = { REGISTER_TXB2D0, REGISTER_TXB2D1, REGISTER_TXB2D2, REGISTER_TXB2D3, REGISTER_TXB2D4, REGISTER_TXB2D5, REGISTER_TXB2D6, REGISTER_TXB2D7 };

const byte REGISTER_RXB0D0 = 0x66;
const byte REGISTER_RXB0D1 = 0x67;
const byte REGISTER_RXB0D2 = 0x68;
const byte REGISTER_RXB0D3 = 0x69;
const byte REGISTER_RXB0D4 = 0x6A;
const byte REGISTER_RXB0D5 = 0x6B;
const byte REGISTER_RXB0D6 = 0x6C;
const byte REGISTER_RXB0D7 = 0x6D;
const byte REGISTER_RXB0Dx[] = { REGISTER_RXB0D0, REGISTER_RXB0D1, REGISTER_RXB0D2, REGISTER_RXB0D3, REGISTER_RXB0D4, REGISTER_RXB0D5, REGISTER_RXB0D6, REGISTER_RXB0D7 };
const byte REGISTER_RXB1D0 = 0x76;
const byte REGISTER_RXB1D1 = 0x77;
const byte REGISTER_RXB1D2 = 0x78;
const byte REGISTER_RXB1D3 = 0x79;
const byte REGISTER_RXB1D4 = 0x7A;
const byte REGISTER_RXB1D5 = 0x7B;
const byte REGISTER_RXB1D6 = 0x7C;
const byte REGISTER_RXB1D7 = 0x7D;
const byte REGISTER_RXB1Dx[] = { REGISTER_RXB1D0, REGISTER_RXB1D1, REGISTER_RXB1D2, REGISTER_RXB1D3, REGISTER_RXB1D4, REGISTER_RXB1D5, REGISTER_RXB1D6, REGISTER_RXB1D7 };
const byte CONTROL_REGISTER_CANSTAT_NORMAL_MODE = 0x00;
const byte CONTROL_REGISTER_CANSTAT_SLEEP_MODE = 0x20;
const byte CONTROL_REGISTER_CANSTAT_LOOPBACK_MODE = 0x40;
const byte CONTROL_REGISTER_CANSTAT_LISTEN_ONLY_MODE = 0x60;
const byte CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE = 0x80;
const byte CONTROL_REGISTER_CANINTE_VALUE = 0x00;
const byte CONTROL_REGISTER_TXB0CTRL_VALUE = 0x00;
const byte CONTROL_REGISTER_TXB1CTRL_VALUE = 0x00;
const byte CONTROL_REGISTER_TXB2CTRL_VALUE = 0x00;
const byte CONTROL_REGISTER_RXB0CTRL_VALUE = 0x03;
const byte CONTROL_REGISTER_RXB1CTRL_VALUE = 0x03;

const byte CONTROL_REGISTER_VALUE_CNF1 = 0x03; // Baud rate prescaler calculated with application (Fosc = 8Mhz and CANspeed = 125kHz)
const byte CONTROL_REGISTER_VALUE_CNF2 = 0x90; // BTLMODE = 1 (PHaseSegment 2 is configured with CNF 3) and PhaseSegment 1 = 8xTQ (7+1)
const byte CONTROL_REGISTER_VALUE_CNF3 = 0x02; // Set PhaseSegment 2 = 6xTQ (5+1)

const byte CONTROL_REGISTER_CANCTRL_NORMAL_MODE = 0x00;
const byte CONTROL_REGISTER_CANCTRL_SLEEP_MODE = 0x20;
const byte CONTROL_REGISTER_CANCTRL_LOOPBACK_MODE = 0x40;
const byte CONTROL_REGISTER_CANCTRL_LISTEN_ONLY_MODE = 0x60;
const byte CONTROL_REGISTER_CANCTRL_CONFIGURATION_MODE = 0x80;

// Set values for interrupt flags mcp2515
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_ALL_IF = 0x00;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_MERRIF = 0x7F;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_WAKIF = 0xBF;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_ERRIF = 0xDF;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_TX2IF = 0xEF;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_TX1IF = 0xF7;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_TX0IF = 0xFB;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_RX1IF = 0xFD;
const byte CONTROL_REGISTER_CANINTF_VALUE_RESET_RX0IF = 0xFE;

// SPI INSTRUCTIONS MCP2515
const byte SPI_INSTRUCTION_RESET = 0xC0;
const byte SPI_INSTRUCTION_READ = 0x03;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0_SIDH = 0x90;
const byte SPI_INSTRUCTION_READ_RX_BUFFER0_D0 = 0x92;
const byte SPI_INSTRUCTION_WRITE = 0x02;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_ID = 0x40;
const byte SPI_INSTRUCTION_LOAD_TX_BUFFER0_DATA = 0x41;
const byte SPI_INSTRUCTION_RTS_BUFFER0 = 0x81;
const byte SPI_INSTRUCTION_RTS_BUFFER1 = 0x82;
const byte SPI_INSTRUCTION_RTS_BUFFER2 = 0x84;
const byte SPI_INSTRUCTION_READ_STATUS = 0xA0;
const byte SPI_INSTRUCTION_RX_STATUS = 0xB0;
const byte SPI_INSTRUCTION_BIT_MODIFY = 0x05;

// REGISTER ADXL
const byte ACCEL_REG_POWER_CONTROL = 0x2D;  /* Address of the Power Control register                */
const byte ACCEL_REG_DATA_FORMAT = 0x31;    /* Address of the Data Format register                  */
const byte ACCEL_REG_X = 0x32; /* Address of the X Axis data register                  */
const byte ACCEL_REG_Y = 0x34; /* Address of the Y Axis data register                  */
const byte ACCEL_REG_Z = 0x36; /* Address of the Z Axis data register                  */
const byte ACCEL_SPI_RW_BIT = 0x80; /* Bit used in SPI transactions to indicate read/write  */
const byte ACCEL_SPI_MB_BIT = 0x40; /* Bit used to indicate multi-byte SPI transactions     */
const byte MESSAGE_SIZE_ADXL = 0x07;

// DATA FOR ADXL
const int ACCEL_RES = 1024;         /* The ADXL345 has 10 bit resolution giving 1024 unique values                     */
const int ACCEL_DYN_RANGE_G = 8;    /* The ADXL345 had a total dynamic range of 8G, since we're configuring it to +-4G */
const int UNITS_PER_G = ACCEL_RES / ACCEL_DYN_RANGE_G;  /* Ratio of raw int values to G units
														*/

														// GLOBAL DATA
const long MAX_WAIT_TIME = 5000;
const int MAX_ERROR_COUNTER_MODESWITCH = 3;
bool stopAllOperations = false;
bool debugMode;
bool currentTxBuffer[] = { true, false, false };

union acc_x
{
	short accelerationRawX;
	byte bytes[2];
};
union acc_y
{
	short accelerationRawY;
	byte bytes[2];
};
union acc_z
{
	short accelerationRawZ;
	byte bytes[2];
};

struct Acceleration
{
	double accelX;
	double accelY;
	double accelZ;
};

double sensorData[3];

byte ReadBuf[7]; // Read buffer of size 6 bytes (2 bytes * 3 axes)

const int ADXL = 1;
const int MCP2515 = 2;
const int NO_DEVICE = 3;
long errorTimerValue;
long startTimer;
bool firstStart;
bool startSequenceIsActive;

byte messageData[] = { 0, 0 };
int counter = 0;
long startTime;

void setup()
{
	// Init variables
	firstStart = true;

	// USER CONFIGURATION
	debugMode = false;

	// Define I/Os
	pinMode(CS_PIN_ADXL, OUTPUT);
	pinMode(CS_PIN_MCP2515, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	pinMode(10, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9

	// Configure I/Os
	digitalWrite(CS_PIN_ADXL, HIGH);
	digitalWrite(CS_PIN_MCP2515, HIGH);
	digitalWrite(10, HIGH);

	// Configure SPI
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
	SPI.begin();

	// Configure ADXL and MCP2515
	initAdxl();
	initMcp2515();
	mcp2515_init_tx_buffer0(EXECUTOR_ID, 0x00, MESSAGE_SIZE_ADXL);
	mcp2515_init_tx_buffer1(EXECUTOR_ID, 0x00, MESSAGE_SIZE_ADXL);
	mcp2515_init_tx_buffer2(EXECUTOR_ID, 0x00, MESSAGE_SIZE_ADXL);

	// Give time to set up
	delay(100);

	// Start timer to measure the program execution
	errorTimerValue = millis();

	startTime = millis();


}

void loop()
{
	getAdxlData();

	if (currentTxBuffer[0])
	{
		for (size_t i = 0; i < 7; i++) mcp2515_load_tx_buffer(ReadBuf[i], REGISTER_TXB0Dx[i], 0);
		currentTxBuffer[0] = false;
		currentTxBuffer[1] = true;
	}
	else if (currentTxBuffer[1]) {
		for (size_t i = 0; i < 7; i++) mcp2515_load_tx_buffer(ReadBuf[i], REGISTER_TXB1Dx[i], 1);
		currentTxBuffer[1] = false;
		currentTxBuffer[2] = true;
	}
	else if (currentTxBuffer[2]) {
		for (size_t i = 0; i < 7; i++) mcp2515_load_tx_buffer(ReadBuf[i], REGISTER_TXB2Dx[i], 2);
		currentTxBuffer[2] = false;
		currentTxBuffer[0] = true;
	}
	//Serial.print("--- CANSTAT---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515));
	//Serial.print("--- CANCTRL---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_CANCTRL, CS_PIN_MCP2515));
	//Serial.print("--- TEC---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_TEC, CS_PIN_MCP2515));
	//Serial.print("--- REC---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_REC, CS_PIN_MCP2515));
	//Serial.print("--- EFLG---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_EFLG, CS_PIN_MCP2515));	
	//Serial.print("--- TXB0CTRL---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_TXB0CTRL, CS_PIN_MCP2515));
	//Serial.print("--- TXB1CTRL---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_TXB1CTRL, CS_PIN_MCP2515));
	//Serial.print("--- TXB2CTRL---");
	//Serial.println(mcp2515_execute_read_command(CONTROL_REGISTER_TXB2CTRL, CS_PIN_MCP2515));
	//Serial.println("------");
	delay(100);
	
}

void initAdxl() {
	writeToSpi(ACCEL_REG_DATA_FORMAT, 0x01, CS_PIN_ADXL);
	writeToSpi(ACCEL_REG_POWER_CONTROL, 0x08, CS_PIN_ADXL);
}

void initMcp2515() {

	delay(100);

	// Reset chip to set in operation mode
	mcp2515_execute_reset_command();
	delay(100);

	// Configure bit timing
	mcp2515_configureCanBus();
	delay(100);

	// Configure interrupts
	mcp2515_configureInterrupts();
	delay(100);

	// Configure bit masks and filters that we can receive everything 
	mcp2515_configureMasksFilters();
	delay(100);

	// Set device to normal mode
	mcp2515_switchMode(CONTROL_REGISTER_CANSTAT_NORMAL_MODE, CONTROL_REGISTER_CANCTRL_NORMAL_MODE);
	delay(100);

	// Set osm
	mcp2515_execute_write_command(CONTROL_REGISTER_CANCTRL, 0x08, CS_PIN_MCP2515);
	delay(100);

	//// Reset all failures
	//mcp2515_execute_write_command(CONTROL_REGISTER_CANINTF, CONTROL_REGISTER_CANINTF_VALUE_RESET_ALL_IF, CS_PIN_MCP2515);
	//delay(100);
}

void mcp2515_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, CS_PIN_MCP2515);

	// Read the register value
	byte actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	while (CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE != (CONTROL_REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
	}

	if (debugMode) Serial.print("Mcp2515 reset succesfully and switch do mode ");
	if (debugMode) Serial.println(actualMode);
}

void mcp2515_configureCanBus() {
	// Configure bit timing

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF1, CONTROL_REGISTER_VALUE_CNF1, CS_PIN_MCP2515);

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF2, CONTROL_REGISTER_VALUE_CNF2, CS_PIN_MCP2515);

	mcp2515_execute_write_command(CONTROL_REGISTER_CNF3, CONTROL_REGISTER_VALUE_CNF3, CS_PIN_MCP2515);

	if (debugMode) Serial.println("Mcp2515 configure bus succesfully");
}

void mcp2515_configureInterrupts() {
	mcp2515_execute_write_command(CONTROL_REGISTER_CANINTE, CONTROL_REGISTER_CANINTE_VALUE, CS_PIN_MCP2515);

	if (debugMode) Serial.println("Mcp2515 configure interrupts succesfully");
}

void mcp2515_configureMasksFilters() {

	// Set parameters for rx buffer 0
	mcp2515_execute_write_command(CONTROL_REGISTER_RXB0CTRL, CONTROL_REGISTER_RXB0CTRL_VALUE, CS_PIN_MCP2515);

	// Set parameters for rx buffer 1
	mcp2515_execute_write_command(CONTROL_REGISTER_RXB1CTRL, CONTROL_REGISTER_RXB1CTRL_VALUE, CS_PIN_MCP2515);

	if (debugMode) Serial.println("Mcp2515 configure masks / filters succesfully");
}

void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch) {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];
	byte actualMode;

	// Repeat mode switching for a specific time when the first trial is without success
	for (int i = 0; i < MAX_ERROR_COUNTER_MODESWITCH; i++)
	{
		mcp2515_execute_write_command(CONTROL_REGISTER_CANCTRL, modeToSwitch, CS_PIN_MCP2515);

		// Read the register value
		actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
		long elapsedTime = 0;
		long errorTimerValue = millis();
		while ((actualMode != modeToSwitch) && (elapsedTime <= MAX_WAIT_TIME))
		{
			actualMode = mcp2515_execute_read_command(CONTROL_REGISTER_CANSTAT, CS_PIN_MCP2515);
			elapsedTime = millis() - errorTimerValue; // Stop time to break loop when mode isnt switching
		}
		if (elapsedTime > MAX_WAIT_TIME)
		{
			if (debugMode) Serial.println("Abort waiting. Max. waiting time reached.");
			if (i == MAX_ERROR_COUNTER_MODESWITCH)
			{
				if (debugMode) Serial.println("ERROR MODE SWITCH - STOP ALL OPERATIONS");
				stopAllOperations = true;
			}
		}
		else break;
	}

	if (debugMode) Serial.print("Mcp2515 switch to mode ");
	if (debugMode) Serial.print(actualMode);
	if (debugMode) Serial.println(" succesfully");
}

byte mcp2515_execute_read_command(byte registerToRead, int cs_pin)
{
	byte returnMessage;

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ);

	// Write the address of the register to read
	SPI.transfer(registerToRead);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	delay(1);

	mcp2515_execute_write_command(CONTROL_REGISTER_CANINTF, CONTROL_REGISTER_CANINTF_VALUE_RESET_ALL_IF, CS_PIN_MCP2515);

	return returnMessage;
}

byte mcp2515_execute_read_state_command(int cs_pin)
{
	byte returnMessage;

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ_STATUS);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	return returnMessage;
}

bool getAdxlData() {
	

	byte RegAddrBuf[] = { ACCEL_REG_X | ACCEL_SPI_RW_BIT | ACCEL_SPI_MB_BIT };

	setCsPin(CS_PIN_ADXL, LOW);
	SPI.transfer(RegAddrBuf[0]);
	for (int i = 0; i < 6; i++) ReadBuf[i] = SPI.transfer(0x00); // Write 0 value to get data
	setCsPin(CS_PIN_ADXL, HIGH);

	ReadBuf[6] = EXECUTOR_ID;
	//Serial.println(millis() - startTime);


	if(!debugMode) return true;
	else {

		acc_x sensorDataX;
		sensorDataX.bytes[0] = ReadBuf[0];
		sensorDataX.bytes[1] = ReadBuf[1];

		acc_y sensorDataY;
		sensorDataY.bytes[0] = ReadBuf[2];
		sensorDataY.bytes[1] = ReadBuf[3];

		acc_z sensorDataZ;
		sensorDataZ.bytes[0] = ReadBuf[4];
		sensorDataZ.bytes[1] = ReadBuf[5];

		sensorData[0] = (double)sensorDataX.accelerationRawX / UNITS_PER_G;
		sensorData[1] = (double)sensorDataY.accelerationRawY / UNITS_PER_G;
		sensorData[2] = (double)sensorDataZ.accelerationRawZ / UNITS_PER_G;

		Serial.println(sensorData[0]);
		Serial.println(sensorData[1]);
		Serial.println(sensorData[2]);
		return true;
	}
}

void mcp2515_load_tx_buffer(byte messageData, byte registerAddress, int txBufferId) {
	mcp2515_execute_write_command(registerAddress, messageData, CS_PIN_MCP2515);

	// Send message
	mcp2515_execute_rts_command(txBufferId);

	delay(1);
}

void mcp2515_init_tx_buffer0(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB0SIDL, identifierLow, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0SIDH, identifierHigh, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB0DLC, messageSize, CS_PIN_MCP2515);
}

void mcp2515_init_tx_buffer1(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB1SIDL, identifierLow, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB1SIDH, identifierHigh, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB1DLC, messageSize, CS_PIN_MCP2515);
}

void mcp2515_init_tx_buffer2(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(REGISTER_TXB2SIDL, identifierLow, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB2SIDH, identifierHigh, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(REGISTER_TXB2DLC, messageSize, CS_PIN_MCP2515);
}

void writeToSpi(byte address, byte data, int cs_pin) {
	byte spiMessage[] = { address, data };

	setCsPin(cs_pin, LOW);
	for (int i = 0; i < 2; i++) SPI.transfer(spiMessage[i]);
	setCsPin(cs_pin, HIGH);
}

void setCsPin(int cs_pin, uint8_t value) {
	digitalWrite(10, value);
	digitalWrite(cs_pin, value);
}

void mcp2515_execute_write_command(byte address, byte data, int cs_pin)
{
	//long startTime = millis();

	setCsPin(cs_pin, LOW); // Enable device
	SPI.transfer(SPI_INSTRUCTION_WRITE); // Write spi instruction write  
	SPI.transfer(address);
	SPI.transfer(data);
	setCsPin(cs_pin, HIGH); // Disable device

	//Serial.println("mcp2515_execute_write_command: ");
	//Serial.println(millis() - startTime);
}

void mcp2515_execute_rts_command(int bufferId)
{
	byte spiMessage[1];

	switch (bufferId)
	{
	case 0:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER0;
		break;
	case 1:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER1;
		break;
	case 2:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER2;
		break;
	default:
		break;
	}
	writeSimpleCommandSpi(spiMessage[0], CS_PIN_MCP2515);
}

void writeSimpleCommandSpi(byte command, int cs_pin)
{
	setCsPin(cs_pin, LOW);
	SPI.transfer(command);
	setCsPin(cs_pin, HIGH);
}

