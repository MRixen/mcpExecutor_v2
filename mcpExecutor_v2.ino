#include <SPI.h>

// USER DEFINED
const int SLOW_DOWN_CODE = 2;

// ---- MCP SETUP BEGIN----

// DEVICE 0
//const byte EXECUTOR_ID = 0x00;
//const byte REGISTER_TXB0SIDL_VALUE = 0x20;
//const byte REGISTER_TXB0SIDH_VALUE = 0x01;
//const byte REGISTER_TXB1SIDL_VALUE = 0x20;
//const byte REGISTER_TXB1SIDH_VALUE = 0x01;
//const byte REGISTER_TXB2SIDL_VALUE = 0x20;
//const byte REGISTER_TXB2SIDH_VALUE = 0x01;

// DEVICE 1
const byte EXECUTOR_ID = 0x01;
const byte REGISTER_TXB0SIDL_VALUE = 0x40;
const byte REGISTER_TXB0SIDH_VALUE = 0x02;
const byte REGISTER_TXB1SIDL_VALUE = 0x40;
const byte REGISTER_TXB1SIDH_VALUE = 0x02;
const byte REGISTER_TXB2SIDL_VALUE = 0x40;
const byte REGISTER_TXB2SIDH_VALUE = 0x02;

// ---- MCP SETUP END ----


// REGISTER MCP2515
const byte REGISTER_BFPCTRL = 0x0C;
const byte REGISTER_TXRTSCTRL = 0x0D;
const byte REGISTER_CANSTAT = 0x0E;
const byte REGISTER_CANCTRL = 0x0F;
const byte REGISTER_TEC = 0x1C;
const byte REGISTER_REC = 0x1D;
const byte REGISTER_CNF1 = 0x2A;
const byte REGISTER_CNF2 = 0x29;
const byte REGISTER_CNF3 = 0x28;
const byte REGISTER_CANINTE = 0x2B;
const byte REGISTER_CANINTF = 0x2C;
const byte REGISTER_EFLG = 0x2D;
const byte REGISTER_TXB0CTRL = 0x30;
const byte REGISTER_TXB1CTRL = 0x40;
const byte REGISTER_TXB2CTRL = 0x50;
const byte REGISTER_RXB0CTRL = 0x60;
const byte REGISTER_RXB1CTRL = 0x70;

const byte REGISTER_RXF0SIDL = 0x01;
const byte REGISTER_RXF1SIDL = 0x05;
const byte REGISTER_RXF2SIDL = 0x09;
const byte REGISTER_RXF3SIDL = 0x11;
const byte REGISTER_RXF4SIDL = 0x15;
const byte REGISTER_RXF5SIDL = 0x19;
const byte REGISTER_RXFxSIDL[] = { REGISTER_RXF0SIDL, REGISTER_RXF1SIDL ,REGISTER_RXF2SIDL ,REGISTER_RXF3SIDL ,REGISTER_RXF4SIDL, REGISTER_RXF5SIDL };

const byte REGISTER_RXF0SIDH = 0x00;
const byte REGISTER_RXF1SIDH = 0x04;
const byte REGISTER_RXF2SIDH = 0x08;
const byte REGISTER_RXF3SIDH = 0x10;
const byte REGISTER_RXF4SIDH = 0x14;
const byte REGISTER_RXF5SIDH = 0x18;
const byte REGISTER_RXFxSIDH[] = { REGISTER_RXF0SIDH, REGISTER_RXF1SIDH ,REGISTER_RXF2SIDH ,REGISTER_RXF3SIDH ,REGISTER_RXF4SIDH, REGISTER_RXF5SIDH };

const byte REGISTER_RXM0SIDL = 0x21;
const byte REGISTER_RXM1SIDL = 0x25;
const byte REGISTER_RXMxSIDL[] = { REGISTER_RXM0SIDL, REGISTER_RXM1SIDL };

const byte REGISTER_RXM0SIDH = 0x20;
const byte REGISTER_RXM1SIDH = 0x24;
const byte REGISTER_RXMxSIDH[] = { REGISTER_RXM0SIDH, REGISTER_RXM1SIDH };

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
const byte REGISTER_CANSTAT_NORMAL_MODE = 0x00;
const byte REGISTER_CANSTAT_SLEEP_MODE = 0x20;
const byte REGISTER_CANSTAT_LOOPBACK_MODE = 0x40;
const byte REGISTER_CANSTAT_LISTEN_ONLY_MODE = 0x60;
const byte REGISTER_CANSTAT_CONFIGURATION_MODE = 0x80;
const byte REGISTER_CANCTRL_NORMAL_MODE = 0x00;
const byte REGISTER_CANCTRL_SLEEP_MODE = 0x20;
const byte REGISTER_CANCTRL_LOOPBACK_MODE = 0x40;
const byte REGISTER_CANCTRL_LISTEN_ONLY_MODE = 0x60;
const byte REGISTER_CANCTRL_CONFIGURATION_MODE = 0x80;

// REGISTER VALUES / DATA MCP2515
const byte REGISTER_CANINTF_VALUE_RESET_ALL_IF = 0x00;
const byte REGISTER_CANINTF_VALUE_RESET_MERRIF = 0x7F;
const byte REGISTER_CANINTF_VALUE_RESET_WAKIF = 0xBF;
const byte REGISTER_CANINTF_VALUE_RESET_ERRIF = 0xDF;
const byte REGISTER_CANINTF_VALUE_RESET_TX2IF = 0xEF;
const byte REGISTER_CANINTF_VALUE_RESET_TX1IF = 0xF7;
const byte REGISTER_CANINTF_VALUE_RESET_TX0IF = 0xFB;
const byte REGISTER_CANINTF_VALUE_RESET_RX1IF = 0xFD;
const byte REGISTER_CANINTF_VALUE_RESET_RX0IF = 0xFE;
const byte REGISTER_CANINTE_VALUE = 0x00;
const byte REGISTER_TXB0CTRL_VALUE = 0x00;
const byte REGISTER_TXB1CTRL_VALUE = 0x00;
const byte REGISTER_TXB2CTRL_VALUE = 0x00;
const byte REGISTER_RXB0CTRL_VALUE = 0x21; // Receive messages with standard identifier that meet filter criteria (filter 1)
const byte REGISTER_RXB1CTRL_VALUE = 0x22;  // Receive messages with standard identifier that meet filter criteria (filter 2)

const byte REGISTER_RXF0SIDL_VALUE = 0x01;
const byte REGISTER_RXF1SIDL_VALUE = 0xFF;
const byte REGISTER_RXF2SIDL_VALUE = 0xFF;
const byte REGISTER_RXF3SIDL_VALUE = 0xFF;
const byte REGISTER_RXF4SIDL_VALUE = 0xFF;
const byte REGISTER_RXF5SIDL_VALUE = 0xFF;
const byte REGISTER_RXFxSIDL_VALUE[] = { REGISTER_RXF0SIDL_VALUE, REGISTER_RXF1SIDL_VALUE , REGISTER_RXF2SIDL_VALUE , REGISTER_RXF3SIDL_VALUE , REGISTER_RXF4SIDL_VALUE , REGISTER_RXF5SIDL_VALUE };

const byte REGISTER_RXF0SIDH_VALUE = 0xFF;
const byte REGISTER_RXF1SIDH_VALUE = 0xFF;
const byte REGISTER_RXF2SIDH_VALUE = 0xFF;
const byte REGISTER_RXF3SIDH_VALUE = 0xFF;
const byte REGISTER_RXF4SIDH_VALUE = 0xFF;
const byte REGISTER_RXF5SIDH_VALUE = 0xFF;
const byte REGISTER_RXFxSIDH_VALUE[] = { REGISTER_RXF0SIDH_VALUE, REGISTER_RXF1SIDH_VALUE , REGISTER_RXF2SIDH_VALUE , REGISTER_RXF3SIDH_VALUE , REGISTER_RXF4SIDH_VALUE , REGISTER_RXF5SIDH_VALUE };

const byte REGISTER_RXM0SIDL_VALUE = 0xE0;
const byte REGISTER_RXM1SIDL_VALUE = 0xE0;
const byte REGISTER_RXMxSIDL_VALUE[] = { REGISTER_RXM0SIDL_VALUE, REGISTER_RXM1SIDL_VALUE };

const byte REGISTER_RXM0SIDH_VALUE = 0xFF;
const byte REGISTER_RXM1SIDH_VALUE = 0xFF;
const byte REGISTER_RXMxSIDH_VALUE[] = { REGISTER_RXM0SIDH_VALUE, REGISTER_RXM1SIDH_VALUE };

const byte REGISTER_TXBxSIDL_VALUE[] = { REGISTER_TXB0SIDL_VALUE, REGISTER_TXB1SIDL_VALUE, REGISTER_TXB2SIDL_VALUE };
const byte REGISTER_TXBxSIDH_VALUE[] = { REGISTER_TXB0SIDH_VALUE, REGISTER_TXB1SIDH_VALUE, REGISTER_TXB2SIDH_VALUE };

const byte REGISTER_CNF1_VALUE = 0x03; // Baud rate prescaler calculated with application (Fosc = 8Mhz and CANspeed = 125kHz)
const byte REGISTER_CNF2_VALUE = 0x90; // BTLMODE = 1 (PHaseSegment 2 is configured with CNF 3) and PhaseSegment 1 = 8xTQ (7+1)
const byte REGISTER_CNF3_VALUE = 0x02; // Set PhaseSegment 2 = 6xTQ (5+1)
const int CS_PIN_MCP2515 = 8;
bool currentTxBuffer[] = { true, false, false };
const byte MESSAGE_SIZE_ADXL = 0x07;

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
const byte REGISTER_ACCEL_REG_POWER_CONTROL = 0x2D;  /* Address of the Power Control register                */
const byte REGISTER_ACCEL_REG_DATA_FORMAT = 0x31;    /* Address of the Data Format register                  */
const byte REGISTER_ACCEL_REG_X = 0x32; /* Address of the X Axis data register                  */
const byte REGISTER_ACCEL_REG_Y = 0x34; /* Address of the Y Axis data register                  */
const byte REGISTER_ACCEL_REG_Z = 0x36; /* Address of the Z Axis data register                  */
const byte REGISTER_ACCEL_SPI_RW_BIT = 0x80; /* Bit used in SPI transactions to indicate read/write  */
const byte REGISTER_ACCEL_SPI_MB_BIT = 0x40; /* Bit used to indicate multi-byte SPI transactions     */


// REGISTER VALUE / DATA ADXL
const int REGISTER_ACCEL_RES_VALUE = 1024;         // The ADXL345 has 10 bit resolution giving 1024 unique values
const int REGISTER_ACCEL_DYN_RANGE_G_VALUE = 8;    // The ADXL345 had a total dynamic range of 8G, since we're configuring it to +-4G
const int UNITS_PER_G = REGISTER_ACCEL_RES_VALUE / REGISTER_ACCEL_DYN_RANGE_G_VALUE;  // Ratio of raw int values to G units
const int CS_PIN_ADXL = 7;
byte ReadBuf[7]; // Read buffer of size 6 bytes (2 bytes * 3 axes)
double sensorData[3];

// GLOBAL DATA
const long MAX_WAIT_TIME = 5000;
const int MAX_ERROR_COUNTER_MODESWITCH = 3;
bool stopAllOperations = false;
bool debugMode;
const int ADXL = 1;
const int MCP2515 = 2;
const int NO_DEVICE = 3;
long errorTimerValue;

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

void setup()
{
	// Configure serial interface
	Serial.begin(9600);

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

	// Set identifier, message length, etc.
	mcp2515_init_tx_buffer0(REGISTER_TXBxSIDL_VALUE[0], REGISTER_TXBxSIDH_VALUE[0], MESSAGE_SIZE_ADXL);
	mcp2515_init_tx_buffer1(REGISTER_TXBxSIDL_VALUE[1], REGISTER_TXBxSIDH_VALUE[1], MESSAGE_SIZE_ADXL);
	mcp2515_init_tx_buffer2(REGISTER_TXBxSIDL_VALUE[2], REGISTER_TXBxSIDH_VALUE[2], MESSAGE_SIZE_ADXL);

	// Start timer to measure the program execution
	errorTimerValue = millis();

	// Give time to set up
	delay(100);
}

void loop()
{
	getAdxlData();
	for (size_t i = 0; i < MESSAGE_SIZE_ADXL; i++) mcp2515_load_tx_buffer(ReadBuf[i], i, MESSAGE_SIZE_ADXL);
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


void initAdxl() {
	writeToSpi(REGISTER_ACCEL_REG_DATA_FORMAT, 0x01, CS_PIN_ADXL);
	writeToSpi(REGISTER_ACCEL_REG_POWER_CONTROL, 0x08, CS_PIN_ADXL);
}

void initMcp2515() {
	// Reset chip to set in operation mode
	mcp2515_execute_reset_command();

	// Configure bit timing
	mcp2515_configureCanBus();

	// Configure interrupts
	mcp2515_configureInterrupts();

	// Set device to normal mode
	mcp2515_switchMode(REGISTER_CANSTAT_NORMAL_MODE, REGISTER_CANCTRL_NORMAL_MODE);
}

void mcp2515_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, CS_PIN_MCP2515);

	// Read the register value
	byte actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, CS_PIN_MCP2515);
	while (REGISTER_CANSTAT_CONFIGURATION_MODE != (REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, CS_PIN_MCP2515);
	}

	if (debugMode) Serial.print("Mcp2515 reset succesfully and switch do mode ");
	if (debugMode) Serial.println(actualMode);
}

void mcp2515_configureCanBus() {
	// Configure bit timing

	mcp2515_execute_write_command(REGISTER_CNF1, REGISTER_CNF1_VALUE, CS_PIN_MCP2515);

	mcp2515_execute_write_command(REGISTER_CNF2, REGISTER_CNF2_VALUE, CS_PIN_MCP2515);

	mcp2515_execute_write_command(REGISTER_CNF3, REGISTER_CNF3_VALUE, CS_PIN_MCP2515);

	if (debugMode) Serial.println("Mcp2515 configure bus succesfully");
}

void mcp2515_configureInterrupts() {
	mcp2515_execute_write_command(REGISTER_CANINTE, REGISTER_CANINTE_VALUE, CS_PIN_MCP2515);

	if (debugMode) Serial.println("Mcp2515 configure interrupts succesfully");
}

void mcp2515_configureMasksFilters(byte registerAddress, byte registerValue) {

	// Set parameters for rx buffer 0
	mcp2515_execute_write_command(REGISTER_RXB0CTRL, REGISTER_RXB0CTRL_VALUE, CS_PIN_MCP2515);

	// Set parameters for rx buffer 1
	mcp2515_execute_write_command(REGISTER_RXB1CTRL, REGISTER_RXB1CTRL_VALUE, CS_PIN_MCP2515);
}

void mcp2515_configureRxBuffer() {

	// Set parameters for rx buffer 0
	mcp2515_execute_write_command(0x60, 0x04, CS_PIN_MCP2515); // as dec 39: Filter 1 is enabled (rollover is active / bukt is set)

	mcp2515_execute_write_command(0x70, 0x00, CS_PIN_MCP2515); // as dec 33: Filter 1 is enabled (if bukt is set)

	if (debugMode) Serial.println("Mcp2515 configure rx buffer succesfully");
}

void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch) {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];
	byte actualMode;

	// Repeat mode switching for a specific time when the first trial is without success
	for (int i = 0; i < MAX_ERROR_COUNTER_MODESWITCH; i++)
	{
		mcp2515_execute_write_command(REGISTER_CANCTRL, modeToSwitch, CS_PIN_MCP2515);

		// Read the register value
		actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, CS_PIN_MCP2515);
		long elapsedTime = 0;
		long errorTimerValue = millis();
		while ((actualMode != modeToSwitch) && (elapsedTime <= MAX_WAIT_TIME))
		{
			actualMode = mcp2515_execute_read_command(REGISTER_CANSTAT, CS_PIN_MCP2515);
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

	delay(SLOW_DOWN_CODE);

	mcp2515_execute_write_command(REGISTER_CANINTF, REGISTER_CANINTF_VALUE_RESET_ALL_IF, CS_PIN_MCP2515);

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

	delay(SLOW_DOWN_CODE);

	return returnMessage;
}

bool getAdxlData() {


	byte RegAddrBuf[] = { REGISTER_ACCEL_REG_X | REGISTER_ACCEL_SPI_RW_BIT | REGISTER_ACCEL_SPI_MB_BIT };

	setCsPin(CS_PIN_ADXL, LOW);
	SPI.transfer(RegAddrBuf[0]);
	for (int i = 0; i < 6; i++) ReadBuf[i] = SPI.transfer(0x00); // Write 0 value to get data
	setCsPin(CS_PIN_ADXL, HIGH);

	ReadBuf[6] = EXECUTOR_ID;

	if (!debugMode) return true;
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

void mcp2515_load_tx_buffer(byte messageData, int byteNumber, int messageSize) {

	byte registerAddress;

	if (currentTxBuffer[0])
	{
		registerAddress = REGISTER_TXB0Dx[byteNumber];
		if (byteNumber == (messageSize-1))
		{
			currentTxBuffer[0] = false;
			currentTxBuffer[1] = true;
		}
	}
	else if (currentTxBuffer[1]) {
		registerAddress = REGISTER_TXB1Dx[byteNumber];
		if (byteNumber == (messageSize - 1))
		{
		currentTxBuffer[1] = false;
		currentTxBuffer[2] = true;
		}
	}
	else if (currentTxBuffer[2]) {
		registerAddress = REGISTER_TXB2Dx[byteNumber];
		if (byteNumber == (messageSize - 1))
		{
		currentTxBuffer[2] = false;
		currentTxBuffer[0] = true;
		}
	}

	mcp2515_execute_write_command(registerAddress, messageData, CS_PIN_MCP2515);

	// DEBUG
	//if(byteNumber == 6) Serial.println(mcp2515_execute_read_command(registerAddress, CS_PIN_MCP2515));

	// Send message
	for (int i = 0; i < 3; i++) if(currentTxBuffer[i] == true) mcp2515_execute_rts_command(i);	
}

void mcp2515_init_tx_buffer(byte identifierLow, byte identifierHigh, byte messageSize, int regAddr) {

	byte regAddr_sidl;
	byte regAddr_sidh;
	byte regAddr_dlc;

	switch (regAddr)
	{
	case 0:
		regAddr_sidl = REGISTER_TXB0SIDL;
		regAddr_sidh = REGISTER_TXB0SIDH;
		regAddr_dlc = REGISTER_TXB0DLC;
		break;
	case 1:
		regAddr_sidl = REGISTER_TXB1SIDL;
		regAddr_sidh = REGISTER_TXB1SIDH;
		regAddr_dlc = REGISTER_TXB1DLC;
		break;
	case 2:
		regAddr_sidl = REGISTER_TXB2SIDL;
		regAddr_sidh = REGISTER_TXB2SIDH;
		regAddr_dlc = REGISTER_TXB2DLC;
		break;
	default:
		break;
	}

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	mcp2515_execute_write_command(regAddr_sidl, identifierLow, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(regAddr_sidh, identifierHigh, CS_PIN_MCP2515);

	// Set data length and set rtr bit to zero (no remote request)
	mcp2515_execute_write_command(regAddr_dlc, messageSize, CS_PIN_MCP2515);
}

void writeToSpi(byte address, byte data, int cs_pin) {
	byte spiMessage[] = { address, data };

	setCsPin(cs_pin, LOW);
	for (int i = 0; i < 2; i++) SPI.transfer(spiMessage[i]);
	setCsPin(cs_pin, HIGH);
	delay(SLOW_DOWN_CODE);
}

void setCsPin(int cs_pin, uint8_t value) {
	digitalWrite(10, value);
	digitalWrite(cs_pin, value);
}

void mcp2515_execute_write_command(byte address, byte data, int cs_pin)
{
	setCsPin(cs_pin, LOW); // Enable device
	SPI.transfer(SPI_INSTRUCTION_WRITE); // Write spi instruction write  
	SPI.transfer(address);
	SPI.transfer(data);
	setCsPin(cs_pin, HIGH); // Disable device
	delay(SLOW_DOWN_CODE);
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
	delay(SLOW_DOWN_CODE);
}

