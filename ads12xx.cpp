#include "ads12xx.h"
// change waitforDRDY() to poll instead of interrupt

// ads12xx setup
ads12xx::ads12xx() {}

void ads12xx::begin(int CS, int START, int DRDY, int _RESET) {
	pinMode(CS, OUTPUT);              // set the slaveSelectPin as an output:
	digitalWriteFast(CS, HIGH); // CS HIGH = nothing selected
	pinMode(DRDY, INPUT);             // DRDY read
	pinMode(START, OUTPUT);
	digitalWriteFast(START, HIGH);
	pinMode(RESET, OUTPUT);              // set the slaveSelectPin as an output:
	digitalWriteFast(RESET, HIGH);
	_CS = CS;
	_DRDY = DRDY;
	delay(500);
	SPI.begin();

	// attachInterrupt(_DRDY, DRDY_Interrupt, FALLING); //Interrupt setup for DRDY detection

	delay(500);
}

bool ads12xx::waitforDRDY(int rdy) {  // polled, not interrupt
  int timeout=0;
	timeout = millis();
  while (digitalReadFast(rdy) == 1) {
		if(millis() - timeout > 1000)
		  return false;  // data should always be ready by now
	}
  return true;
}

#define PAUSE delayMicroseconds(25) // was 10 usec

// function to get a 3byte conversion result from the adc
// BUGGY: sometimes skips a byte (!?)
long ads12xx::GetConversion() {
	int32_t regData;
	if(ads12xx::waitforDRDY(_DRDY)==0){  // Wait until DRDY is LOW
		Serial.println("Timeout in Wait_for_DRDY");
		return 0;
		};
	SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
	PAUSE;
	digitalWriteFast(_CS, LOW); //Pull SS Low to Enable Communications with ADS1247
	PAUSE; PAUSE; // Datasheet claims 0 setup time needed here. Untrue?
	SPI.transfer(RDATA); //Issue RDATA
  PAUSE;

	// assemble 3 bytes into one 32 bit word
  regData = ((uint32_t)SPI.transfer(NOP) << 16) & 0x00FF0000;
  regData |= ((uint32_t)SPI.transfer(NOP) << 8);
  regData |= SPI.transfer(NOP);

  PAUSE; PAUSE;
	digitalWriteFast(_CS, HIGH);
	// PAUSE;
	SPI.endTransaction();

	//  sign-extend negative 24-bit number to i32
  if (regData & 0x800000)
  	{	regData |= 0xFF000000; }
	return regData;
}

// function to write a register value to the adc
// argumen: adress for the register to write into, value to write
void ads12xx::SetRegisterValue(uint8_t regAdress, uint8_t regValue) {
#ifdef ADS1248
	if (regAdress == IDAC0) {
		regValue = regValue | IDAC0_ID;	  // add non 0 non-write register value IDAC0_ID
	}
#endif
	uint8_t regValuePre = ads12xx::GetRegisterValue(regAdress);
	if (regValue != regValuePre) {
		//digitalWriteFast(_START, HIGH);
		delayMicroseconds(10);
		//waitforDRDY(_DRDY);
		SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with SPI_SPEED, MSB first, SPI Mode1
		digitalWriteFast(_CS, LOW);
		delayMicroseconds(10);
		//SPI.transfer(SDATAC); //Issue SDATAC
		delayMicroseconds(10);
		SPI.transfer(WREG | regAdress); // send 1st command byte, address of the register
		SPI.transfer(0x00);		// send 2nd command byte, write only one register
		SPI.transfer(regValue);         // write data (1 Byte) for the register
		delayMicroseconds(10);
		digitalWriteFast(_CS, HIGH);
		//digitalWriteFast(_START, LOW);
		if (regValue != ads12xx::GetRegisterValue(regAdress)) {   //Check if write was succesfull
			Serial.print("Write to Register 0x");
			Serial.print(regAdress, HEX);
			Serial.println(" failed!");
		}
		SPI.endTransaction();
	}
}


//function to read a register value from the adc
//argument: adress for the register to read
unsigned long ads12xx::GetRegisterValue(uint8_t regAdress) {
	uint8_t bufr;
	//digitalWriteFast(_START, HIGH);
	//waitforDRDY(_DRDY);
	SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with 4Mhz clock, MSB first, SPI Mode0
	digitalWriteFast(_CS, LOW);
	delayMicroseconds(10);
	SPI.transfer(RREG | regAdress); // send 1st command byte, address of the register
	SPI.transfer(0x00);			// send 2nd command byte, read only one register
	delayMicroseconds(10);
	bufr = SPI.transfer(NOP);	// read data of the register
	delayMicroseconds(10);
	digitalWriteFast(_CS, HIGH);
	//digitalWriteFast(_START, LOW);
	SPI.endTransaction();
	return bufr;
}

/*
Sends a Command to the ADC
Like SELFCAL, GAIN, SYNC, WAKEUP
*/
void ads12xx::SendCMD(uint8_t cmd) {
	ads12xx::waitforDRDY(_DRDY);
	SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with 4Mhz clock, MSB first, SPI Mode0
	digitalWriteFast(_CS, LOW);
	delayMicroseconds(10);
	SPI.transfer(cmd);
	delayMicroseconds(10);
	digitalWriteFast(_CS, HIGH);
	SPI.endTransaction();
}


// function to reset the adc
void ads12xx::Reset() {
	SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
	digitalWriteFast(_CS, LOW);
	delayMicroseconds(10);
	SPI.transfer(RESET); //Reset
	delay(2); //Minimum 0.6ms required for Reset to finish.
	SPI.transfer(SDATAC); //Issue SDATAC
	delayMicroseconds(100);
	digitalWriteFast(_CS, HIGH);
	SPI.endTransaction();
}
