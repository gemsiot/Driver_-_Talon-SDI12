/******************************************************************************
SDI12Talon
Interface for SDI12 Talon
Bobby Schulz @ GEMS Sensing
7/15/2022
https://github.com/gemsiot/Driver_-_Talon-SDI12

Allows control of all elements of the I2C Talon, including IO interfacing and self diagnostics 

0.0.0

///////////////////////////////////////////////////////////////////FILL QUOTE////////////////////////////////////////////////////////////////////////////////

Distributed as-is; no warranty is given.

© 2023 Regents of the University of Minnesota. All rights reserved.
******************************************************************************/

#include <SDI12Talon.h>

SDI12Talon::SDI12Talon(uint8_t talonPort_, uint8_t hardwareVersion) : ioAlpha(0x25), adcSense(0x6B), apogeeSense(0x4D)
{
	if(talonPort_ > 0) talonPort = talonPort_ - 1;
	else talonPort = 255; //Reset to null default if not in range
	version = hardwareVersion; //Copy to local
	talonInterface = BusType::SDI12;
}

String SDI12Talon::begin(time_t time, bool &criticalFault, bool &fault) 
{
	// Serial.println("IO EXP BEGIN"); //DEBUG!
	//Only use isEnabled() if using particle
	// #if defined(ARDUINO) && ARDUINO >= 100 
		// Wire.begin();
	// #elif defined(PARTICLE)
	// if(Wire.isEnabled()) Wire.end(); //DEBUG!
	// Wire.setSpeed(CLOCK_SPEED_100KHZ); 
	if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	
	// #endif

	// bool criticalFault = false; //Used to keep track if a critical error has been encountered during the initialization
	// bool fault = false; //Used to keep track if a non-critical error has been encountered during the initialization
	int startingErrors = numErrors; //Grab the number of errors which have been logged when we start the begin call, used to keep track of new errors
	//Initalize io expanders 
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	int ioError = ioAlpha.begin();
	ioSense.begin(Wire);
	// ioError[1] = ioBeta.begin();
	// ioError[2] = ioGamma.begin();	

	// for(int i = 0; i < 3; i++) {
		if(ioError != 0) { 
			throwError(IO_INIT_FAIL | ioError | talonPortErrorCode); //Throw error on first init error, not again 
			criticalFault = true; //If any IO expander fails, this is a critical error  
			// break;
		}
	// }

	// Wire.beginTransmission(ADR_ADS1115);
	// Wire.write(0x00);
	// if(Wire.endTransmission() != 0) {
	// 	throwError(ADC_INIT_ERROR); //Throw ADC initialization error
	// 	fault = true; //Set non-critical fault flag
	// }
	// ads.begin();

	setPinDefaults();
	pinMode(TX_SDI12, OUTPUT); 
	Serial1.begin(1200, SERIAL_8N1); //Initialize SDI12 serial port
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, LOW); //Turn off sensing
	// disablePowerAll(); //Turn off all power  
	// disableDataAll(); //Turn off all data
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Set fault lines as outputs
		ioAlpha.pinMode(i, OUTPUT); 
		ioAlpha.digitalWrite(i, LOW);
	}

	for(int i = 1; i <= numPorts; i++) { //Enable power
		faults[i - 1] = false; //Reset fault state
		uint16_t zeroCurrent = getBaseCurrent();
		Serial.print("Base Current: ");
		Serial.println(zeroCurrent); //DEBUG!
		enablePower(i, true); //Turn on power for each port
		delayMicroseconds(500); //Make sure ports are switched 
		uint16_t currentRising = getCurrent();
		if(currentRising > maxTalonCurrent) {
			enablePower(i, false); //Turn port back off
			throwError(TALON_POWER_FAIL_EXCESS | talonPortErrorCode); //Throw error for excess power, do not throw any specific fault this port, yet... - it may not be its fault
			//DONT FAULT PORT YET
			faults[i - 1] = true; //Fault for lack of better method FIX! - add intelegent switched system to deal with excess bus current if no single sensor exceeds it 
			if((currentRising - zeroCurrent) > maxTalonCurrent) throwError(SENSOR_POWER_INIT_FAIL | 0x100 | talonPortErrorCode | i); //Throw concatonated error code - rising fail fail
		}
		else if((currentRising - zeroCurrent) > maxPortCurrent) throwError(SENSOR_POWER_WARNING | 0x100 | talonPortErrorCode | i); //Throw warning for exceeding 500mA - does not MEAN this port is a problem
		delay(10); //Wait for capacitive spike to go away
		if(testOvercurrent(zeroCurrent)) { //Check if STILL excess current 
			enablePower(i, false); //Turn port back off
			faults[i - 1] = true; //Store which ports have faulted 
			Serial.print("Port Fault: "); //DEBUG!
			Serial.println(i);
			throwError(SENSOR_POWER_INIT_FAIL | 0x200 | talonPortErrorCode | i); //Throw concatonated error code - steady state fail
		}
		else if(i == 4) { //If testing port 4 (Apogee port) AND fault did not occour while turning it on, check for SDI-12 presence 
			unsigned long currentTime = millis(); //Grab current time
			float val = 0;
			Serial.println("Apogee SDI-12 Testing:"); //DEBUG!
			while((millis() - currentTime) < 100) { //Take continuious measures for up to 100ms
				val = apogeeSense.getVoltage(5.0); //Get voltage with a 5V refernce value 
				// Serial.println(val);
				if(val > 4.5) {
					apogeeDetected = true; //If pulse is observed, flag SDI-12 as detected
					break; //Exit while if condition met
				}
			}
		}
	}
	delay(2000); //Delay to wait for high power draw of O2 sensor to be over 
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH); //Turn sensing back on
	// delay(500); //Delay to wait for high power draw of O2 sensor to be over 
	for(int i = 1; i <= numPorts; i++) { //Toggle power to all ports to reset faults
		if(!faults[i - 1]) { //Only toggle back on if no fault
			// enablePower(i, true);
			// // delayMicroseconds(10);
			// delay(1); //DEBUG!
			// enablePower(i, false);
			// // delayMicroseconds(10);
			// delay(1); //DEBUG!
			// enablePower(i, true);
			/////////////////////////////////////////////////////////// THIS FIXES O2 START ERROR FOR SOME REASON!!!!!! ??????????????????????????????????????
			ioAlpha.digitalWrite(i - 1, HIGH);
			delayMicroseconds(10);
			ioAlpha.digitalWrite(i - 1, LOW);
			delayMicroseconds(10);
			ioAlpha.digitalWrite(i - 1, HIGH);
		}
		
	}
	// delay(500); //Delay to wait for high power draw of O2 sensor to be over 

	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Release fault lines
		ioAlpha.pinMode(i, INPUT); 
		// ioAlpha.digitalWrite(i, LOW);
	}
	ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
	// delay(100); //Let output settle //DEBUG! 
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	// ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW);
	// delay(10);
	// ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW);
	
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	// ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, LOW); //Reset sensing 
	// delay(10);
	
	
	///////////////////// RUN DIAGNOSTICS /////////////
	// digitalWrite(21, LOW); //DEBUG!!!! TURN OFF I2C OB
	// String diagnosticResults = selfDiagnostic(2); //Run level two diagnostic //DEBUG!
	// String diagnosticResults = "{}"; //DEBUG!
	// Serial.print("Init Diagnostic: "); //DEBUG!
	// Serial.println(diagnosticResults); //DEBUG!

	////////// RESET COUNTERS //////////////////////////
	// clearCount(time); //Clear counter and pass time info in
	// ioAlpha.digitalWrite(pinsAlpha::RST, LOW);
	// ioAlpha.digitalWrite(pinsAlpha::RST, HIGH); //Reset counters
	// ioBeta.digitalWrite(pinsBeta::LOAD, HIGH); //Load new counter values //TEST: check delta in fall-rise time to make sure there is enough time between reset and load 
	// ioBeta.digitalWrite(pinsBeta::LOAD, LOW);


	Serial.print("Talon Port: "); //DEBUG!
	Serial.println(talonPort);
	Serial.print("Kestrel Port: "); //DEBUG!
	Serial.println(KestrelPins::PortBPins[talonPort]);

	initDone = true; //Set init flag
	// if(criticalFault == true) return -1; //If a critical fault was detected, return with critical fault code
	if(numErrors - startingErrors > 0 || fault == true) fault = true; //If a non-critical fault was detected, or additional errors thrown, set fault
	// else return 0; //Only if no additional errors present, return operational state
	// return diagnosticResults; //Return diagnostic string
	return "";

}


// int AuxTalon::reportErrors(uint32_t *errorOutput, size_t length)
// {
// 	if(numErrors > length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, but array provided still too small
// 		for(int i = 0; i < length; i++) { //Write as many as we can back
// 			errorOutput[i] = error[i];
// 		}
// 		return -1; //Throw error for insufficnet array length
// 	}
// 	else if(numErrors < length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, provided array of good size (DESIRED)
// 		for(int i = 0; i < numErrors; i++) { //Write all back into array 
// 			errorOutput[i] = error[i];
// 		}
// 		return 0; //Return success indication
// 	}
// 	else if(numErrors > MAX_NUM_ERRORS && MAX_NUM_ERRORS < length) { //Overwritten, but array of good size 
// 		for(int i = 0; i < MAX_NUM_ERRORS; i++) { //Write all back into array 
// 			errorOutput[i] = error[i];
// 		}
// 		return 1; //Return overwrite indication
// 	}
// 	return -1; //Return fault if unknown cause 
// }
int SDI12Talon::sleep(bool State)
{
	return 0; //DEBUG!
}

String SDI12Talon::getErrors()
{
	// if(numErrors > length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, but array provided still too small
	// 	for(int i = 0; i < length; i++) { //Write as many as we can back
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return -1; //Throw error for insufficnet array length
	// }
	// if(numErrors < length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, provided array of good size (DESIRED)
	// 	for(int i = 0; i < numErrors; i++) { //Write all back into array 
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return 0; //Return success indication
	// }
	String output = "\"Talon-SDI12\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + "\"0x" + String(errors[i], HEX) + "\","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors) + ","; //Append number of errors
	output = output + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;

	// return -1; //Return fault if unknown cause 
}

// int SDI12Talon::throwError(uint32_t error)
// {
// 	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
// 	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
// 	return numErrors;
// }

String SDI12Talon::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
	unsigned long diagnosticStart = millis(); 
	if(getTalonPort() == 0) throwError(TALON_MISSING); //If Talon not found, report failure
	String output = "\"Talon-SDI12\":{";
	if(diagnosticLevel == 0) {
		//TBD
	}

	if(diagnosticLevel <= 1) {
		//TBD
		// output = output + "\"lvl-1\":{},";
	}

	if(diagnosticLevel <= 2) {
		//TBD
	}

	if(diagnosticLevel <= 3) {
		output = output + "\"Apogee_Type\":";
		if(getTalonPort() != 0) {
			if(apogeeDetected == true) output = output + "\"SDI-12\","; //Report SDI-12 if set to true
			else output = output + "\"Analog\","; //Otherwise report default 
		}
		else output = output + "null,"; //Report null if talon not detected 
		//TBD
 	}

	if(diagnosticLevel <= 4) {
		if(getTalonPort() != 0) { //If Talon has been detected, run tests
			for(int i = pinsSense::MUX_SEL0; i <= pinsSense::MUX_EN; i++) { //Set all pins to output
				ioSense.pinMode(i, OUTPUT); 
			}
			ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH); //Make sure 3v3 Sense is turned on
			ioSense.digitalWrite(pinsSense::MUX_EN, LOW); //Turn MUX on 
			int SenseError = adcSense.Begin(); //Initialize ADC 
			if(SenseError == 0) { //Only proceed if ADC connects correctly
				adcSense.SetResolution(18); //Set to max resolution (we paid for it right?) 

				output = output + "\"PORT_V\":["; //Open group
				ioSense.digitalWrite(pinsSense::MUX_SEL2, LOW); //Read voltages
				for(int i = 0; i < numPorts; i++){ //Increment through all ports
					ioSense.digitalWrite(pinsSense::MUX_SEL0, i & 0b01); //Set with lower bit
					ioSense.digitalWrite(pinsSense::MUX_SEL1, (i & 0b10) >> 1); //Set with high bit
					delay(1); //Wait for voltage to stabilize
					output = output + String(adcSense.GetVoltage(true)*voltageDiv, 6); //Print high resultion voltage as volts
					if(i < (numPorts - 1)) output = output + ","; //Append comma if not the last reading
					// Serial.print("\tPort");
					// Serial.print(i);
					// Serial.print(":");
					// Serial.print(adcSense.GetVoltage(true)*voltageDiv, 6); //Print high resolution voltage
					// Serial.print(" V\n");  
				}
				output = output + "],"; //Close group
				output = output + "\"PORT_I\":["; //Open group
				ioSense.digitalWrite(pinsSense::MUX_SEL2, HIGH); //Read currents
				for(int i = 0; i < numPorts; i++){ //Increment through 4 voltages
					ioSense.digitalWrite(pinsSense::MUX_SEL0, i & 0b01); //Set with lower bit
					ioSense.digitalWrite(pinsSense::MUX_SEL1, (i & 0b10) >> 1); //Set with high bit
					delay(1); //Wait for voltage to stabilize
					output = output + String(adcSense.GetVoltage(true)*currentDiv*1000, 6); //Print high resultion current as mA
					if(i < (numPorts - 1)) output = output + ","; //Append comma if not the last reading
					// Serial.print("\tPort");
					// Serial.print(i);
					// Serial.print(":");
					// Serial.print(adcSense.GetVoltage(true)*currentDiv*1000, 6); //Print high resolution current measure in mA
					// Serial.print(" mA\n");  
				}
				output = output + "],"; //Close group
			}
			else { //If unable to initialzie ADC
				output = output + "\"PORT_V\":[null],\"PORT_I\":[null],";
				throwError(SENSE_ADC_INIT_FAIL | talonPortErrorCode); //Throw error for ADC failure
			}
			ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Turn MUX back off 

			enableData(4, false); //Switch to ADC input
			delay(10); //wait for voltage to settle
			output = output + "\"Apogee_V\":" + String(apogeeSense.getVoltage(5.0)) + ","; //Append apogee voltage reading, if SDI-12 is detected or not
			enableData(4, true); //Try to renable data, if analog has been detected, this will fail and it will keep it as analog configuration
		}
		else output = output + "\"PORT_V\":[null],\"PORT_I\":[null],\"Apogee_V\":null,"; //Append null filled string
	}
	

	if(diagnosticLevel <= 5) {
		if(getTalonPort() != 0) {
			disableDataAll(); //Turn off all data 
			for(int i = 0; i < numPorts; i++) {
				faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
				if (faults[i] == true) {
					throwError(SENSOR_POWER_FAIL | talonPortErrorCode | i + 1); //Throw power fault error with given port appended 
				}
			}

			output = output + "\"ALPHA\":" + String(ioAlpha.readBus()) + ","; //Append ALPHA port readout
			output = output + "\"ALPHA_INT\":" + String(ioAlpha.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append ALPHA interrupt readout
			uint8_t senseBus = 0; //Manually concatonate pin reads from ioSense expander
			for(int i = 0; i < 4; i++) { //NOT associated with numPorts, which is why variable is not used
				senseBus = senseBus | (ioSense.read(i) << i); //DEBUG!
			}
			output = output + "\"MUX\":" + String(senseBus) + ","; //Append MUX controller readout

			output = output + "\"I2C_OB\":[";
			for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
				Wire.beginTransmission(adr);
				if(Wire.endTransmission() == 0) {
					output = output + String(adr) + ",";
				}
				delay(1); //DEBUG!
			}
			if(output.substring(output.length() - 1).equals(",")) {
				output = output.substring(0, output.length() - 1); //Trim trailing ',' is present
			}
			output = output + "],"; // close array
			/////////// LOOPBACK /////////////
			// output = output + "\"LB\":"; //Add loopback key
			// disableDataAll(); //Disconnect all external data
			// ioSense.digitalWrite(pinsSense::MUX_EN, LOW); //Enable mux to enable loopback 
			// ioAlpha.digitalWrite(pinsAlpha::FOUT, HIGH); //Release FOUT
			// ioAlpha.digitalWrite(pinsAlpha::DIR, LOW); //Force enable
			// Serial1.begin(1200, SERIAL_8N1); //Make sure serial is enabled 
			// // unsigned long localTime = millis();
			// // while(Serial1.available() > 0 && (millis() - localTime) < 100) Serial1.read(); //Clear buffer with timeout
			// for(int i = 0; i < 128; i++) Serial1.read(); //Clear buffer
			// Serial1.println("DEADBEEF");
			// Serial1.flush(); //Wait to finish transmission
			// String result = Serial1.readStringUntil('\n');
			// Serial.print("Loopback result"); //DEBUG!
			// Serial.println(result); //DEBUG!
			// result = result.trim(); //Get rid of format characters 
			// if(result.equals("DEADBEEF")) output = output + "1,"; //Append pass
			// else output = output + "0,"; //Otherwise append fail
			// ///////// ISOLATION ///////////
			// output = output + "\"ISO\":"; //Add isolation key
			// ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Disable loopback
			// // localTime = millis();
			// // while(Serial1.available() > 0 && (millis() - localTime) < 100) Serial1.read(); //Clear buffer with timeout
			// for(int i = 0; i < 128; i++) Serial1.read(); //Clear buffer
			// Serial1.println("DEADBEEF");
			// Serial1.flush(); //Wait to finish transmission
			// if(Serial1.available() > 0) output = output + "0,"; //Fail if any serial info is read in
			// else output = output + "1,"; //Append pass if no data read in
			// // ioAlpha.digitalWrite(pinsAlpha::DIR, LOW); //Default back to input
			///////// GET PORT ADRs ////////////
			output = output + "\"ADRs\":["; //Grab address from each port
			for(int i = 1; i <= numPorts; i++) {
				enableData(i, true); //Turn on data port
				String adr = sendCommand("?!");
				int adrVal = adr.toInt();
				if(adr.equals("") || (!adr.equals("0") && adrVal == 0)) output = output + "null"; //If no return, report null
				else output = output + adr; //Otherwise report the read value
				if(i < numPorts) output = output + ","; //Add comma if not last entry
				enableData(i, false); //Turn data port back off
			}
			output = output + "],"; //Close array

			ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		}
		else output = output + "\"ALPHA\":null,\"ALPHA_INT\":null,\"MUX\":null,\"I2C_OB\":[null],\"LB\":null,\"ISO\":null,\"ADRs\":[null,null,null,null],"; //Append null filled string 
		
	}
	if((millis() - diagnosticStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x200 | talonPortErrorCode | sensorPortErrorCode); //Throw error for diagnostic taking too long
	return output + "\"Pos\":[" + getTalonPortString() + "]}"; //Write position in logical form - Return compleated closed output
}

bool SDI12Talon::hasReset()
{
	// int error = 0; //Used to store the error from I2C read
	// uint16_t outputState = ioAlpha.readWord(0x06, error); //Read from configuration register 
	// if(((outputState >> pinsAlpha::MUX_EN) & 0x01) == 0x00) return false; //If output is still 
	Wire.beginTransmission(0x41); //Write to sense IO expander 
	Wire.write(0x03); //Read from configuration reg
	int error = Wire.endTransmission();
	Wire.requestFrom(0x41, 1); //Read single byte back
	uint8_t state = Wire.read();

	if(((state >> pinsSense::MUX_EN) & 0x01) == 0 && error == 0) return false; //If MUX_EN is set as an output AND there is no I2C error, device has not reset  
	else return true; //If the MUX_EN pin is no longer configured as an output, assume the Talon has reset
}

int SDI12Talon::restart()
{
	bool hasCriticalError = false;
	bool hasError = false;
	Serial.println("SDI-12 Restart Call"); //DEBUG!
	if(hasReset() && initDone) {
		Serial.println("RESTART SDI12 TALON - BEGIN"); //DEBUG!
		begin(0, hasCriticalError, hasError); //If Talon has been power cycled, run begin function again
	}
	//THROW ERROR!
	// setPinDefaults(); //Reset IO expander pins to their default state
	bool hasFault = false;
	for(int i = 0; i < numPorts; i++) {
		// if(ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i) || ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) || ioAlpha.digitalRead(pinsAlpha::EN1 + i)) { //If previous fault, or current fault or if channel is disabled (assume fault)
		if(ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i) || ioAlpha.digitalRead(pinsAlpha::FAULT1 + i)) { //If previous fault, or current fault	
			throwError(SENSOR_POWER_FAIL | talonPortErrorCode | i + 1); //Throw error because a power failure has occured  
			hasFault = true; //Set flag if any return true
		}
	}
	if(hasFault) { //If there are power faults, reset the system
		ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, LOW); //Turn off sensing
		// disablePowerAll(); //Turn off all power  
		// disableDataAll(); //Turn off all data
		for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Set fault lines as outputs
			ioAlpha.pinMode(i, OUTPUT); 
			ioAlpha.digitalWrite(i, LOW);
		}

	for(int i = 1; i <= numPorts; i++) { //Enable power
		faults[i - 1] = false; //Reset fault state
		uint16_t zeroCurrent = getBaseCurrent();
		Serial.print("Base Current: ");
		Serial.println(zeroCurrent); //DEBUG!
		enablePower(i, true); //Turn on power for each port
		delayMicroseconds(500); //Make sure ports are switched 
		uint16_t currentRising = getCurrent();
		if(currentRising > maxTalonCurrent) {
			enablePower(i, false); //Turn port back off
			throwError(TALON_POWER_FAIL_EXCESS | talonPortErrorCode); //Throw error for excess power, do not throw any specific fault this port, yet... - it may not be its fault
			//DONT FAULT PORT YET
			faults[i - 1] = true; //Fault for lack of better method FIX! - add intelegent switched system to deal with excess bus current if no single sensor exceeds it 
			if((currentRising - zeroCurrent) > maxTalonCurrent) throwError(SENSOR_POWER_INIT_FAIL | 0x100 | talonPortErrorCode | i); //Throw concatonated error code - rising fail fail
		}
		else if((currentRising - zeroCurrent) > maxPortCurrent) throwError(SENSOR_POWER_WARNING | 0x100 | talonPortErrorCode | i); //Throw warning for exceeding 500mA - does not MEAN this port is a problem
		delay(10); //Wait for capacitive spike to go away
		if(testOvercurrent(zeroCurrent)) { //Check if STILL excess current 
			enablePower(i, false); //Turn port back off
			faults[i - 1] = true; //Store which ports have faulted 
			Serial.print("Port Fault: "); //DEBUG!
			Serial.println(i);
			throwError(SENSOR_POWER_INIT_FAIL | 0x200 | talonPortErrorCode | i); //Throw concatonated error code - steady state fail
		}
		else if(i == 4) { //If testing port 4 (Apogee port) AND fault did not occour while turning it on, check for SDI-12 presence 
			unsigned long currentTime = millis(); //Grab current time
			float val = 0;
			Serial.println("Apogee SDI-12 Testing:"); //DEBUG!
			while((millis() - currentTime) < 100) { //Take continuious measures for up to 100ms
				val = apogeeSense.getVoltage(5.0); //Get voltage with a 5V refernce value 
				// Serial.println(val);
				if(val > 4.5) {
					apogeeDetected = true; //If pulse is observed, flag SDI-12 as detected
					break; //Exit while if condition met
				}
			}
		}
	}
		delay(2000); //Delay to wait for high power draw of O2 sensor to be over 
		ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH); //Turn sensing back on

		for(int i = 1; i <= numPorts; i++) { //Toggle power to all ports to reset faults
			if(!faults[i - 1]) { //Only toggle back on if no fault
				// enablePower(i, true);
				// delayMicroseconds(10);
				// enablePower(i, false);
				// delayMicroseconds(10);
				// enablePower(i, true);
				ioAlpha.digitalWrite(i - 1, HIGH);
				delayMicroseconds(10);
				ioAlpha.digitalWrite(i - 1, LOW);
				delayMicroseconds(10);
				ioAlpha.digitalWrite(i - 1, HIGH);
			}
		}

		for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Release fault lines
			ioAlpha.pinMode(i, INPUT); 
			// ioAlpha.digitalWrite(i, LOW);
		}
		ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		for(int i = 0; i < numPorts; i++) {
			if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i)) {
				throwError(SENSOR_POWER_FAIL_PERSISTENT | talonPortErrorCode | i + 1); //Throw error because a power failure still present
				hasFault = true; //Set flag if any return true
			}
		}
	}
	return 0; //FIX!
}

String SDI12Talon::getData(time_t time)
{
	unsigned long dataStart = millis();
	// String output = "{\"I2C_TALON\":"; //OPEN JSON BLOB
	String output = "\"Talon-SDI12\":{"; //OPEN JSON BLOB
	output = output + "\"Apogee\":";
	if(apogeeDetected == true) output = output + "null"; //If SDI is used, there is no data, report null
	else output = output + String(apogeeSense.getVoltage(5.0)); //Append voltage result if no SDI-12 detected 
	output = output + "}"; //Close blob
	if((millis() - dataStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x100 | talonPortErrorCode | sensorPortErrorCode); //Throw error for data taking too long
	return output; //DEBUG!
}






void SDI12Talon::setPinDefaults()
{
	// // if(talonPort < 4 && talonPort > 0) { //Only set if talonPort is in range
	// pinMode(KestrelPins::PortBPins[talonPort], OUTPUT); //Set Kestrel GPIO pins to specified states
	// pinMode(KestrelPins::PortAPins[talonPort], INPUT); 

	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal bus
	// // }
	

	// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, LOW); //Preempt loopback to off
	// ioAlpha.pinMode(pinsAlpha::LOOPBACK_EN, OUTPUT); 
	ioAlpha.pinMode(pinsAlpha::FOUT, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::DIR, LOW); //Default to recieve 
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT);
	// ioAlpha.digitalWrite(pinsAlpha::DIR, HIGH); //Default to transmit 
	

	ioAlpha.pinMode(pinsAlpha::POS_DETECT, INPUT); //Set position dection switch as input

	ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, LOW); //Turn on 3v3 sense by default
	ioAlpha.pinMode(pinsAlpha::SENSE_EN, OUTPUT); //Set SENSE_EN as an output
	
	
	ioAlpha.digitalWrite(pinsAlpha::EN1, LOW); //Preempt all power enables to low
	ioAlpha.digitalWrite(pinsAlpha::EN2, LOW); 
	ioAlpha.digitalWrite(pinsAlpha::EN3, LOW); 
	ioAlpha.digitalWrite(pinsAlpha::EN4, LOW); 

	ioAlpha.pinMode(pinsAlpha::EN1, OUTPUT); //Set all power enable to OUTPUT
	ioAlpha.pinMode(pinsAlpha::EN2, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::EN3, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::EN4, OUTPUT);

	ioAlpha.digitalWrite(pinsAlpha::DATA_EN1, LOW); //Preempt all I2C enables to low
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN2, LOW); 
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN3, LOW); 
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN4, LOW); //Default to analog connection

	ioAlpha.pinMode(pinsAlpha::DATA_EN1, OUTPUT); //Set all I2C enable to OUTPUT
	ioAlpha.pinMode(pinsAlpha::DATA_EN2, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::DATA_EN3, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::DATA_EN4, OUTPUT);

	ioAlpha.pinMode(pinsAlpha::FAULT1, INPUT_PULLDOWN); //Set FAULTx to input, default to pulldown to avoid erronious trips
	ioAlpha.pinMode(pinsAlpha::FAULT2, INPUT_PULLDOWN);
	ioAlpha.pinMode(pinsAlpha::FAULT3, INPUT_PULLDOWN);
	ioAlpha.pinMode(pinsAlpha::FAULT4, INPUT_PULLDOWN);

	ioAlpha.setLatch(pinsAlpha::FAULT1, true); //Turn on latching for fault inputs
	ioAlpha.setLatch(pinsAlpha::FAULT2, true);
	ioAlpha.setLatch(pinsAlpha::FAULT3, true);
	ioAlpha.setLatch(pinsAlpha::FAULT4, true);

	ioAlpha.setInterrupt(pinsAlpha::FAULT1, true); //Turn on interrupts for all FAULT pins
	ioAlpha.setInterrupt(pinsAlpha::FAULT2, true);
	ioAlpha.setInterrupt(pinsAlpha::FAULT3, true);
	ioAlpha.setInterrupt(pinsAlpha::FAULT4, true);

	ioSense.pinMode(pinsSense::MUX_EN, OUTPUT);
	ioSense.pinMode(pinsSense::MUX_SEL0, OUTPUT);
	ioSense.pinMode(pinsSense::MUX_SEL1, OUTPUT);
	ioSense.pinMode(pinsSense::MUX_SEL2, OUTPUT);

	ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Disable sensing MUX by default (this also disables loopback)
	ioSense.digitalWrite(pinsSense::MUX_SEL0, LOW);
	ioSense.digitalWrite(pinsSense::MUX_SEL1, LOW);
	ioSense.digitalWrite(pinsSense::MUX_SEL2, LOW);

	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Release to external bus
	// /////////////////////////////// Initialize IO ALPHA Pins /////////////////////////////////
	// ioAlpha.pinMode(pinsAlpha::MUX_EN, OUTPUT); //THIS CONFIGURATION IS ONLY MADE HERE, USED AS A HARDWARE SETUP TEST ELSEWHERE!
	// ioAlpha.digitalWrite(pinsAlpha::MUX_EN, HIGH); //Turn MUX off by default

	// ioAlpha.pinMode(pinsAlpha::MUX_SEL0, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::MUX_SEL1, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::MUX_SEL2, OUTPUT);

	// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, LOW);
	// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, LOW);
	// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, LOW);

	// ioAlpha.digitalWrite(pinsAlpha::ACTRL1, LOW); //Preempt before output enable
	// ioAlpha.digitalWrite(pinsAlpha::ACTRL2, LOW); //Preempt before output enable
	// ioAlpha.digitalWrite(pinsAlpha::ACTRL3, LOW); //Preempt before output enable

	// ioAlpha.pinMode(pinsAlpha::ACTRL1, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::ACTRL2, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::ACTRL3, OUTPUT);

	// ioAlpha.pinMode(pinsAlpha::FAULT1, INPUT_PULLUP); 
	// ioAlpha.pinMode(pinsAlpha::FAULT2, INPUT_PULLUP);
	// ioAlpha.pinMode(pinsAlpha::FAULT3, INPUT_PULLUP);

	// ioAlpha.pinMode(pinsAlpha::EN1, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::EN2, OUTPUT);
	// ioAlpha.pinMode(pinsAlpha::EN3, OUTPUT);

	// ioAlpha.digitalWrite(pinsAlpha::EN1, HIGH); //Default to ON
	// ioAlpha.digitalWrite(pinsAlpha::EN2, HIGH); //Default to ON
	// ioAlpha.digitalWrite(pinsAlpha::EN3, HIGH); //Default to ON

	// ioAlpha.pinMode(pinsAlpha::ADC_INT, INPUT_PULLUP); 

	// ioAlpha.pinMode(pinsAlpha::REG_EN, OUTPUT);
	// ioAlpha.digitalWrite(pinsAlpha::REG_EN, HIGH); //Turn on 5V converter 

	// ioAlpha.pinMode(pinsAlpha::RST, OUTPUT);
	// ioAlpha.digitalWrite(pinsAlpha::RST, HIGH); //Default to negate reset

	// /////////////////////////////// Initialize IO BETA Pins /////////////////////////////////
	// ioBeta.pinMode(pinsBeta::OUT1, INPUT);
	// ioBeta.pinMode(pinsBeta::OUT2, INPUT);
	// ioBeta.pinMode(pinsBeta::OUT3, INPUT);

	// ioBeta.pinMode(pinsBeta::OVF1, INPUT);
	// ioBeta.pinMode(pinsBeta::OVF2, INPUT);
	// ioBeta.pinMode(pinsBeta::OVF3, INPUT);

	// ioBeta.pinMode(pinsBeta::D1_SENSE, INPUT);
	// ioBeta.pinMode(pinsBeta::D2_SENSE, INPUT);
	// ioBeta.pinMode(pinsBeta::D3_SENSE, INPUT);

	// ioBeta.pinMode(pinsBeta::OD1, INPUT);
	// ioBeta.pinMode(pinsBeta::OD2, INPUT);
	// ioBeta.pinMode(pinsBeta::OD3, INPUT);

	// ioBeta.digitalWrite(pinsBeta::LOAD, LOW); //Preempt before output enable
	// ioBeta.pinMode(pinsBeta::LOAD, OUTPUT);

	// ioBeta.digitalWrite(pinsBeta::COUNT_EN1, LOW); //Preempt before output enable
	// ioBeta.digitalWrite(pinsBeta::COUNT_EN2, LOW); //Preempt before output enable
	// ioBeta.digitalWrite(pinsBeta::COUNT_EN3, LOW); //Preempt before output enable

	// ioBeta.pinMode(pinsBeta::COUNT_EN1, OUTPUT);
	// ioBeta.pinMode(pinsBeta::COUNT_EN2, OUTPUT);
	// ioBeta.pinMode(pinsBeta::COUNT_EN3, OUTPUT);

	// ioBeta.setLatch(pinsBeta::OVF1, true); //Turn on latching for all OVF interrupts
	// ioBeta.setLatch(pinsBeta::OVF2, true);
	// ioBeta.setLatch(pinsBeta::OVF3, true);

	// ioBeta.setInterrupt(pinsBeta::OVF1, true); //Turn on interrupts for all OVF pins
	// ioBeta.setInterrupt(pinsBeta::OVF2, true);
	// ioBeta.setInterrupt(pinsBeta::OVF3, true);

	// ioAlpha.setLatch(pinsAlpha::FAULT1, true); //Turn on latching for fault inputs
	// ioAlpha.setLatch(pinsAlpha::FAULT2, true);
	// ioAlpha.setLatch(pinsAlpha::FAULT3, true);

	// ioAlpha.setInterrupt(pinsAlpha::FAULT1, true); //Turn on interrupts for all FAULT pins
	// ioAlpha.setInterrupt(pinsAlpha::FAULT2, true);
	// ioAlpha.setInterrupt(pinsAlpha::FAULT3, true);
	
}

// bool SDI12Talon::hasReset()
// {
// 	// int error = 0; //Used to store the error from I2C read
// 	// uint16_t outputState = ioAlpha.readWord(0x06, error); //Read from configuration register 
// 	// if(((outputState >> pinsAlpha::MUX_EN) & 0x01) == 0x00) return false; //If output is still 
// 	// else return true; //If the MUX_EN pin is no longer configured as an output, assume the Talon has reset
// 	return false; //DEBUG!
// }

String SDI12Talon::getMetadata()
{
	unsigned long metadataStart = millis();
	Wire.beginTransmission(0x58); //Write to UUID range of EEPROM
	Wire.write(0x98); //Point to start of UUID
	int error = Wire.endTransmission();
	// uint64_t uuid = 0;
	String uuid = "";

	if(error != 0) throwError(TALON_EEPROM_READ_FAIL | error);
	else {
		uint8_t val = 0;
		Wire.requestFrom(0x58, 8); //EEPROM address
		for(int i = 0; i < 8; i++) {
			val = Wire.read();//FIX! Wait for result??
			// uuid = uuid | (val << (8 - i)); //Concatonate into full UUID
			uuid = uuid + String(val, HEX); //Print out each hex byte
			// Serial.print(Val, HEX); //Print each hex byte from left to right
			// if(i < 7) Serial.print('-'); //Print formatting chracter, don't print on last pass
			if(i < 7) uuid = uuid + "-"; //Print formatting chracter, don't print on last pass
		}
	}

	String metadata = "\"Talon-SDI12\":{";
	if(error == 0) metadata = metadata + "\"SN\":\"" + uuid + "\","; //Append UUID only if read correctly, skip otherwise 
	metadata = metadata + "\"Hardware\":\"v" + String(version >> 4, HEX) + "." + String(version & 0x0F, HEX) + "\","; //Report version as modded BCD
	metadata = metadata + "\"Firmware\":\"v" + FIRMWARE_VERSION + "\","; //Report firmware version as modded BCD
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
	if((millis() - metadataStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x300 | talonPortErrorCode | sensorPortErrorCode); //Throw error for metadata taking too long
	return metadata; 
	// return ""; //DEBUG!
}

// uint8_t SDI12Talon::totalErrors()
// {
// 	return numErrors;
// }

// bool SDI12Talon::ovfErrors()
// {
// 	if(numErrors > MAX_NUM_ERRORS) return true;
// 	else return false;
// }

// uint8_t AuxTalon::getTalonPort()
// {
// 	return port + 1; //Switch to rational counting
// }

// uint8_t AuxTalon::getTalon()
// {
// 	return port;
// }

// void SDI12Talon::setTalonPort(uint8_t port)
// {
// 	// if(port_ > numPorts || port_ == 0) throwError(PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error 
// 	if(port > 4 || port == 0) throwError(SENSOR_PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error //FIX! How to deal with magic number? This is the number of ports on KESTREL, how do we know that??
// 	else { //If in range, update the port values
// 		talonPort = port - 1; //Set global port value in index counting
// 		portErrorCode = (port + 1) << 4; //Set port error code in rational counting 
// 	}
// }

// int SDI12Talon::disableDataAll()
// {
// 	Serial.println("DISABLE ALL DATA I2C TALON"); //DEBUG!
// 	for(int i = 1; i <= numPorts; i++) {
// 		enableData(i, false);
// 	}
// 	return 0; //DEBUG!
// }

// int SDI12Talon::disablePowerAll()
// {
// 	Serial.println("DISABLE ALL POWER I2C TALON"); //DEBUG!
// 	for(int i = 1; i <= numPorts; i++) {
// 		enablePower(i, false);
// 	}
// 	return 0; //DEBUG!
// }

int SDI12Talon::enableData(uint8_t port, bool state)
{
	//FIX! Check for range and throw error
	bool success = false; 
	// pinMode(KestrelPins::PortBPins[talonPort], OUTPUT); //DEBUG!
	// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect I2C to internal I2C
	// pinMode(D6, OUTPUT); //DEBUG!
	// digitalWrite(D6, HIGH); //Connect I2C to internal I2C
	if(!apogeeDetected && port == 4) return false; //If Apogee port is commanded and SDI-12 has not been detected, ignore and return 
	ioAlpha.pinMode(pinsAlpha::DATA_EN1 + port - 1, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN1 + port - 1, state);
	if(ioAlpha.digitalRead(pinsAlpha::DATA_EN1 + port - 1) == state) {
		success = true; //If readback matches, set is a success
		portEnabled[port] = state; //Only update state if enable was successful 
	}
	else success = false; //Otherwise clear flag
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C
	// digitalWrite(D6, LOW); //Connect I2C to default external I2C
	return success; //DEBUG!
}

int SDI12Talon::enablePower(uint8_t port, bool state)
{
	// ioAlpha.pinMode(pinsAlpha)
	//FIX! Check for range and throw error
	bool success = false; 
	if(faults[port - 1] == false) { //Only mess with power if fault has not occoured 
		// pinMode(KestrelPins::PortBPins[talonPort], OUTPUT); //DEBUG!
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect I2C to internal I2C //DEBUG!
		// pinMode(D6, OUTPUT); //DEBUG!
		// digitalWrite(D6, HIGH); //Connect I2C to internal I2C
		// digitalWrite(6, LOW); //Connect I2C to internal I2C //DEBUG!
		ioAlpha.pinMode(pinsAlpha::EN1 + port - 1, OUTPUT);
		ioAlpha.digitalWrite(pinsAlpha::EN1 + port - 1, state);
		if(ioAlpha.digitalRead(pinsAlpha::EN1 + port - 1) == state) success = true; //If readback matches, set is a success
		else success = false; //Otherwise clear flag
		// digitalWrite(6, HIGH); //Connect I2C to internal I2C //DEBUG!
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C 
		// digitalWrite(D6, LOW); //Connect I2C to default external I2C
	}
	
	return success; //DEBUG!
}

int SDI12Talon::getEnabledPort()
{
	int numEnabledPorts = 0; 
	int pos = 0;
	for(int i = 0; i < numPorts; i++) {
		if(portEnabled[i] == true) {
			numEnabledPorts++; //Increment port count each time one is found
			pos = i + 1; //Grab current port 
		}
	}
	if(numEnabledPorts > 1) return 0; //If more than one port is enabled, return 0 (sys wide indication)
	else return pos; //If exactly 1 or 0 are enabled return pos. Catches 0 condition and retuns 0 default, otherwise returns the singular port which is enabled
}

void SDI12Talon::sendBreak()
{
	// io.pinMode(0, Dir, OUTPUT); //Make sure direction is set to output
	// io.pinMode(0, FOut, OUTPUT); //Make sure the force out pin is in output mode
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT); //DEBUG!
	ioAlpha.pinMode(pinsAlpha::FOUT, OUTPUT); //DEBUG!
	ioAlpha.digitalWrite(pinsAlpha::DIR, HIGH); //Set direction to output
	ioAlpha.digitalWrite(pinsAlpha::FOUT, LOW); //Set low to force output high
	delay(13); //Wait for bus to acnowledge action
	ioAlpha.digitalWrite(pinsAlpha::FOUT, HIGH); //Stop forcing output
}

void SDI12Talon::mark(unsigned long period)
{
	// io.pinMode(0, Dir, OUTPUT); //Make sure direction control pin is set as an output
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT); //DEBUG!
	digitalWrite(TX_SDI12, 1); //Preempt value before turning output on
	delay(1);
	pinMode(TX_SDI12, OUTPUT); //Make sure transmit pin is set as output
	ioAlpha.digitalWrite(pinsAlpha::DIR, HIGH); //Set direction to output
	digitalWrite(TX_SDI12, 1); //Begin marking condition
	delay(period); //Wait for a given marking period
	// digitalWrite(TX, 0); //Stop marking  
}

void SDI12Talon::space(unsigned long Period)
{
	// io.pinMode(0, Dir, OUTPUT); //Make sure direction control pin is set as an output
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT); //DEBUG!
	pinMode(TX_SDI12, OUTPUT); //Make sure transmit pin is set as output
	ioAlpha.digitalWrite(pinsAlpha::DIR, HIGH); //Set direction to output
	digitalWrite(TX_SDI12, 0); //Begin spacing condition
	delay(Period); //Wait for a given marking period
	digitalWrite(TX_SDI12, 1); //Stop spacing
}

void SDI12Talon::releaseBus() 
{
	// io.pinMode(0, Dir, OUTPUT); //Make sure direction pin is set as an output
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT); //DEBUG!
	ioAlpha.digitalWrite(pinsAlpha::DIR, LOW); //Set direction to inpout
}

int SDI12Talon::getAddress()
{
	String val = sendCommand("?!");
	
	if(val.charAt(0) > 0x39 || val.charAt(0) < 0x30) { //Check if address is outside of valid range
		throwError(SDI12_COM_FAIL | 0x100 | talonPortErrorCode | getEnabledPort()); //Throw address out of range error
		return -1;
	}
	else return val.toInt();
}

int SDI12Talon::startMeasurment(int Address)
{
	String val = command("M", Address);
	for(int i = 0; i < val.length(); i++) {
		if(val.charAt(i) < 0x30 || val.charAt(i) > 0x39 && val.charAt(i) != 0x0A && val.charAt(i) != 0x0D) { //If char is non-numeric AND not <CR> or <LF>) {
			if(i == 0) throwError(SDI12_COM_FAIL | 0x100 | talonPortErrorCode | getEnabledPort()); //If in the first index, throw address out of range error
			else throwError(SDI12_COM_FAIL | 0x300 | talonPortErrorCode | getEnabledPort()); //If not the first index, throw ACK out of range error
			return -1;
		}
	}
	return (val.substring(1,4)).toInt(); //Return number of seconds to wait
}

int SDI12Talon::startMeasurmentCRC(int Address)
{
	String val = command("MC", Address);
	Serial.print("SDI12 Start Measure CRC: "); //DEBUG!
	Serial.print(val);
	Serial.print(",");
	Serial.println(val.substring(1,3)); //DEBUG!
	for(int i = 0; i < val.length(); i++) {
		if(((val.charAt(i) < 0x30) || (val.charAt(i) > 0x39)) && (val.charAt(i) != 0x0A) && (val.charAt(i) != 0x0D)) { //If char is non-numeric AND not <CR> or <LF>
			if(i == 0) throwError(SDI12_COM_FAIL | 0x100 | talonPortErrorCode | getEnabledPort()); //If in the first index, throw address out of range error, append the port which is enabled
			else throwError(SDI12_COM_FAIL | 0x300 | talonPortErrorCode | getEnabledPort()); //If not the first index, throw ACK out of range error, append the port which is enabled 
			Serial.print("CRC Fail: "); //DEBUG!
			Serial.println(val.charAt(i), DEC); //Print value of char which failed
			return -1;
		}
	}
	return (val.substring(1,4)).toInt(); //Return number of seconds to wait
}

String SDI12Talon::continuousMeasurmentCRC(int Measure, int Address)
{
	if(Measure >= 0 && Measure <= 9) {
		String val = command("RC" + String(Measure), Address);
		// Serial.print("SDI12 Start Measure CRC: "); //DEBUG!
		// Serial.print(val);
		// Serial.print(",");
		// Serial.println(val.substring(1,3)); //DEBUG!


		// for(int i = 0; i < val.length(); i++) {
		// 	if(((val.charAt(i) < 0x30) || (val.charAt(i) > 0x39)) && (val.charAt(i) != 0x0A) && (val.charAt(i) != 0x0D)) { //If char is non-numeric AND not <CR> or <LF>
		// 		if(i == 0) throwError(SDI12_COM_FAIL | 0x100 | talonPortErrorCode | getEnabledPort()); //If in the first index, throw address out of range error, append the port which is enabled
		// 		else throwError(SDI12_COM_FAIL | 0x300 | talonPortErrorCode | getEnabledPort()); //If not the first index, throw ACK out of range error, append the port which is enabled 
		// 		// Serial.print("CRC Fail: "); //DEBUG!
		// 		// Serial.println(val.charAt(i), DEC); //Print value of char which failed
		// 		return ""; //Return empty string and flag CRC failure
		// 	}
		// }

		if((val.substring(0, 1)).toInt() != Address) { //If address returned is not the same as the address read, throw error
			throwError(SDI12_SENSOR_MISMATCH | 0x100 | talonPortErrorCode | sensorPortErrorCode); //Throw error on address change, this is a weird place for this error to happen, but could
			return ""; //Flag failure to parse
		}

		return val; //Pass
	}
	else return ""; //If measure is out of range
}


String SDI12Talon::command(String Command, int Address) //Correctly place address and line end into string to pass to SendCommand
{
  Command = String(Address) + Command + "!";
  return sendCommand(Command); //Return results
}

String SDI12Talon::sendCommand(String Command) 
{
	sendBreak(); //Send break to start message
	mark(MARKING_PERIOD); //Mark for standard period before command
	Serial1.begin(1200, SERIAL_8N1);
	Serial1.print(serialConvert8N1to7E1(Command)); //Send converted value
	Serial1.flush(); //Make sure data is transmitted before releasing the bus
	delay(2); //DEBUG! Return to 1ms??
	unsigned long localTime = millis();
	while(Serial1.available() > 0 && (millis() - localTime) < 10) Serial1.read(); //Clear buffer, read for at most 10ms //DEBUG!
	releaseBus(); //Switch bus to recieve 
	

	unsigned long LocalTime = millis();
	char Data[100] = {0}; //Make data array for storage FIX! Change length to not be arbitrary
	bool GotData = false; //Used to keep track of terminating character has been recieved 
	int Pos = 0; //Keep track of position in data array
	while(!GotData && (millis() - LocalTime) <  timeoutStandard) {
		if(Serial1.available() > 0) {
			Data[Pos] = serialConvert7E1to8N1(Serial1.read()); //If byte is availiable, read converted version in
			Pos++; //Increment position

		}
    if(Data[Pos] == '\n') GotData = true; //Set flag if end character is read in //DEBUG!
	}
	String Val = String(Data); //Convert to String
	Val.trim(); //Get rid of any trailing characters 
	return Val; 
}

bool SDI12Talon::testCRC(String message)
{
	// Serial.print("SDI12 Message: "); //DEBUG!
	// Serial.println(message); //DEBUG!
	uint8_t msgCRCBuff[4] = {0}; //Init buffer to grab characters from CRC, add 1 for terminating character enforced by getBytes
	message.trim(); //Remove <CR> and <LF> if still there 
	String msgCRCStr = message.substring(message.length() - 3); //Grab last 3 characters, this SHOULD be the CRC
	// Serial.print("SDI12 CRC: "); //DEBUG!
	// Serial.println(msgCRCStr); //DEBUG!
	msgCRCStr.getBytes(msgCRCBuff, 4); //Convert chars to bytes, put them in array
	// Serial.print("SDI12 CRC BUFF: "); //DEBUG!
	// Serial.print(msgCRCBuff[0]); //DEBUG!
	// Serial.print(msgCRCBuff[1]); //DEBUG!
	// Serial.println(msgCRCBuff[2]); //DEBUG!
	if((msgCRCBuff[0] & 0x40) != 0x40 || (msgCRCBuff[1] & 0x40) != 0x40 || (msgCRCBuff[2] & 0x40) != 0x40) { //If all CRC characters are not of the specified format, throw error and do not bother evaluating CRC
		// Serial.println("CRC INVALID!"); 
		throwError(SDI12_COM_FAIL | 0x200 | talonPortErrorCode | getEnabledPort()); //Throw error with CRC fail subtype
		//THROW ERROR!
		return 0; //Return false result
	}
	else {
		uint16_t msgCRC = ((msgCRCBuff[0] & 0x3F) << 12) | ((msgCRCBuff[1] & 0x3F) << 6) | (msgCRCBuff[2] & 0x3F); //Concatonate CRC from individual characters 
		// Serial.print("SDI12 MSG CRC: "); //DEBUG!
		// Serial.println(msgCRC); //DEBUG!
		String msgStr = message.substring(0, message.length() - 3); //Grab message up to CRC
		const uint8_t msgLen = message.length() - 3; 
		uint8_t msgBuff[msgLen + 1] = {0}; //Create buffer to store the bytes from each character of the message //Add 1 for terminating character enforced by getBytes
		msgStr.getBytes(msgBuff, msgLen + 1); //Dump the message into bytes of the buffer

		//Calculate internal CRC
		uint16_t crc = 0;
		for(int i = 0; i < msgLen; i++) { //Iterate over all characters in message
			crc = msgBuff[i]^crc; //Take XOR of character and CRC
			for(int count = 0; count < 8; count++) { //Iterate over each character bit
				if((crc & 0x01) == 0x01) { //If LSB is 1
					crc = crc >> 1; //Right shift 1
					crc = 0xA001^crc; //CRC = XOR(0xA001, crc)
				}
				else { //If LSB is 0
					crc = crc >> 1; //Right shift 1
				}
			}
		}
		// Serial.print("SDI12 CRC Test: "); //DEBUG!
		// Serial.println(crc); //DEBUG!
		if(crc == msgCRC) return 1; //If CRCs match, return success
		else {
			throwError(SDI12_COM_FAIL | 0x200 | talonPortErrorCode | getEnabledPort()); //Throw error with CRC fail subtype
			return 0; //If CRCs do not match, return a failure 
		}
	}
	return 0;
	
}

String SDI12Talon::serialConvert8N1to7E1(String Msg) //Take input message in 8N1 format and convert to 7E1 format 
{
	char MsgBuf[25] = {0}; //Initalize a char buffer to break apart string message //FIX! Make variable? Or just larger?
	uint8_t BufLength = sizeof(MsgBuf); //Keep track of size of buffer being used
	Msg.toCharArray(MsgBuf, BufLength); //Break message string into an array of chars 
	for(int i = 0; i < BufLength; i++) {
		MsgBuf[i] = MsgBuf[i] | (getParity(MsgBuf[i]) << 7); //Add parity bit to left most bit
	}
	// Serial.println(String(MsgBuf)); //DEBUG!
	return String(MsgBuf); //Convert array back to string and return this 
}

char SDI12Talon::serialConvert7E1to8N1(char Msg) //Take input message in 8N1 format and convert to 7E1 format 
{
	return Msg & 0x7F; //Clear parity bit
}

bool SDI12Talon::getParity(char Data) //Calculate partiy of a char of data
{
	bool Parity = 0;
	while(Data != 0) {
		Parity = !Parity; //Invert parity each time a '1' is encountered
		Data = Data & (Data - 1); //Clear next '1' in message
	}
	return Parity; 
}

bool SDI12Talon::isPresent()
{
	//FIX! Update for more complete interrogation 
	Wire.beginTransmission(0x25);
	int error = Wire.endTransmission();
	if(error == 0) return true;
	else return false;
	// return false; //DEBUG!
}

uint16_t SDI12Talon::getCurrent()
{
	bool prevI2C = digitalRead(KestrelPins::I2C_OB_EN);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C 
	digitalWrite(KestrelPins::I2C_OB_EN, HIGH); //Turn on OB I2C bus
	int ADR = 0x14;
	// Wire.beginTransmission(ADR);
	// Wire.write(0x00); //Make sure read rate is set to 1024 sps
	// Wire.write(0x00); 
	// uint8_t error = Wire.endTransmission();

	Wire.beginTransmission(ADR);
	Wire.write(0x1F); //Write refresh command
	Wire.write(0x00); //Initilize a clear
	uint8_t error = Wire.endTransmission();
	delay(1);
	

	Wire.beginTransmission(ADR);
	Wire.write(0x0E); //Get sense 4 value
	error = Wire.endTransmission(); //Store Error

	unsigned long localTime = millis();
	Wire.requestFrom(ADR, 2, true);
	while(Wire.available() < 2 && (millis() - localTime) < 10); //Wait up to 10 ms 
	uint8_t byteHigh = Wire.read();  //Read in high and low bytes (big endian)
	uint8_t byteLow = Wire.read();

	uint16_t result = ((byteHigh << 8) | byteLow); //concatonate result 
	// digitalWrite(KestrelPins::PortBPins[talonPort], prevState); //Return OB enable to previous state
	digitalWrite(KestrelPins::I2C_OB_EN, prevI2C); //Turn on OB I2C bus back off
	// Serial.print("Overcurrent Test: "); //DEBUG!
	// Serial.println(result);
	if(error != 0) {
		throwError(CSA_READ_FAIL | (error << 8)); //Throw error if error in reading from CSA, OR with error code from I2C
		return maxTalonCurrent + 1; //Return excessive current to induce error
	}
	else return result;
	// if((result) > 13104 || error != 0) return true; //If current is greater than 2A TOTAL, or unable to read current, return true
	// else return false; //Otherwise return false 
}
bool SDI12Talon::testOvercurrent(uint16_t baseCurrent)
{
	// bool prevState = digitalRead(KestrelPins::PortBPins[talonPort]); //Check the current state of the OB enable line
	bool prevI2C = digitalRead(KestrelPins::I2C_OB_EN);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C 
	digitalWrite(KestrelPins::I2C_OB_EN, HIGH); //Turn on OB I2C bus
	int ADR = 0x14;
	// Wire.beginTransmission(ADR);
	// Wire.write(0x00); //Make sure read rate is set to 1024 sps
	// Wire.write(0x00); 
	// uint8_t error = Wire.endTransmission();

	Wire.beginTransmission(ADR);
	Wire.write(0x1F); //Write refresh command
	Wire.write(0x00); //Initilize a clear
	uint8_t error = Wire.endTransmission();
	delay(1);
	

	Wire.beginTransmission(ADR);
	Wire.write(0x0E); //Get sense 4 value
	error = Wire.endTransmission(); //Store Error

	unsigned long localTime = millis();
	Wire.requestFrom(ADR, 2, true);
	while(Wire.available() < 2 && (millis() - localTime) < 10); //Wait up to 10 ms 
	uint8_t byteHigh = Wire.read();  //Read in high and low bytes (big endian)
	uint8_t byteLow = Wire.read();

	uint16_t result = ((byteHigh << 8) | byteLow); //concatonate result 
	// digitalWrite(KestrelPins::PortBPins[talonPort], prevState); //Return OB enable to previous state
	digitalWrite(KestrelPins::I2C_OB_EN, prevI2C); //Turn on OB I2C bus back off
	Serial.print("Overcurrent Test: "); //DEBUG!
	Serial.println(result);
	if(error != 0) throwError(CSA_READ_FAIL | (error << 8)); //Throw error if error in reading from CSA, OR with error code from I2C
	if((result - baseCurrent) > maxPortCurrent || error != 0) return true; //If current is greater than 500mA ADDITIONAL, or unable to read current, return true
	else return false; //Otherwise return false 
}

uint16_t SDI12Talon::getBaseCurrent()
{
	bool prevI2C = digitalRead(KestrelPins::I2C_OB_EN);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C 
	digitalWrite(KestrelPins::I2C_OB_EN, HIGH); //Turn on OB I2C bus
	int ADR = 0x14;
	// Wire.beginTransmission(ADR);
	// Wire.write(0x00); //Make sure read rate is set to 1024 sps
	// Wire.write(0x00); 
	// uint8_t error = Wire.endTransmission();

	Wire.beginTransmission(ADR);
	Wire.write(0x1F); //Write refresh command
	Wire.write(0x00); //Initilize a clear
	uint8_t error = Wire.endTransmission();
	delay(1);
	

	Wire.beginTransmission(ADR);
	Wire.write(0x0E); //Get sense 4 value
	error = Wire.endTransmission(); //Store Error

	unsigned long localTime = millis();
	Wire.requestFrom(ADR, 2, true);
	while(Wire.available() < 2 && (millis() - localTime) < 10); //Wait up to 10 ms 
	uint8_t byteHigh = Wire.read();  //Read in high and low bytes (big endian)
	uint8_t byteLow = Wire.read();

	uint16_t result = ((byteHigh << 8) | byteLow); //concatonate result 
	digitalWrite(KestrelPins::I2C_OB_EN, prevI2C); //Turn on OB I2C bus back off
	if(error != 0) return 0; //If error in read, use base of 0
	else return result;
}