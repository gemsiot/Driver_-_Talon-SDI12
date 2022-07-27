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
******************************************************************************/

#include <SDI12Talon.h>

SDI12Talon::SDI12Talon(uint8_t talonPort_, uint8_t hardwareVersion) : ioAlpha(0x25), adcSense(0x6B)
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
			throwError(IO_INIT_ERROR | ioError); //Throw error on first init error, not again 
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
		enablePower(i, true); //Turn on power for each port
		delayMicroseconds(1500);
		if(testOvercurrent()) { //Check if excess current 
			enablePower(i, false); //Turn port back off
			faults[i - 1] = true; //Store which ports have faulted 
			Serial.print("Port Fault: "); //DEBUG!
			Serial.println(i);
		}
	}
	delay(500); //Delay to wait for high power draw of O2 sensor to be over 
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
	String diagnosticResults = selfDiagnostic(2); //Run level two diagnostic //DEBUG!
	// String diagnosticResults = "{}"; //DEBUG!
	Serial.print("Init Diagnostic: "); //DEBUG!
	Serial.println(diagnosticResults); //DEBUG!

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
	return diagnosticResults; //Return diagnostic string

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
	String output = "{\"Talon-I2C\":{"; // OPEN JSON BLOB
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
	output = output + "}}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;

	// return -1; //Return fault if unknown cause 
}

int SDI12Talon::throwError(uint32_t error)
{
	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
	return numErrors;
}

String SDI12Talon::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
	String output = "{\"Talon-SDI12\":{";
	if(diagnosticLevel == 0) {
		//TBD
		output = output + "\"lvl-0\":{},";
		// return output + "\"lvl-0\":{},\"Pos\":[" + String(port) + "]}}";
	}

	if(diagnosticLevel <= 1) {
		//TBD
		output = output + "\"lvl-1\":{},";
	}

	if(diagnosticLevel <= 2) {
		//TBD
		output = output + "\"lvl-2\":{},";
		// String level3 = selfDiagnostic(3, time); //Call the lower level of self diagnostic 
		// level3 = level3.substring(1,level3.length() - 1); //Trim off opening and closing brace
		// output = output + level3; //Concatonate level 4 on top of level 3
		// output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;
		// return "{\"lvl-2\":{}," + selfDiagnostic(3, time).substring(0, ) }";
		// pinMode(KestrelPins::PortAPins[port], INPUT); //DEBUG!
	}

	if(diagnosticLevel <= 3) {
		//TBD
		// Serial.println(millis()); //DEBUG!
		output = output + "\"lvl-3\":{"; //OPEN JSON BLOB
		/////// TEST I2C WITH LOOPBACK ////////////
		output = output + "\"I2C_PORT_FAIL\":["; 
		disableDataAll(); //Turn off all data 
		digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C
		Wire.beginTransmission(0x22);
		int error = Wire.endTransmission(); //Get error from write to empty bus
		if(error == 0) throwError(I2C_OB_ISO_FAIL | talonPortErrorCode); //We should not be able to connect to IO exp in this state, if we can throw error 
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, HIGH); //Connect loopback 
		for(int i = 0; i <= numPorts; i++) { //Iterate over each port
			int totalErrors = 0; //Track how many of the test calls fail
			if(i > 0) {
				enablePower(i, true); //Turn on power to a given power after testing the base bus
				enableData(i, true); //Turn on data to a given port after testing the base bus
			}
			// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C (w/loopback enabled, so it is a combined bus now)
			for(int adr = 0; adr < sizeof(expectedI2CVals); adr++) { //Check for addresses present 
				Wire.beginTransmission(expectedI2CVals[adr]); //Check for each expected address
				// Wire.write(0x00);
				int error = Wire.endTransmission();
				if(error == 2) { //If bad bus error detected 
					// Serial.print("I2C Error Detected: "); //DEBUG!
					// Serial.println(error);
					totalErrors++; //If a failure occours, increment error count
				}
				// delay(1); //DEBUG!
			}
			Serial.print("Total Errors: "); //DEBUG!
			Serial.println(totalErrors);
			if(totalErrors > 0) { //If any bus failures were detected 
				throwError(I2C_PORT_FAIL | talonPortErrorCode | i); //Throw error for the port under test
				output = output + String(i) + ",";
			}
			if(i > 0) enableData(i, false); //Disable bus again
		}
		if(output.substring(output.length() - 1).equals(",")) {
			output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
		}
		output = output + "]"; //Close I2C port array
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, LOW); //Disable loopback 
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Return to default external connecton
		///////// TEST INPUT DRIVES //////////////
		// const int numPulses = 5; //Number of pulses to use for testing
		// for(int i = 0; i < 3; i++) {
		// 	ioBeta.digitalWrite(pinsBeta::OD1 + i, LOW); //Preempt the output as low
		// 	ioBeta.pinMode(pinsBeta::OD1 + i, OUTPUT); //Set line to output	
		// 	ioBeta.digitalWrite(pinsBeta::D1_SENSE + i, LOW); //Preempt low
		// 	ioBeta.pinMode(pinsBeta::D1_SENSE + i, OUTPUT); //Set to output to drive push-pull
		// 	ioBeta.pinMode(pinsBeta::OUT1 + i, INPUT); //Set as input so we can measure the output of the input buffer
		// }
		// clearCount(time); //Clear counters to start with 0 value
		// bool inputError = false; //Used to keep track if there is an error in the input driver circuit
		// for(int port = 0; port < 3; port++) {
		// 	for(int p = 0; p < numPulses; p++) { //Pulse input 5 times
		// 		if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != HIGH) inputError = true; //If the OUTx line is not sitting high, there is an error in the input circuit
		// 		ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, HIGH);
		// 		if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != LOW) inputError = true; //If after toggling the Dx input high, the output does not go low there is an error in the input circuit 
		// 		delay(1);
		// 		ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, LOW);
		// 		delay(1);
		// 	}
		// 	readCounters(); //Read in values after testing
		// 	// clearCount(time);
		// 	if(inputError) { //If the OUTx line did not change as expected when toggled, this is a buffer error
		// 		throwError(INPUT_BUF_ERROR | 0b0100 | port); //OR with port number and Dx indicator 
		// 	}
		// 	else if(counts[port] != numPulses) { //If OUTx line responded as expected, but we STILL did not end up with the correct count, then this is a counter problem 
		// 		throwError(COUNTER_INCREMENT_ERROR | port); 
		// 		// for(int i = 0; i < 3; i++) { //DEBUG!
		// 		// 	Serial.println(counts[i]); 
		// 		// }
		// 	}	
		// 	inputError = false; //Reset input error
		// }
		// clearCount(time); //Clear counters again
		// readCounters(); //Read in values after clearing 
		// if(counts[0] != 0 || counts[1] != 0 || counts[2] != 0) throwError(COUNTER_CLEAR_ERROR); //If counter does not clear correctly, throw error 
		// inputError = false; //Clear error flag for next test 
		
		// for(int i = 0; i < 3; i++) {
		// 	ioBeta.digitalWrite(pinsBeta::D1_SENSE + i, LOW); //Drive all Dx_SENSE lines low to prevent erronious output ticks
		// 	ioBeta.digitalWrite(pinsBeta::OD1 + i, LOW); //Drive all ODx lines low to start
		// }
	
		// for(int port = 0; port < 3; port++) {
		// 	for(int p = 0; p < numPulses; p++) { //Pulse input 5 times
		// 		if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != HIGH) inputError = true; //If the OUTx line is not sitting low, there is an error in the input circuit
		// 		// ioBeta.digitalWrite(pinsBeta::OD1 + port, HIGH);
		// 		ioBeta.pinMode(pinsBeta::OD1 + port, INPUT); //Switch ODx to input to release to pullup (do this instead of push-pull to prevent output shorting)
		// 		if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != LOW) inputError = true; //If after toggling the ODx input high, the output does not go low there is an error in the input circuit //FIX! switch to interrupt measurment for this part
		// 		delay(1);
		// 		// ioBeta.digitalWrite(pinsBeta::OD1 + port, LOW);
		// 		ioBeta.pinMode(pinsBeta::OD1 + port, OUTPUT); //Turn on ODx output to drive low
		// 		delay(1);
		// 	}
		// 	// readCounters(); //Read in values after testing
		// 	if(inputError) { //If the OUTx line did not change as expected when toggled, this is a buffer error
		// 		throwError(INPUT_BUF_ERROR | port); //OR with port number
		// 	}
		// }

		// ///////////// IDENTIFY Dx INPUTS //////////
		// bool digitalInputOccupied[3] = {false}; //Used to store results of digital input testing
		// int digitalInputCurrentState[3] = {0}; //Used to store current state of digital input (0 = LOW, 1 = HIGH, -1 = N/A)

		// bool pullupVal = 0;
		// bool pulldownVal = 0;
		// for(int port = 0; port < 3; port++) {
		// 	ioBeta.pinMode(pinsBeta::D1_SENSE + port, INPUT_PULLUP); //Set given digital input pin as pullup
		// 	delay(1); //Wait for line to charge if floating
		// 	pullupVal = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read state when pullup is applied
		// 	ioBeta.pinMode(pinsBeta::D1_SENSE + port, INPUT_PULLDOWN); //Set given digital input pin as pulldown
		// 	delay(1);
		// 	pulldownVal = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read state when pullup is disconnected 
		// 	if(pullupVal == HIGH && pulldownVal == LOW) digitalInputOccupied[port] = false;
		// 	else digitalInputOccupied[port] = true;

		// 	if(digitalInputOccupied[port] == true) {
		// 		ioBeta.pinMode(pinsBeta::D1_SENSE, INPUT); //Return to high impedance input
		// 		digitalInputCurrentState[port] = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read in current value
		// 	}
		// 	else digitalInputCurrentState[port] = -1; //Set to N/A if the port is unoccupied  
		// }

		// String occupied = "\"Dx_USE\":[";
		// String currentState = "\"Dx_STATE\":[";
		// for(int i = 0; i < 3; i++) {
		// 	occupied = occupied + String(digitalInputOccupied[i]) + ",";
		// 	currentState = currentState + String(digitalInputCurrentState[i]) + ",";
		// }
		// occupied = occupied.substring(0, occupied.length() - 1) + "],"; //Trim off trailing ',' and close
		// currentState = currentState.substring(0, currentState.length() - 1) + "],"; //Trim off trailing ',' and close
		// output = output + occupied + currentState; //Concatonate together


		// for(int i = 0; i < 3; i++) { //Return pins to defaults
		// 	ioBeta.pinMode(pinsBeta::OD1 + i, INPUT); //Return to input	
		// 	ioBeta.pinMode(pinsBeta::D1_SENSE + i, INPUT); //Return to input
		// 	ioBeta.pinMode(pinsBeta::OUT1 + i, INPUT); //Keep as input
		// }
		// clearCount(time); //Clear counters again //Total time between start and this clear event is < 300ms, ok to use same timestamp
		// // Serial.println(millis()); //DEBUG!

		// ////////////// TEST ANALOG OUTPUTS /////////////////
		// float senseOpen[3] = {0};
		// float senseDischarged[3] = {0};
		// float senseLoaded[3] = {0};
		// unsigned long dischargePeriod = 250; //Time to wait while discharging in ms

		// for(int port = 0; port < 3; port++) {
		// 	senseOpen[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read baseline port value at full range
		// 	ioAlpha.pinMode(pinsAlpha::ACTRL1 + port, OUTPUT); //Set MOSFET drive to output
		// 	ioAlpha.digitalWrite(pinsAlpha::ACTRL1 + port, HIGH); //Turn on MOSFET to discharge output
		// 	delay(dischargePeriod); //Wait for RC circuit to discharge
		// 	senseLoaded[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read fully discharged value
		// 	ioAlpha.digitalWrite(pinsAlpha::ACTRL1 + port, LOW); //Turn MOSFET off to release line
		// 	senseDischarged[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read in discharged value
		// }
		// output = output + "\"AIN_SENSE\":{"; //Open array

		// String open = "\"OPEN\":[";
		// String discharged = "\"DIS\":[";
		// String loaded = "\"LOAD\":[";

		// for(int i = 0; i < 3; i++) {
		// 	open = open + String(senseOpen[i], 4) + ",";
		// 	discharged = discharged + String(senseDischarged[i], 4) + ",";
		// 	loaded = loaded + String(senseLoaded[i], 4) + ",";
		// }
		// open = open.substring(0, open.length() - 1) + "],"; //Trim off trailing ',' and close
		// discharged = discharged.substring(0, discharged.length() - 1) + "],"; //Trim off trailing ',' and close
		// loaded = loaded.substring(0, loaded.length() - 1) + "]"; //Trim off trailing ',' and close

		// output = output + open + discharged + loaded + "}},"; //Close AIN_SENSE
		// String level4 = selfDiagnostic(4); //Call the lower level of self diagnostic 
		// level4 = level4.substring(1,level4.length() - 1); //Trim off opening and closing brace
		// output = output + level4; //Concatonate level 4 on top of level 3
		output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

 	}

	if(diagnosticLevel <= 4) {
		// String output = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// output = output.substring(0,output.length() - 1); //Trim off closing brace
		output = output + "\"lvl-4\":{"; //OPEN JSON BLOB

		// ioSense.begin(); //Initalize voltage sensor IO expander
		///////////// SENSE VOLTAGE AND CURRENT FOR PORTS ///////////
		for(int p = 1; p <= numPorts; p++) {
			enablePower(p, true); //Turn on power to all ports before measuring //DEBUG!
		}
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
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
			output = output + "]"; //Close group
		}
		else { //If unable to initialzie ADC
			output = output + "\"PORT_V\":[null],\"PORT_I\":[null]";
			throwError(SENSE_ADC_INIT_FAIL | talonPortErrorCode); //Throw error for ADC failure
		}
		ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Turn MUX back off 
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Return to default external connecton
		// ioAlpha.digitalWrite(pinsAlpha::EN1, HIGH); //Make sure all ports are enabled before testing 
		// ioAlpha.digitalWrite(pinsAlpha::EN2, HIGH); 
		// ioAlpha.digitalWrite(pinsAlpha::EN3, HIGH); 
		// ioAlpha.digitalWrite(pinsAlpha::MUX_EN, LOW); //Turn MUX on 
		// ioAlpha.digitalWrite(pinsAlpha::REG_EN, HIGH); //Turn on power to ADC and any 5V ports
		// adcConfig(0x71, 0x80); //Configure the ADC to read from channel 3 (MUX in) and use full scale range (6.144V) and 128 sps for high speed 
		// float portInputVoltage[3] = {0};
		// float portOutputVoltage[3] = {0};
		// String portInputString = "\"RAIL_IN\":["; //Sub string for input vals
		// String portOutputString = "\"RAIL_OUT\":["; //Sub string for output vals

 		// for (int i = 0; i < 3; i++) {
 		// 	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, (i & 0x01)); //Write low bit of counter to MUX_SEL0
 		// 	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, ((i >> 1) & 0x01)); //Write high bit of counter to MUX_SEL1
		// 	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, LOW); //Read external ports first
		// 	delay(1); //DEBUG!
		// 	portOutputVoltage[i] = float(adcRead(3, 0))*(adcGainConv[0]); //Read from port 3 with no gain, convert to mV
		// 	portOutputString = portOutputString + String(portOutputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
		// 	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); //Read internal ports next
		// 	delay(1); //DEBUG!
		// 	portInputVoltage[i] = float(adcRead(3, 0))*(adcGainConv[0]); //Read from port 3 with no gain, convert to mV
		// 	portInputString = portInputString + String(portInputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
		// 	if((portInputVoltage[i] - portOutputVoltage[i])/portInputVoltage[i] > MAX_DISAGREE) throwError(BUS_DISAGREE | i); //Throw port disagree error and note position of port
		// }
		
		// portInputString = portInputString.substring(0, portInputString.length() - 1) + "],"; //Trim trailing ',' and cap substring
		// portOutputString = portOutputString.substring(0, portOutputString.length() - 1) + "]"; //Trim trailing ',' and cap substring

		// const float max3v3 = 3300*(1 + MAX_DISAGREE/2.0); //Calc ranges for bus values (working in mV!)
		// const float min3v3 = 3300*(1 - MAX_DISAGREE/2.0);
		// const float max5v = 5000*(1 + MAX_DISAGREE/2.0);
		// const float min5v = 5000*(1 - MAX_DISAGREE/2.0);
		// for(int i = 0; i < 3; i++) {
		// 	if(portInputVoltage[i] < max3v3 && portInputVoltage[i] > min3v3) portVoltageSettings[i] = 0; //If within 3v3 range, set port config accordingly 
		// 	else if(portInputVoltage[i] < max5v && portInputVoltage[i] > min5v) portVoltageSettings[i] = 1; //If within the 5v range, set the port config accordingly 
		// 	else {
		// 		if(portInputVoltage[i] < min3v3) portVoltageSettings[i] = 0; //Make assumption about switch position, set config accordingly
		// 		else portVoltageSettings[i] = 1;
		// 		throwError(BUS_OUTOFRANGE | i); //Throw out of range error and note position of port
		// 	}
		// }
		// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, 1); //Connect MUX to 5V rail
		// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, 1);
		// ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); 
		// float busVoltage_5V = float(adcRead(3, 0))*(adcGainConv[0]); //Read 5V port with no gain, convert to mV
		// if((busVoltage_5V - 5000)/5000 > MAX_DISAGREE) throwError(BUS_OUTOFRANGE | 3); //Throw out of range error and note position of port

		// output = output + portInputString + portOutputString + ",\"5V0_RAIL\":" + String(busVoltage_5V, 4) +  "},"; //Concatonate strings and cap
		// // String level5 = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// // level5 = level5.substring(1,level5.length() - 1); //Trim off opening and closing brace
		// // output = output + level5; //Concatonate level 5 on top of level 4
		output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

	}

	if(diagnosticLevel <= 5) {
		output = output + "\"lvl-5\":{"; //OPEN JSON BLOB
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		disableDataAll(); //Turn off all data 
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		for(int i = 0; i < numPorts; i++) {
			// overflow[i] = ioBeta.getInterrupt(pinsBeta::OVF1 + i); //Read in overflow values
			faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
			if (faults[i] == true) {
				throwError(SENSOR_POWER_FAIL | portErrorCode | i); //Throw power fault error with given port appended 
			}
		}

		output = output + "\"ALPHA\":" + String(ioAlpha.readBus()) + ","; //Append ALPHA port readout
		// output = output + "\"BETA\":" + String(ioBeta.readBus()) + ","; //Append BETA port readout
		output = output + "\"ALPHA_INT\":" + String(ioAlpha.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append ALPHA interrupt readout
		uint8_t senseBus = 0; //Manually concatonate pin reads from ioSense expander
		for(int i = 0; i < 4; i++) { //NOT associated with numPorts, which is why variable is not used
			senseBus = senseBus | (ioSense.read(i) << i); //DEBUG!
			// senseBus = 0; //DEBUG!
		}
		output = output + "\"MUX\":" + String(senseBus) + ","; //Append MUX controller readout

		output = output + "\"I2C_OB\":[";
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// digitalWrite(D6, HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, LOW); //Make sure loopback is turned off
		for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
			Wire.beginTransmission(adr);
			// Wire.write(0x00);
			if(Wire.endTransmission() == 0) {
				output = output + String(adr) + ",";
			}
			delay(1); //DEBUG!
		}
		if(output.substring(output.length() - 1).equals(",")) {
			output = output.substring(0, output.length() - 1); //Trim trailing ',' is present
		}
		output = output + "],"; // close array

		output = output + "\"ADRs\":["; //Grab address from each port
		for(int i = 1; i <= numPorts; i++) {
			enableData(i, true); //Turn on data port
			String adr = sendCommand("?!");
			int adrVal = adr.toInt();
			if(adr.equals("") || (adr.equals("0") && adrVal != 0)) output = output + "null"; //If no return, report null
			else output = output + adr; //Otherwise report the read value
			if(i < numPorts) output = output + ","; //Add comma if not last entry
			enableData(i, false); //Turn data port back off
		}
		output = output + "]"; //Close array

		// disableDataAll(); //Make sure all data ports are turned off to begin with
		// for(int port = 1; port <= numPorts; port++) { //CHECK EACH SENSOR PORT FOR ADDRESSES
		// 	output = output + "\"I2C_" + String(port) + "\":["; //Append identifer for
		// 	Serial.print("Enable States: "); //DEBUG! And following prints
		// 	Serial.print(enablePower(port, true)); //Turn on power to given port //DEBUG! REPLACE!
		// 	Serial.println(enableData(port, true)); //Turn on data to given port
		// 	// delay(10);
		// 	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C
		// 	for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
		// 		Wire.beginTransmission(adr);
		// 		// Wire.write(0x00);
		// 		int error = Wire.endTransmission();
		// 		if(adr == 0) { //DEBUG!
		// 			Serial.print("Zero Error: ");
		// 			Serial.println(error); 
		// 		}
		// 		if(error == 0) {
		// 			output = output + String(adr) + ",";
		// 		}
		// 		delay(1); //DEBUG!
		// 	}
		// 	enableData(port, false); //Turn off data to given port
		// 	if(output.substring(output.length() - 1).equals(",")) {
		// 		output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
		// 	}
		// 	output = output + "]"; //Close array
		// 	if(port < numPorts) output = output + ","; //Only add comma if not the last entry in array 
		// }
		output = output + "}"; //Close pair
		
		
		// // output = output + "}"; //CLOSE JSON BLOB, 
		ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		// // ioBeta.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Beta
		// // return output + ",\"Pos\":[" + String(port) + "]}}";
		// // return output;
	}
	return output + ",\"Pos\":[" + getTalonPort() + "]}}"; //Write position in logical form - Return compleated closed output
	// else return ""; //Return empty string if reaches this point 

	// return "{}"; //Return null if reach end	
	// return output + ",\"Pos\":[" + String(port) + "]}}"; //Append position and return
	// return "{}"; //DEBUG!
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
	if(hasReset()) begin(0, hasCriticalError, hasError); //If Talon has been power cycled, run begin function again
	// setPinDefaults(); //Reset IO expander pins to their default state
	bool hasFault = false;
	for(int i = 0; i < numPorts; i++) {
		if(ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i)) {
			throwError(SENSOR_POWER_FAIL | talonPortErrorCode | i); //Throw error because a power failure has occured  
			hasFault = true; //Set flag if any return true
		}
	}
	if(hasFault) { //If there are power faults, reset the system
		ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, LOW); //Turn off sensing
		disablePowerAll(); //Turn off all power  
		disableDataAll(); //Turn off all data
		for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Set fault lines as outputs
			ioAlpha.pinMode(i, OUTPUT); 
			ioAlpha.digitalWrite(i, LOW);
		}

		for(int i = 1; i <= numPorts; i++) { //Enable power
			faults[i - 1] = false; //Reset fault state
			enablePower(i, true); //Turn on power for each port
			if(testOvercurrent()) { //Check if excess current 
				enablePower(i, false); //Turn port back off
				faults[i - 1] = true; //Store which ports have faulted 
				Serial.print("Port Fault: "); //DEBUG!
				Serial.println(i);
			}
		}
		ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH); //Turn sensing back on

		for(int i = 1; i <= numPorts; i++) { //Toggle power to all ports to reset faults
			if(!faults[i - 1]) { //Only toggle back on if no fault
				enablePower(i, true);
				delayMicroseconds(10);
				enablePower(i, false);
				delayMicroseconds(10);
				enablePower(i, true);
			}
		}

		for(int i = pinsAlpha::FAULT1; i <= pinsAlpha::FAULT4; i++) { //Release fault lines
			ioAlpha.pinMode(i, INPUT_PULLUP); 
			// ioAlpha.digitalWrite(i, LOW);
		}
		ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		for(int i = 0; i < numPorts; i++) {
			if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i)) {
				throwError(SENSOR_POWER_FAIL_PERSISTENT | talonPortErrorCode | i); //Throw error because a power failure still present
				hasFault = true; //Set flag if any return true
			}
		}
	}
	return 0; //FIX!
}

String SDI12Talon::getData(time_t time)
{
	// String output = "{\"I2C_TALON\":"; //OPEN JSON BLOB
	String output = "{\"I2C_TALON\":null}"; //DUMMY JSON BLOB
	// const time_t startTime = clearTime; //Grab current clear time //FIX! change to report the time used in calculation
	// const time_t stopTime = time; //Grab the time the current update is made
	// updateCount(time); //Update counter values
	// updateAnalog(); //Update analog readings
	
	// String output = "{\"AUX_TALON\":{"; //OPEN JSON BLOB

	// String analogData = "\"AIN\":[";
	// String analogAvgData = "\"AIN_AVG\":[";
	// String countData = "\"COUNTS\":[";
	// String rateData = "\"RATE\":[";
	// for(int i = 0; i < 3; i++) {
	// 	analogData = analogData + String(analogVals[i], 7) + ",";
	// 	analogAvgData = analogAvgData + String(analogValsAvg[i], 7) + ",";
	// 	countData = countData + String(counts[i]) + ",";
	// 	rateData = rateData + String(rates[i], 7) + ",";
	// }
	// analogData = analogData.substring(0,analogData.length() - 1) + "],"; //Trim trailing ',' and close array
	// analogAvgData = analogAvgData.substring(0,analogAvgData.length() - 1) + "],";
	// countData = countData.substring(0,countData.length() - 1) + "],";
	// rateData = rateData.substring(0,rateData.length() - 1) + "],";

	// output = output + analogData + analogAvgData + countData + rateData; //Concatonate all sub-strings
	// output = output + "\"START\":" + String((long) startTime) + ","; //Concatonate start time
	// output = output + "\"STOP\":" + String((long) stopTime) + ","; //Concatonate stop time
	// output = output + "\"Pos\":[" + String(port) + "]"; //Concatonate position 
	// output = output + "}}"; //CLOSE JSON BLOB
	// return output;
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
	ioAlpha.pinMode(pinsAlpha::DIR, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::DIR, HIGH); //Default to transmit 

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
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN4, LOW); 

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
	Wire.beginTransmission(0x58); //Write to UUID range of EEPROM
	Wire.write(0x98); //Point to start of UUID
	int error = Wire.endTransmission();
	// uint64_t uuid = 0;
	String uuid = "";

	if(error != 0) throwError(EEPROM_I2C_ERROR | error);
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

	String metadata = "{\"Talon-SDI12\":{";
	if(error == 0) metadata = metadata + "\"SN\":\"" + uuid + "\","; //Append UUID only if read correctly, skip otherwise 
	metadata = metadata + "\"Hardware\":\"v" + String(version >> 4, HEX) + "." + String(version & 0x0F, HEX) + "\","; //Report version as modded BCD
	metadata = metadata + "\"Firmware\":\"v" + FIRMWARE_VERSION + "\","; //Report firmware version as modded BCD
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
	metadata = metadata + "}}"; //CLOSE  
	return metadata; 
	return ""; //DEBUG!
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
	ioAlpha.pinMode(pinsAlpha::DATA_EN1 + port - 1, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::DATA_EN1 + port - 1, state);
	if(ioAlpha.digitalRead(pinsAlpha::DATA_EN1 + port - 1) == state) success = true; //If readback matches, set is a success
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

bool SDI12Talon::testOvercurrent()
{
	// bool prevState = digitalRead(KestrelPins::PortBPins[talonPort]); //Check the current state of the OB enable line
	bool prevI2C = digitalRead(KestrelPins::I2C_OB_EN);
	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect I2C to default external I2C 
	digitalWrite(KestrelPins::I2C_OB_EN, HIGH); //Turn on OB I2C bus
	int ADR = 0x14;
	Wire.beginTransmission(ADR);
	Wire.write(0x1F); //Write refresh command
	Wire.write(0x00); //Initilize a clear
	uint8_t error = Wire.endTransmission();
	// delay(1);
	

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
	if(result > 3276 || error != 0) return true; //If current is greater than 500mA, or unable to read current, return true
	else return false; //Otherwise return false 
}