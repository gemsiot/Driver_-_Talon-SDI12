#include "SDI12TalonSensor.h"

SDI12TalonSensor::SDI12TalonSensor(SDI12Talon& talon) : talon(talon)
{
	sensorInterface = BusType::SDI12;
}

String SDI12TalonSensor::getMetadata() {
	uint8_t adr = (talon.sendCommand("?!")).toInt();
	String id = talon.command("I", adr);
	Serial.println(id); // DEBUG!
	String sdi12Version;
	String mfg;
	String model;
	String senseVersion;
	String sn;
	if ((id.substring(0, 1)).toInt() != adr) { //If address returned is not the same as the address read, throw error
		Serial.println("ADDRESS MISMATCH!"); //DEBUG!
		//Throw error!
		sdi12Version = "null";
		mfg = "null";
		model = "null";
		senseVersion = "null";
		sn = "null";
	} else {
		sdi12Version = (id.substring(1,3)).trim(); //Grab SDI-12 version code
		mfg = (id.substring(3, 11)).trim(); //Grab manufacturer
		model = (id.substring(11,17)).trim(); //Grab sensor model name
		senseVersion = (id.substring(17,20)).trim(); //Grab version number
		sn = (id.substring(20,33)).trim(); //Grab the serial number
	}
	String metadata = String("{\"") + name() + "\":{";
	metadata = metadata + "\"Hardware\":\"" + senseVersion + "\",";
	metadata = metadata + "\"Firmware\":\"" + firmwareVersion() + "\",";
	metadata = metadata + "\"SDI12_ver\":\"" + sdi12Version.substring(0,1) + "." + sdi12Version.substring(1,2) + "\",";
	metadata = metadata + "\"ADR\":" + String(adr) + ",";
	metadata = metadata + "\"Mfg\":\"" + mfg + "\",";
	metadata = metadata + "\"Model\":\"" + model + "\",";
	metadata = metadata + "\"SN\":\"" + sn + "\",";
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position
	appendExtraMetadata(metadata); // Any sensor-specific metdata
	metadata = metadata + "}}"; // CLOSE
	return metadata;
}

void SDI12TalonSensor::appendExtraMetadata(String& metadata) {}
