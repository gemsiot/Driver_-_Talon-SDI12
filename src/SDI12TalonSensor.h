#ifndef SDI12TalonSensor_h
#define SDI12TalonSensor_h

#include "SDI12Talon.h"

class SDI12TalonSensor : public Sensor {
protected:
	SDI12Talon& talon;
	SDI12TalonSensor(SDI12Talon& talon);

public:
	// C++ Can't require static methods, but these two are needed
	// static void isPresent(SDI12Talon& talon, uint8_t port)
	// constructor accepting (SDI12Talon& talon, uint8_t port)
	String getMetadata() final;
	virtual const char* firmwareVersion() = 0;
	virtual const char* name() = 0;
	virtual void appendExtraMetadata(String& metadata);
};

struct SDI12TalonSensorFactory {
	bool (*isPresent)(SDI12Talon& talon, uint8_t port);
	SDI12TalonSensor* (*create)(SDI12Talon& talon, uint8_t port);

	template <typename Sensor>
	static SDI12TalonSensor* CreateSensor(SDI12Talon& talon, uint8_t port) {
		// If you see errors here, your sensor is lacking a constructor accepting (SDI12Talon& talon, uint8_t port)
		return new Sensor(talon, port);
	}
	template <typename Sensor>
	static constexpr SDI12TalonSensorFactory Create() {
		// If you see errors here, your sensor is lacking an isPresent(SDI12Talon& talon, uint8_t port)
		bool (*isPresent)(SDI12Talon& talon, uint8_t port) = Sensor::isPresent;
		SDI12TalonSensor* (*create)(SDI12Talon& talon, uint8_t port) = CreateSensor<Sensor>;
		return {isPresent, create};
	}
};

#endif // SDI12TalonSensor_h
