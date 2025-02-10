#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

// IMPORTANT: IF USING AN ACTUAL IMU, COMMENT OUT THE LINE BELOW THIS LINE
#define USE_MOCK_IMU

#ifdef USE_MOCK_IMU
	#define IMU_TYPE MockLSM9DS1
	#pragma message("Mock IMU Implementation enabled")
	#include "MockLSM9DS1.h"
#else
	#include <Adafruit_LSM9DS1.h>
	#define IMU_TYPE Adafruit_LSM9DS1
	#pragma message("Using Real IMU Implementation")
#endif

// Declare (but don't define) the global IMU instance
extern IMU_TYPE imu;

// Optional helper function for initialization with retry
bool initIMU(int maxRetries = 5);

#endif // IMU_INTERFACE_H