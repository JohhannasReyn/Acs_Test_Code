#include "IMUInterface.h"

// Define the global IMU instance
IMU_TYPE imu;

bool initIMU(int maxRetries) {
  #ifdef USE_MOCK_IMU
    return imu.begin(); // Mock always succeeds
  #else
    int retries = 0;
    while (!imu.begin() && retries < maxRetries) {
      Serial.println("IMU initialization failed. Retrying...");
      delay(1000);
      retries++;
    }
    return retries < maxRetries;
  #endif
}