// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// This section generates random values within a realistic range to computer the EKF's processing time ---
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

// #ifndef MOCK_LSM9DS1_H
// #define MOCK_LSM9DS1_H

// #define MAG_RANGE 	50.0 	// the range used for the magnetic field ± μT
// #define GYRO_RANGE 	255.0	// the range used for the gyroscope ± degrees/sec
// #define ACCEL_RANGE 2.0		// the range used for the accelerometer ± m/s^2
// #define GRAVITY   	9.81	// the gravity value in m/s^2
// #define TEMP_RANGE 	10.0	// the range used for the temperature ± °C
// #define TEMP_MID 	25.0 	// the baseline temperature in °C

// #include <Arduino.h>

// struct sensors_vec_t {
//     float x;
//     float y;
//     float z;
// };

// struct sensors_event_t {
//     sensors_vec_t magnetic;
//     sensors_vec_t acceleration;
//     sensors_vec_t gyro;
//     float temperature;
// 	float timestamp;
// };

// class MockLSM9DS1 {
//   private:
//     bool _isInitialized;
// 	float _simulatedTime;
// 	uint8_t _accelRange;
// 	uint8_t _magGain;
// 	uint8_t _gyroScale;

//   public:
//     static const int LSM9DS1_ACCELRANGE_2G = 0;
//     static const int LSM9DS1_MAGGAIN_4GAUSS = 0;
//     static const int LSM9DS1_MAGGAIN_8GAUSS = 1;
//     static const int LSM9DS1_MAGGAIN_12GAUSS = 2;
//     static const int LSM9DS1_GYROSCALE_245DPS = 0;

//     MockLSM9DS1() {
//         _isInitialized = false;
// 		_simulatedTime = 0;
//         randomSeed(analogRead(0));  // Initialize random seed
//     }

//     bool begin() { _isInitialized = true; return true; }
//     void setupAccel(uint8_t range) { _accelRange = range; }
//     void setupMag(uint8_t gain) { _magGain = gain; }
//     void setupGyro(uint8_t scale) { _gyroScale = scale; }
// 	void resetTime() { _simulatedTime = 0.0f; }
// 	float getSimulatedTime() { return _simulatedTime; }
//     float generateRandomFloat(float min, float max) { return min + (random(1000000) / 1000000.0) * (max - min); }

// 	void getEvent(sensors_event_t* accelEvent, sensors_event_t* magEvent, 
//         sensors_event_t* gyroEvent, sensors_event_t* tempEvent) {
//         // Simulate sensor readings based on time
//         _simulatedTime += 0.1; // Increment time by 0.1 seconds

//         // Generate simulated magnetic field readings (oscillating pattern)
//         magEvent->magnetic.x = 20.0 * sin(_simulatedTime * 0.5);
//         magEvent->magnetic.y = 15.0 * cos(_simulatedTime * 0.3);
//         magEvent->magnetic.z = 10.0 * sin(_simulatedTime * 0.7);

//         // Generate simulated gyroscope readings
//         gyroEvent->gyro.x = 5.0 * sin(_simulatedTime);
//         gyroEvent->gyro.y = 3.0 * cos(_simulatedTime);
//         gyroEvent->gyro.z = 4.0 * sin(_simulatedTime * 1.5);

//         // Generate simulated accelerometer readings (including gravity)
//         accelEvent->acceleration.x = 0.1 * sin(_simulatedTime);
//         accelEvent->acceleration.y = 0.1 * cos(_simulatedTime);
//         accelEvent->acceleration.z = 9.81; // Gravity

//         // Simulated temperature
//         tempEvent->temperature = 25.0 + (sin(_simulatedTime * 0.1) * ACCEL_RANGE);
// 		tempEvent->timestamp = _simulatedTime * 1000000; // Simulate timestamp as time elapsed
//     }

//     void getEvent(sensors_event_t* imuEvent) {
//         // Random magnetic field (-50 to 50 μT range)
//         imuEvent->magnetic.x = generateRandomFloat(-MAG_RANGE, MAG_RANGE);
//         imuEvent->magnetic.y = generateRandomFloat(-MAG_RANGE, MAG_RANGE);
//         imuEvent->magnetic.z = generateRandomFloat(-MAG_RANGE, MAG_RANGE);

//         // Random gyro (-245 to 245 degrees/sec range)
//         imuEvent->gyro.x = generateRandomFloat(-GYRO_RANGE, GYRO_RANGE);
//         imuEvent->gyro.y = generateRandomFloat(-GYRO_RANGE, GYRO_RANGE);
//         imuEvent->gyro.z = generateRandomFloat(-GYRO_RANGE, GYRO_RANGE);

//         // Random acceleration (-2 to 2 g range, except Z which includes gravity)
//         imuEvent->acceleration.x = generateRandomFloat(-ACCEL_RANGE, ACCEL_RANGE);
//         imuEvent->acceleration.y = generateRandomFloat(-ACCEL_RANGE, ACCEL_RANGE);
//         imuEvent->acceleration.z = generateRandomFloat(GRAVITY-ACCEL_RANGE, GRAVITY+ACCEL_RANGE); // 9.81 ± 2

//         // Random temperature (15°C to 35°C range)
//         imuEvent->temperature = generateRandomFloat(TEMP_MID - TEMP_RANGE, TEMP_MID + TEMP_RANGE);
//     }

// };

// typedef MockLSM9DS1 Adafruit_LSM9DS1;

// #endif // MOCK_LSM9DS1_H

// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// The code below uses trigonometric functions to generate values in order to simulate smooth rotation ---
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

#ifndef MOCK_LSM9DS1_H
#define MOCK_LSM9DS1_H

#define MAG_RANGE 	50.0 	// the range used for the magnetic field ± μT
#define GYRO_RANGE 	255.0	// the range used for the gyroscope ± degrees/sec
#define ACCEL_RANGE 2.0		// the range used for the accelerometer ± m/s^2
#define GRAVITY   	9.81	// the gravity value in m/s^2
#define TEMP_RANGE 	10.0	// the range used for the temperature ± °C
#define TEMP_MID 	25.0 	// the baseline temperature in °C

#include <Arduino.h>

// Recreate necessary structs/classes from Adafruit library
struct sensors_vec_t {
    float x;
    float y;
    float z;
};

struct sensors_event_t {
    sensors_vec_t magnetic;
    sensors_vec_t acceleration;
    sensors_vec_t gyro;
    float temperature;
};

class MockLSM9DS1 {
public:
    // Constants to match Adafruit library
    static const int LSM9DS1_ACCELRANGE_2G = 0;
    static const int LSM9DS1_MAGGAIN_4GAUSS = 0;
    static const int LSM9DS1_MAGGAIN_8GAUSS = 1;
    static const int LSM9DS1_MAGGAIN_12GAUSS = 2;
    static const int LSM9DS1_GYROSCALE_245DPS = 0;

    MockLSM9DS1() {
        _isInitialized = false;
        _simulatedTime = 0;
    }

    bool begin() {
        _isInitialized = true;
        return true;
    }

    void setupAccel(uint8_t range) {
        _accelRange = range;
    }

    void setupMag(uint8_t gain) {
        _magGain = gain;
    }

    void setupGyro(uint8_t scale) {
        _gyroScale = scale;
    }

    void getEvent(sensors_event_t* accelEvent, sensors_event_t* magEvent, 
                  sensors_event_t* gyroEvent, sensors_event_t* tempEvent) {
        // Simulate sensor readings based on time
        _simulatedTime += 0.1; // Increment time by 0.1 seconds

        // Generate simulated magnetic field readings (oscillating pattern)
        magEvent->magnetic.x = 20.0 * sin(_simulatedTime * 0.5);
        magEvent->magnetic.y = 15.0 * cos(_simulatedTime * 0.3);
        magEvent->magnetic.z = 10.0 * sin(_simulatedTime * 0.7);

        // Generate simulated gyroscope readings
        gyroEvent->gyro.x = 5.0 * sin(_simulatedTime);
        gyroEvent->gyro.y = 3.0 * cos(_simulatedTime);
        gyroEvent->gyro.z = 4.0 * sin(_simulatedTime * 1.5);

        // Generate simulated accelerometer readings (including gravity)
        accelEvent->acceleration.x = 0.1 * sin(_simulatedTime);
        accelEvent->acceleration.y = 0.1 * cos(_simulatedTime);
        accelEvent->acceleration.z = 9.81; // Gravity

        // Simulated temperature
        tempEvent->temperature = 25.0 + (sin(_simulatedTime * 0.1) * ACCEL_RANGE);
    }

private:
    bool _isInitialized;
    uint8_t _accelRange;
    uint8_t _magGain;
    uint8_t _gyroScale;
    float _simulatedTime;
};

// Create a typedef to make it easy to switch between real and mock implementations
typedef MockLSM9DS1 Adafruit_LSM9DS1;

#endif // MOCK_LSM9DS1_H


// -----------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------
// Implementation below this point reads values from a csv file: [pwm, mag_x, mag_y, mag_z, voltage] ---
// -----------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------

// #ifndef MOCK_LSM9DS1_H
// #define MOCK_LSM9DS1_H

// #include <Arduino.h>
// #include <SD.h>

// struct sensors_vec_t {
//     float x;
//     float y;
//     float z;
// };

// struct sensors_event_t {
//     sensors_vec_t magnetic;
//     sensors_vec_t acceleration;
//     sensors_vec_t gyro;
//     float temperature;
// };

// class MockLSM9DS1 {
// public:
//     static const int LSM9DS1_ACCELRANGE_2G = 0;
//     static const int LSM9DS1_MAGGAIN_4GAUSS = 0;
//     static const int LSM9DS1_MAGGAIN_8GAUSS = 1;
//     static const int LSM9DS1_MAGGAIN_12GAUSS = 2;
//     static const int LSM9DS1_GYROSCALE_245DPS = 0;

//     MockLSM9DS1() {
//         _isInitialized = false;
//         _currentIndex = 0;
//         _dataCount = 0;
//     }

//     bool begin() {
//         if (!SD.begin(BUILTIN_SDCARD)) {
//             return false;
//         }
        
//         File dataFile = SD.open("../test_random_voltage.txt");
//         if (!dataFile) {
// 			Serial.print("Failed to open data file: ");
// 			SD.
//             return false;
//         }

//         while (dataFile.available() && _dataCount < MAX_READINGS) {
//             String line = dataFile.readStringUntil('\n');
            
//             // Parse the CSV line
//             int startIdx = 0;
//             int commaIdx = line.indexOf(',', startIdx);
            
//             if (commaIdx > 0) {
//                 _pwmData[_dataCount] = line.substring(startIdx, commaIdx).toFloat();
//                 startIdx = commaIdx + 1;
                
//                 // Parse mag_x
//                 commaIdx = line.indexOf(',', startIdx);
//                 _magX[_dataCount] = line.substring(startIdx, commaIdx).toFloat();
//                 startIdx = commaIdx + 1;
                
//                 // Parse mag_y
//                 commaIdx = line.indexOf(',', startIdx);
//                 _magY[_dataCount] = line.substring(startIdx, commaIdx).toFloat();
//                 startIdx = commaIdx + 1;
                
//                 // Parse mag_z
//                 commaIdx = line.indexOf(',', startIdx);
//                 _magZ[_dataCount] = line.substring(startIdx, commaIdx).toFloat();
//                 startIdx = commaIdx + 1;
                
//                 // Parse voltage
//                 commaIdx = line.indexOf(',', startIdx);
//                 _voltageData[_dataCount] = line.substring(startIdx, commaIdx).toFloat();
                
//                 _dataCount++;
//             }
//         }
        
//         dataFile.close();
//         _isInitialized = (_dataCount > 0);
//         return _isInitialized;
//     }

//     void setupAccel(uint8_t range) { _accelRange = range; }
//     void setupMag(uint8_t gain) { _magGain = gain; }
//     void setupGyro(uint8_t scale) { _gyroScale = scale; }

//     void getEvent(sensors_event_t* accelEvent, sensors_event_t* magEvent, 
//                   sensors_event_t* gyroEvent, sensors_event_t* tempEvent) {
//         if (!_isInitialized || _dataCount == 0) return;

//         // Use the loaded magnetic field readings
//         magEvent->magnetic.x = _magX[_currentIndex];
//         magEvent->magnetic.y = _magY[_currentIndex];
//         magEvent->magnetic.z = _magZ[_currentIndex];

//         // Keep original simulated values for other sensors
//         gyroEvent->gyro.x = 5.0 * sin(_currentIndex * 0.1);
//         gyroEvent->gyro.y = 3.0 * cos(_currentIndex * 0.1);
//         gyroEvent->gyro.z = 4.0 * sin(_currentIndex * 0.15);

//         accelEvent->acceleration.x = 0.1 * sin(_currentIndex * 0.1);
//         accelEvent->acceleration.y = 0.1 * cos(_currentIndex * 0.1);
//         accelEvent->acceleration.z = 9.81;

//         tempEvent->temperature = 25.0 + (sin(_currentIndex * 0.01) * ACCEL_RANGE);

//         _currentIndex = (_currentIndex + 1) % _dataCount;
//     }

//     float getCurrentVoltage() {
//         return _isInitialized ? _voltageData[_currentIndex] : 0.0f;
//     }

//     float getCurrentPWM() {
//         return _isInitialized ? _pwmData[_currentIndex] : 0.0f;
//     }

// private:
//     static const int MAX_READINGS = 1000;  // Maximum number of readings to store
//     bool _isInitialized;
//     uint8_t _accelRange;
//     uint8_t _magGain;
//     uint8_t _gyroScale;
    
//     // Arrays to store file data
//     float _pwmData[MAX_READINGS];
//     float _magX[MAX_READINGS];
//     float _magY[MAX_READINGS];
//     float _magZ[MAX_READINGS];
//     float _voltageData[MAX_READINGS];
    
//     int _currentIndex;
//     int _dataCount;
// };

// typedef MockLSM9DS1 Adafruit_LSM9DS1;

// #endif // MOCK_LSM9DS1_H