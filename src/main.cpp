#include "constants/constants.h"
#include "ekf/CalibratedEKF.hpp"
#include "IMUInterface.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <SD.h>
#include <Wire.h>
#include <unity.h>
#include <chrono>
#include "DataLogging.hpp"

// Pins for all inputs, keep in mind the PWM defines must be on PWM
// pins, 2.29.24 Note that some of the pins were re-arranged to make a cleaner
// setup For breadboard #define AIN1 31 #define AIN2 32 #define PWMA 30 #define
// STBY 29 #define LED 13

// For updated breadboard (12.03.2024)
#define STBY 32
#define AIN1 31
#define AIN2 30
#define PWMA 29
#define LED 13


static const String FILENAME_RSO = "satellite_data_clean";// RSO = Random + Softiron Offset
static const String FILENAME_RSO_EKF = "satellite_data_dirty";

// For CubeSat
// For x-magnetorquer
// #define AIN1 25
// #define AIN2 24
// #define PWMA 10
// #define STBY 9

// For y-magnetorquer
// #define AIN1 8
// #define AIN2 7
// #define PWMA 6
// #define STBY 9

// For z-magnetorquer
// #define AIN1 28
// #define AIN2 29
// #define PWMA 30
// #define STBY 27
// #define CSAG 21
// #define CSM 20
// #define SHDN 31
// #define RBSLEEP 19
// #define BURN1 14
// #define BURN2 15

// void setUp(void) {
//     // Initialize EKF before each test
//     Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(STATE_SIZE);
//     Eigen::MatrixXd initial_covariance =
//     Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1; Eigen::MatrixXd
//     process_noise = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
//     0.001; Eigen::MatrixXd measurement_noise =
//     Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01; Eigen::MatrixXd
//     H = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

//     ekf.initialize(dt, initial_state, initial_covariance, process_noise,
//     measurement_noise, H, 3.6f);
// }

// Structure to hold measurement data
struct MeasurementData {
  double pwm;
  double mag_x, mag_y, mag_z;
  double gyro_x, gyro_y, gyro_z;
  double voltage;
};


// Helper function to write measurement data
void writeMeasurementData(MeasurementData data, String filename) {
    // Create array with proper C++ syntax
    double values[8] = {
        data.pwm, 
        data.mag_x, 
        data.mag_y, 
        data.mag_z, 
        data.gyro_x, 
        data.gyro_y, 
        data.gyro_z, 
        data.voltage
    };
    // Use DataLog instead of dataLogger
    DataLog(values, 8, filename);
}

// // Helper function to parse a line of CSV data
// MeasurementData parseLine(const String &line) {
//   MeasurementData data = {0}; // Initialize all fields to 0
//   int start = 0;
//   int end = line.indexOf(',');
//   int index = 0;

//   while (end >= 0) {
//     String value = line.substring(start, end);
//     value.trim();
//     double parsed = value.toFloat();

//     switch (index) {
//     case 0:
//       data.pwm = parsed;
//       break;
//     case 1:
//       data.mag_x = parsed;
//       break;
//     case 2:
//       data.mag_y = parsed;
//       break;
//     case 3:
//       data.mag_z = parsed;
//       break;
//     case 4:
//       data.gyro_x = parsed;
//       break;
//     case 5:
//       data.gyro_y = parsed;
//       break;
//     case 6:
//       data.gyro_z = parsed;
//       break;
//     case 7:
//       data.voltage = parsed;
//       break;
//     }

//     start = end + 1;
//     end = line.indexOf(',', start);
//     index++;
//   }

//   // Handle last value if not empty
//   if (start < line.length()) {
//     String value = line.substring(start);
//     value.trim();
//     data.voltage = value.toFloat();
//   }

//   return data;
// }

class IMUProcessor {
private:
  	CalibratedEKF ekf;
  	const double dt = 0.01; // 10ms sample time

  	void initializeEKF() {
  	  	Eigen::VectorXd initial_state(6);
  	  	initial_state << 0, 0, 0, 0, 0, 0;

  	  	Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;
  	  	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);
  	  	Q.topLeftCorner(3, 3) *= 0.01;
  	  	Q.bottomRightCorner(3, 3) *= 0.01;

  	  	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6);
  	  	R.topLeftCorner(3, 3) *= 0.1;
  	  	R.bottomRightCorner(3, 3) *= 0.1;

  	  	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);

  	  	ekf.initialize(dt, initial_state, initial_covariance, Q, R, H, 3.8);
  	}

public:
  	IMUProcessor() { initializeEKF(); }

  	void processMeasurement(const MeasurementData &data, Eigen::VectorXd& filtered_output) {
    	ekf.updateVoltage(data.voltage);

    	// Print raw measurements
    	Serial.println("Raw Measurements:");
    	Serial.print("PWM: ");
    	Serial.println(data.pwm);
    	Serial.print("Magnetic Field (X,Y,Z): ");
    	Serial.print(data.mag_x);
    	Serial.print(", ");
    	Serial.print(data.mag_y);
    	Serial.print(", ");
    	Serial.println(data.mag_z);
    	Serial.print("Angular Velocity (X,Y,Z): ");
    	Serial.print(data.gyro_x);
    	Serial.print(", ");
    	Serial.print(data.gyro_y);
    	Serial.print(", ");
    	Serial.println(data.gyro_z);
    	Serial.print("Voltage: ");
    	Serial.println(data.voltage);

    	// Update measurement vector
    	ekf.Z << data.mag_x, data.mag_y, data.mag_z, data.gyro_x, data.gyro_y,
    	    data.gyro_z;

    	// Process measurement
    	ekf.step();

    	// Get filtered state
    	filtered_output = ekf.state;

    	// Print filtered measurements
    	Serial.println("\nFiltered Measurements:");
    	Serial.print("Magnetic Field (X,Y,Z): ");
    	Serial.print(filtered_output(0) - data.mag_x);
    	Serial.print(", ");
    	Serial.print(filtered_output(1) - data.mag_y);
    	Serial.print(", ");
    	Serial.println(filtered_output(2) - data.mag_z);
    	Serial.print("Angular Velocity (X,Y,Z): ");
    	Serial.print(filtered_output(3) - data.gyro_x);
    	Serial.print(", ");
    	Serial.print(filtered_output(4) - data.gyro_y);
    	Serial.print(", ");
    	Serial.println(filtered_output(5) - data.gyro_z);

    	// Print error metrics
    	Serial.print("Innovation Magnitude: ");
    	Serial.println(ekf.getLastInnovationMagnitude());
    	Serial.print("Prediction Error: ");
    	Serial.println(ekf.getLastPredictionError());
		Serial.println("Divergence:");
		Serial.print("Magnetic Field (X,Y,Z): ");
    	Serial.print(filtered_output(0) - data.mag_x);
    	Serial.print(", ");
    	Serial.print(filtered_output(1) - data.mag_y);
    	Serial.print(", ");
    	Serial.println(filtered_output(2) - data.mag_z);
    	Serial.print("Angular Velocity (X,Y,Z): ");
    	Serial.print(filtered_output(3) - data.gyro_x);
    	Serial.print(", ");
    	Serial.print(filtered_output(4) - data.gyro_y);
    	Serial.print(", ");
    	Serial.println(filtered_output(5) - data.gyro_z);
    	if (ekf.getLastError() != EKFError::OK) {
    	  	// Serial.print("Error Status: ");
    	  	// Serial.println(static_cast<int>(ekf.getLastError()));
    	}

    	Serial.println("----------------------------------------");
  	}
};

// Test case for parsing CSV data
#define file_name "test"

// Global variables
const int STATE_SIZE = 6;
const int CS_PIN = BUILTIN_SDCARD;

#ifndef RUN_TESTS // Only compile this if we're not running tests

void setup() {
  	// Use magnetorquer, gyro, magnetometer
  	String test_name = "softiron_testing";

  	pinMode(LED, OUTPUT);
  	digitalWrite(LED, LOW);  // LED off initially
	
  	Serial.begin(9600);
	delay(2000);  // Give time for serial connection
  	unsigned long startTime = millis();
  	const unsigned long timeout = 5000;
	
  	while (!Serial && (millis() - startTime < timeout)) {
  		digitalWrite(LED, !digitalRead(LED));  // Toggle LED while waiting
  		delay(100);
  	}
	
  	digitalWrite(LED, HIGH);  // LED on when proceeding

  	Serial.print("Starting ");
	Serial.print(test_name);
	Serial.println("...");

  	// Setup pins
  	pinMode(AIN1, OUTPUT);
  	pinMode(AIN2, OUTPUT);
  	pinMode(PWMA, OUTPUT);
  	pinMode(STBY, OUTPUT);
  	digitalWrite(STBY, HIGH);
  	pinMode(LED, OUTPUT);
  	digitalWrite(LED, HIGH);

  	// Extra pins for CubeSat
  	// pinMode(SHDN, OUTPUT);
  	// pinMode(RBSLEEP, OUTPUT);
  	// pinMode(BURN1, OUTPUT);
  	// pinMode(BURN2, OUTPUT);

  	// analogWrite(PWMA, 0);

  	// Extra set pins for CubeSat
  	// digitalWrite(SHDN, LOW);
  	// digitalWrite(RBSLEEP, LOW);
  	// digitalWrite(BURN1, LOW);
  	// digitalWrite(BURN2, LOW);
  	// analogWrite(PWMA, 0);
  	// delay(60000);

  	if (test_name == "nothing") {
  		Serial.println("Do nothing");
  		delay(10000);
  	}

  	if (test_name == "softiron_testing") {///////////////////////////////////////////////////////////////// SOFTIRON TESTING

  	  	// Setup files to store values.
		DataLogSetup(FILENAME_RSO);// Random values + softiron offsets.
		DataLogSetup(FILENAME_RSO_EKF);// Random values + softiron offsets filtered through the EKF.

  	  	Serial.println("Initializing the IMU...");

  	  	// Initialize IMU processor and data logging
  	  	IMUProcessor processor;

  	  	// Initialize IMU
  	  	if (!initIMU()) {
  	    	Serial.println("Failed to initialize IMU after multiple retries");
  	    	// Attempts to connect to IMU are done in IMUInterface
  	    	return;// Handle failure case
  	  	}

  	  	// float mag_hardiron_x = -4.9547000000000025;
  	  	// float mag_hardiron_y = 49.75155;
  	  	// float mag_hardiron_z = -13.855600000000003;
  	  	// float pwmY_ox_1 = 8.83096680e-03;
  	  	// float pwmY_ox_2 = 4.26409072e-07;
  	  	// float pwmY_ox_3 = -6.69370023e-09;
  	  	// float pwmY_oy_1 = -2.64514092e-01;
  	  	// float pwmY_oy_2 = -9.82458813e-06;
  	  	// float pwmY_oy_3 = 9.11136691e-08;
  	  	// float pwmY_oz_1 = -1.90567242e-02;
  	  	// float pwmY_oz_2 = -5.99945842e-06;
  	  	// float pwmY_oz_3 = 7.85718685e-10;

  	  	// Configure IMU settings
  	  	imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  	  	imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
  	  	imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  	  	sensors_event_t accel, mag, gyro, temp;
  	  	delay(1000);

        long long total_offset_time = 0, total_filter_time = 0;
		int count = 0;
		// Timestamp 1 set before generating any imu values
		auto start = std::chrono::high_resolution_clock::now();
  	  	for (int i = -255; i < 255; ++i) {
  	    	// Serial.println(pwm_num);
  	    	int pwm_num = (rand() % 511) - 255; // Generating random integer [-255,255]

			// set timestamp 1
	  		auto t1 = std::chrono::high_resolution_clock::now();
  	    	
			if (pwm_num < 0) {
  	    	  	digitalWrite(AIN1, LOW);
  	    	  	digitalWrite(AIN2, HIGH);
  	    	  	analogWrite(PWMA, -pwm_num);
  	    	} else {
  	    	  	digitalWrite(AIN1, HIGH);
  	    	  	digitalWrite(AIN2, LOW);
  	    	  	analogWrite(PWMA, pwm_num);
  	    	}
			imu.getEvent(&accel,&mag,&gyro,&temp);

            // Used to read voltage
  	   		int voltage_value_pin = 23; // For the Breadboard
  	   		float voltage_ref = 3.3f;
  	   		int resolution = 1023;
  	   		int r1 = 4700;
  	   		int r2 = 10000;
			
			MeasurementData iron_offset;
			iron_offset.pwm = pwm_num;
			iron_offset.mag_x = mag.magnetic.x - constants::imu::mag_hardiron_x;
			iron_offset.mag_y = mag.magnetic.y - constants::imu::mag_hardiron_y;
			iron_offset.mag_z = mag.magnetic.z - constants::imu::mag_hardiron_z;
			iron_offset.gyro_x = gyro.gyro.x - constants::imu::gyro_hardiron_x;
			iron_offset.gyro_y = gyro.gyro.y - constants::imu::gyro_hardiron_y;
			iron_offset.gyro_z = gyro.gyro.z - constants::imu::gyro_hardiron_z;
			iron_offset.voltage = analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;

            // set timestamp 2, add time to sum
			auto t2 = std::chrono::system_clock::now();
			auto diff_t21 = t2 - t1;

			Serial.println("\nReducing generated values with hardiron offsets...");
			
			// Log data to file #1
			double offset_values[8] = {
                iron_offset.pwm,
                iron_offset.mag_x,
                iron_offset.mag_y,
                iron_offset.mag_z,
                iron_offset.gyro_x,
                iron_offset.gyro_y,
                iron_offset.gyro_z,
                iron_offset.voltage
            };
			DataLog(offset_values, 8, FILENAME_RSO);

			//Serial.println("\nPassing data through Calibrated EKF...");
			Eigen::VectorXd filtered_output;
            // Process measurement through EKF
  	    	processor.processMeasurement(iron_offset,filtered_output);// set timestamp 3, add time to sum
			const auto t3 = std::chrono::system_clock::now();
			auto diff_t32 = t3 - t2;

			// Serial.println("\nLogging filtered values...");
			// Log data to file #2
			double filtered_values[8] = {
                iron_offset.pwm,
                filtered_output(0),
                filtered_output(1),
                filtered_output(2),
                filtered_output(3),
                filtered_output(4),
                filtered_output(5),
                iron_offset.voltage
            };
            DataLog(filtered_values, 8, FILENAME_RSO_EKF);
			// Serial.println("\n----------------------------------------");
			
			total_offset_time += std::chrono::duration_cast<std::chrono::milliseconds>(diff_t21).count();
        	total_filter_time += std::chrono::duration_cast<std::chrono::milliseconds>(diff_t32).count();
			++count;

        	Serial.print("Offsets calculation: ");
        	Serial.print(total_offset_time);
        	Serial.print(" ms, ");
	
        	Serial.print("EKF calculation: ");
        	Serial.print(total_filter_time);
        	Serial.println(" ms.");
  	    	delay(100);
  	  	}
		const auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		double actual_time = static_cast<double>(duration.count() - (100.0*count));
        double per_offset_processed = ((double)total_offset_time / actual_time) * 100.0;
		double per_ekf_processed = ((double)total_filter_time / actual_time) * 100.0;
		Serial.println("\nTest completed.");
		Serial.print(count);
        Serial.print(" calculation took ");
        Serial.print(duration.count());
        Serial.print(" ms to process (with ");
        Serial.print(100*count);
		Serial.println(" ms of delay time)");
        // Calculate average processing times
		Serial.print(actual_time);
		Serial.println(" ms actually spent processing data.");
        Serial.println("Efficiency as a percentage: ");
		if (per_offset_processed >= 1.0 || per_ekf_processed >= 1.0) {
        	Serial.print(per_offset_processed);
        	Serial.println("% processing offset adjusted values.");
			Serial.print(per_ekf_processed);
			Serial.println("% of processing values through the EKF.");
		} else {
			if (per_offset_processed < 1.0) {
				Serial.println("Processing values by adjusting their offsets took less than 1.0% of total processing time.");
			} else {
				Serial.print(per_offset_processed);
        		Serial.println("% processing offset adjusted values.");
			}
			if (per_ekf_processed < 1.0) {
				Serial.println("Processing values with the EKF took less than 1.0% of total processing time.");
				Serial.println("The time difference is most likely from a combination of generating random values, and returning text via Serial.");
			} else {
				Serial.print(per_ekf_processed);
				Serial.println("% of processing values through the EKF.");
			}
		}
  	  	digitalWrite(LED, LOW);
  	}

  	if (test_name == "magnetorquer") {
  	  	Serial.println("turned on");
  	  	digitalWrite(AIN1, HIGH);
  	  	digitalWrite(AIN2, LOW);

  	  	analogWrite(PWMA, 255);

  	  	delay(4000);

  	  	analogWrite(PWMA, 0);
  	  	digitalWrite(AIN1, LOW);
  	  	digitalWrite(AIN2, LOW);
  	  	Serial.println("turned off");
  	}

  	if (test_name == "magnetometer") {
  	  	if (!imu.begin()) {
  	  	  	while (1) {
  	  	  	 	Serial.println("wrong");
  	  	  	 	delay(100);
  	  	  	};
  	  	}

  	  	Serial.println("Setting up imu9DS1 9DOF");
  	  	imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  	  	imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
  	  	imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  	  	sensors_event_t accel, mag, gyro, temp;

  	  	for (int i = 0; i < 25; i++) {
  	  	  	imu.getEvent(&accel,&mag,&gyro,&temp);
  	  	  	Serial.println(mag.magnetic.z);
  	  	  	delay(100);
  	  	}

  	  	Serial.println("turned on");

  	  	digitalWrite(AIN1, LOW);
  	  	digitalWrite(AIN2, HIGH);

  	  	analogWrite(PWMA, 255);
  	  	for (int i = 25; i < 75; i++) {
  	  	  	imu.getEvent(&accel,&mag,&gyro,&temp);
  	  	  	Serial.println(mag.magnetic.z);
  	  	  	delay(100);
  	  	}

  	  	Serial.println("turned off");
  	  	analogWrite(PWMA, 0);

  	  	for (int i = 75; i < 100; i++) {
  	  	  	imu.getEvent(&accel,&mag,&gyro,&temp);
  	  	  	Serial.println(mag.magnetic.z);
  	  	  	delay(100);
  	  	}

  	  	digitalWrite(LED, LOW);
  	}

  	if (test_name == "gyro") {
  	  	if (!imu.begin()) {
  	  	  	while (1) {
  	  	  	  	Serial.println("wrong");
  	  	  	  	delay(100);
  	  	  	};
  	  	}

  	  	Serial.println("Setting up imu9DS1 9DOF");
  	  	imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  	  	imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
  	  	imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

        sensors_event_t accel, mag, gyro, temp;

  	  	for (int i = 0; i < 50; i++) {
  	  	  	imu.getEvent(&accel, &mag, &gyro, &temp);
  	  	  	Serial.println(gyro.gyro.z);
  	  	  	delay(100);
  	  	}
  	}

  	if (test_name == "light_test") {
  	  	for (int i = 0; i < 5; i++) {
  	  	  	digitalWrite(LED, LOW);
  	  	  	delay(1000);
  	  	  	digitalWrite(LED, HIGH);
  	  	  	delay(1000);
  	  	}
  	}

  	digitalWrite(STBY, LOW);
  	digitalWrite(LED, LOW);
}


void loop() {
    // static uint32_t lastPrint = 0;
    // static uint32_t loopCount = 0;
    
    // // Add watchdog reset if you're using it
    // // watchdog.reset();
    
    // // Process sensor data
    // if (processNextDataPoint()) {
    //     loopCount++;
        
    //     // Print debug info every second
    //     if (millis() - lastPrint > 1000) {
    //         Serial.print("Loop count: ");
    //         Serial.print(loopCount);
    //         Serial.print(", Free memory: ");
    //         Serial.println(FreeRam());  // You'll need to implement FreeRam()
    //         lastPrint = millis();
    //     }
    // } else {
    //     Serial.println("Failed to process data point");
    //     delay(1000);  // Prevent spam if there's an error
    // }
    
    // // Add a small delay to prevent overwhelming the system
    // delay(10);
}

#endif // RUN_TESTS