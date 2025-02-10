#if defined(ARDUINO) && defined(UNIT_TEST)

#include <unity.h>
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <SD.h>
#include "../Include/ekf/CalibratedEKF.hpp"
#include "../Include/constants/constants.h"

// Test fixtures
CalibratedEKF ekf;
const float dt = 0.01f;  // 10ms sample time
const int STATE_SIZE = 6;
const int CS_PIN = BUILTIN_SDCARD;  // SD card chip select pin

// Structure to hold measurement data
struct MeasurementData {
    float pwm;
    float mag_x, mag_y, mag_z;
    float gyro_x, gyro_y, gyro_z;
    float voltage;
};

void setUp(void) {
    // Initialize EKF before each test
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.001;
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01;
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    
    ekf.initialize(dt, initial_state, initial_covariance, process_noise, measurement_noise, H, 3.6f);
}


void tearDown(void) {
    // Clean up after each test if needed
}

// Helper function to parse a line of CSV data
MeasurementData parseLine(const String& line) {
    MeasurementData data;
    int start = 0;
    int end = line.indexOf(',');
    int index = 0;
    
    while (end >= 0) {
        String value = line.substring(start, end);
        value.trim();
        float parsed = value.toFloat();
        
        switch(index) {
            case 0: data.pwm = parsed; break;
            case 1: data.mag_x = parsed; break;
            case 2: data.mag_y = parsed; break;
            case 3: data.mag_z = parsed; break;
            case 4: data.gyro_x = parsed; break;
            case 5: data.gyro_y = parsed; break;
            case 6: data.gyro_z = parsed; break;
            case 7: data.voltage = parsed; break;
        }
        
        start = end + 1;
        end = line.indexOf(',', start);
        index++;
    }
    
    // Handle last value if not empty
    if (start < static_cast<int>(line.length())) {
        String value = line.substring(start);
        value.trim();
        data.voltage = value.toFloat();
    }
    
    return data;
}

void test_ekf_with_real_data() {
    if (!SD.begin(CS_PIN)) {
        TEST_FAIL_MESSAGE("Failed to initialize SD card");
        return;
    }
    
    File dataFile = SD.open("bread_voltimu_3.6.txt", FILE_READ);
    if (!dataFile) {
        TEST_FAIL_MESSAGE("Failed to open data file");
        return;
    }
    
    bool header_skipped = false;
    String line;
    float total_mag_error = 0.0f;
    float total_gyro_error = 0.0f;
    int measurement_count = 0;
    
    while (dataFile.available()) {
        line = dataFile.readStringUntil('\n');
        line.trim();
        
        // Skip header or empty lines
        if (!header_skipped) {
            if (line.startsWith("---START---")) {
                header_skipped = true;
            }
            continue;
        }
        if (line.length() == 0) continue;
        
        // Parse data line
        MeasurementData measurement = parseLine(line);
        
        // Update EKF parameters
        ekf.updateVoltage(measurement.voltage);
        
        // Create measurement vector
        Eigen::VectorXd Z(STATE_SIZE);
        Z << measurement.mag_x, measurement.mag_y, measurement.mag_z,
             measurement.gyro_x, measurement.gyro_y, measurement.gyro_z;
        ekf.Z = Z;
        
        // Run EKF step
        ekf.step();
        
        // Calculate errors
        float mag_error = sqrt(
            pow(measurement.mag_x - ekf.state(0), 2) +
            pow(measurement.mag_y - ekf.state(1), 2) +
            pow(measurement.mag_z - ekf.state(2), 2)
        );
        
        float gyro_error = sqrt(
            pow(measurement.gyro_x - ekf.state(3), 2) +
            pow(measurement.gyro_y - ekf.state(4), 2) +
            pow(measurement.gyro_z - ekf.state(5), 2)
        );
        
        total_mag_error += mag_error;
        total_gyro_error += gyro_error;
        measurement_count++;
        
        // Verify EKF is functioning correctly
        TEST_ASSERT_EQUAL(EKFError::OK, ekf.getLastError());
        
        // Print some debug information every 50 measurements
        if (measurement_count % 50 == 0) {
            Serial.print("Processing measurement ");
            Serial.print(measurement_count);
            Serial.print(", Current PWM: ");
            Serial.print(measurement.pwm);
            Serial.print(", Voltage: ");
            Serial.print(measurement.voltage);
            Serial.print(", Mag Error: ");
            Serial.print(mag_error);
            Serial.print(", Gyro Error: ");
            Serial.println(gyro_error);
        }
    }
    
    dataFile.close();
    
    // Calculate and print average errors
    if (measurement_count > 0) {
        float avg_mag_error = total_mag_error / measurement_count;
        float avg_gyro_error = total_gyro_error / measurement_count;
        
        Serial.println("\nTest Results:");
        Serial.print("Average Magnetic Field Error: ");
        Serial.println(avg_mag_error);
        Serial.print("Average Angular Velocity Error: ");
        Serial.println(avg_gyro_error);
        
        // Add some basic assertions for error bounds
        TEST_ASSERT_LESS_THAN(10.0f, avg_mag_error);  // Adjust threshold as needed
        TEST_ASSERT_LESS_THAN(1.0f, avg_gyro_error);  // Adjust threshold as needed
    }
}

void test_voltage_interpolation() {
    // Test interpolation at different voltage levels
    float test_voltages[] = {3.6654f, 3.7522f, 3.9876f, 4.0124f, 4.13445f};
    
    for (float voltage : test_voltages) {
        ekf.updateVoltage(voltage);
        
        Eigen::VectorXd Z = Eigen::VectorXd::Zero(STATE_SIZE);
        ekf.Z = Z;
        ekf.step();
        
        TEST_ASSERT_EQUAL(EKFError::OK, ekf.getLastError());
    }
}
#ifdef RUN_TESTS  // Only compile this if we're running tests
void setup() {
    delay(2000);  // Allow board to settle
    
    // Initialize serial for test output
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Serial.println("Starting CalibratedEKF Tests...");
    // Run all tests
    UNITY_BEGIN();
    RUN_TEST(test_voltage_interpolation);
    RUN_TEST(test_ekf_with_real_data);
    UNITY_END();
}

void loop() {
    // Empty loop as we only need to run tests once
    delay(1000);
}
#endif // RUN_TESTS

#endif // defined(ARDUINO) && defined(UNIT_TEST)