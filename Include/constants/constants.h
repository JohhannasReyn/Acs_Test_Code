#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include <stddef.h>

namespace constants {
    namespace imu {
        static constexpr size_t VOLTAGE_LEVELS = 7;
        static constexpr float MIN_VOLTAGE = 3.6f;
        static constexpr float MAX_VOLTAGE = 4.2f;
        static constexpr float VOLTAGE_STEP = 0.1f;

		extern const float mag_hardiron_x;
        extern const float mag_hardiron_y;
        extern const float mag_hardiron_z;
        extern const float gyro_hardiron_x;
        extern const float gyro_hardiron_y;
        extern const float gyro_hardiron_z;

        // New coefficient structures to match the polynomial calibration
        struct PWMCoefficients {
          float coeff_1;
          float coeff_2;
          float coeff_3;
        };

        struct PWMDirectionCoefficients {
          PWMCoefficients ox;
          PWMCoefficients oy;
          PWMCoefficients oz;
        };

        struct VoltageCoefficients {
          PWMDirectionCoefficients pwmX;
          PWMDirectionCoefficients pwmY;
          PWMDirectionCoefficients pwmZ;
        };

        extern std::map<float, VoltageCoefficients> voltage_coefficients;
    }
}

#endif // CONSTANTS_H