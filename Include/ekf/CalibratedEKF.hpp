#ifndef CALIBRATED_EKF_H
#define CALIBRATED_EKF_H

#pragma once

#include "../constants/constants.h"
#include "../ekf/ekf.hpp"
#include <ArduinoEigen.h>
#include <SD.h>
#include <map>
#include <math.h>
#include <deque>

typedef std::array<float, 3> CoeffArray;

using namespace constants::imu;

enum class EKFError {
    OK = 0,
    VOLTAGE_OUT_OF_BOUNDS = 1,
    INSUFFICIENT_CALIBRATION_DATA = 2,
    INTERPOLATION_ERROR = 3,
    OFFSET_CALCULATION_ERROR = 4,
    STATE_BOUNDS_EXCEEDED = 5
};

class CalibratedEKF : public EKF {
private:
    float last_innovation_magnitude;
    float last_prediction_error;
    float current_voltage;
    EKFError last_error;
    CoeffArray x_offsets;
    CoeffArray y_offsets;
    CoeffArray z_offsets;

    // Innovation tracking
    static constexpr size_t INNOVATION_HISTORY_SIZE = 50;
    std::array<std::deque<double>, 6> innovation_history;
    std::array<std::deque<double>, 6> offset_history;
    
    // Learning parameters
    static constexpr double LEARNING_RATE = 0.1;
    static constexpr double ERROR_DECAY = 0.95;
    std::array<double, 6> cumulative_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    static constexpr size_t getVoltageIndex(float voltage) {
        if (voltage <= MIN_VOLTAGE) return 0;
        if (voltage >= MIN_VOLTAGE + (VOLTAGE_LEVELS - 1) * VOLTAGE_STEP) {
            return VOLTAGE_LEVELS - 1;
        }
        return static_cast<size_t>((voltage - MIN_VOLTAGE) / VOLTAGE_STEP);
    }

    bool getInterpolatedOffsets(float voltage, PWMCoefficients &coeffs,
        const std::function<const PWMCoefficients &(const VoltageCoefficients &)> &coeffSelector) {
        last_error = EKFError::OK;
        
        if (voltage < 3.6 || voltage > 4.2) {
            voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
            last_error = EKFError::VOLTAGE_OUT_OF_BOUNDS;
        }
        
        auto upper_it = voltage_coefficients.upper_bound(voltage);
        auto lower_it = std::prev(upper_it);
        
        float lower_voltage = lower_it->first;
        float upper_voltage = upper_it->first;
        float factor = (voltage - lower_voltage) / (upper_voltage - lower_voltage);

        const PWMCoefficients &lower_coeffs = coeffSelector(lower_it->second);
        const PWMCoefficients &upper_coeffs = coeffSelector(upper_it->second);

        coeffs.coeff_1 = lower_coeffs.coeff_1 + factor * (upper_coeffs.coeff_1 - lower_coeffs.coeff_1);
        coeffs.coeff_2 = lower_coeffs.coeff_2 + factor * (upper_coeffs.coeff_2 - lower_coeffs.coeff_2);
        coeffs.coeff_3 = lower_coeffs.coeff_3 + factor * (upper_coeffs.coeff_3 - lower_coeffs.coeff_3);

        return last_error == EKFError::OK;
    }

    void update_error_learning(const Eigen::VectorXd& error) {
        static const double weight_sum = std::accumulate(
            std::begin(std::array<double, INNOVATION_HISTORY_SIZE>({ERROR_DECAY})), 
            std::end(std::array<double, INNOVATION_HISTORY_SIZE>({ERROR_DECAY})), 
            0.0, 
            [](double sum, double decay) { return sum + std::pow(decay, sum); }
        );

        for (int i = 0; i < 6; ++i) {
            // Update innovation history
            if (innovation_history[i].size() >= INNOVATION_HISTORY_SIZE) {
                innovation_history[i].pop_front();
            }
            innovation_history[i].push_back(error(i));

            // Calculate weighted average of recent errors
            double weighted_error = 0.0;
            size_t idx = 0;
            for (const auto& hist_error : innovation_history[i]) {
                weighted_error += hist_error * std::pow(ERROR_DECAY, idx++) / weight_sum;
            }

            // Update offset history
            if (offset_history[i].size() >= INNOVATION_HISTORY_SIZE) {
                offset_history[i].pop_front();
            }
            offset_history[i].push_back(weighted_error);

            // Update cumulative offset with learning rate
            cumulative_offset[i] += LEARNING_RATE * weighted_error;
        }
    }

    bool calculatePWMValues(float voltage, float pwm_x, float pwm_y, float pwm_z,
                          float &pwmX_ox, float &pwmX_oy, float &pwmX_oz,
                          float &pwmY_ox, float &pwmY_oy, float &pwmY_oz,
                          float &pwmZ_ox, float &pwmZ_oy, float &pwmZ_oz) {
        PWMCoefficients x_ox_coeffs, x_oy_coeffs, x_oz_coeffs;
        PWMCoefficients y_ox_coeffs, y_oy_coeffs, y_oz_coeffs;
        PWMCoefficients z_ox_coeffs, z_oy_coeffs, z_oz_coeffs;

        bool success = true;

        success &= getInterpolatedOffsets(voltage, x_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmX.ox; });
        success &= getInterpolatedOffsets(voltage, x_oy_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmX.oy; });
        success &= getInterpolatedOffsets(voltage, x_oz_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmX.oz; });

        success &= getInterpolatedOffsets(voltage, y_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmY.ox; });
        success &= getInterpolatedOffsets(voltage, y_oy_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmY.oy; });
        success &= getInterpolatedOffsets(voltage, y_oz_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmY.oz; });

        success &= getInterpolatedOffsets(voltage, z_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmZ.ox; });
        success &= getInterpolatedOffsets(voltage, z_oy_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmZ.oy; });
        success &= getInterpolatedOffsets(voltage, z_oz_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmZ.oz; });

        if (!success) {
            return false;
        }

        // Calculate PWM values with polynomial coefficients
        pwmX_ox = x_ox_coeffs.coeff_1 * pwm_x + x_ox_coeffs.coeff_2 * pow(pwm_x, 2) + x_ox_coeffs.coeff_3 * pow(pwm_x, 3);
        pwmX_oy = x_oy_coeffs.coeff_1 * pwm_x + x_oy_coeffs.coeff_2 * pow(pwm_x, 2) + x_oy_coeffs.coeff_3 * pow(pwm_x, 3);
        pwmX_oz = x_oz_coeffs.coeff_1 * pwm_x + x_oz_coeffs.coeff_2 * pow(pwm_x, 2) + x_oz_coeffs.coeff_3 * pow(pwm_x, 3);

        pwmY_ox = y_ox_coeffs.coeff_1 * pwm_y + y_ox_coeffs.coeff_2 * pow(pwm_y, 2) + y_ox_coeffs.coeff_3 * pow(pwm_y, 3);
        pwmY_oy = y_oy_coeffs.coeff_1 * pwm_y + y_oy_coeffs.coeff_2 * pow(pwm_y, 2) + y_oy_coeffs.coeff_3 * pow(pwm_y, 3);
        pwmY_oz = y_oz_coeffs.coeff_1 * pwm_y + y_oz_coeffs.coeff_2 * pow(pwm_y, 2) + y_oz_coeffs.coeff_3 * pow(pwm_y, 3);

        pwmZ_ox = z_ox_coeffs.coeff_1 * pwm_z + z_ox_coeffs.coeff_2 * pow(pwm_z, 2) + z_ox_coeffs.coeff_3 * pow(pwm_z, 3);
        pwmZ_oy = z_oy_coeffs.coeff_1 * pwm_z + z_oy_coeffs.coeff_2 * pow(pwm_z, 2) + z_oy_coeffs.coeff_3 * pow(pwm_z, 3);
        pwmZ_oz = z_oz_coeffs.coeff_1 * pwm_z + z_oz_coeffs.coeff_2 * pow(pwm_z, 2) + z_oz_coeffs.coeff_3 * pow(pwm_z, 3);

        return true;
    }

public:
    CalibratedEKF() : current_voltage(MIN_VOLTAGE),
                      last_error(EKFError::OK),
                      x_offsets{0.0f, 0.0f, 0.0f},
                      y_offsets{0.0f, 0.0f, 0.0f},
                      z_offsets{0.0f, 0.0f, 0.0f} {
        for (int i = 0; i < 6; ++i) {
            innovation_history[i] = std::deque<double>();
            offset_history[i] = std::deque<double>();
        }
    }

    EKFError getLastError() const { return last_error; }

    double getLastInnovationMagnitude() const {
        return last_innovation_magnitude;
    }

    double getLastPredictionError() const { return last_prediction_error; }

    void initialize(double delta_t, const Eigen::VectorXd &initial_state,
                   const Eigen::MatrixXd &initial_covariance,
                   const Eigen::MatrixXd &process_noise_covariance,
                   const Eigen::MatrixXd &noise_covariance,
                   const Eigen::MatrixXd &Hd, float initial_voltage) {
        EKF::initialize(delta_t, initial_state, initial_covariance,
                       process_noise_covariance, noise_covariance, Hd);
        this->updateVoltage(initial_voltage);
    }

    bool updateVoltage(float voltage) {
        if (voltage < MIN_VOLTAGE || voltage > MAX_VOLTAGE) {
            last_error = EKFError::VOLTAGE_OUT_OF_BOUNDS;
            current_voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
        } else {
            current_voltage = voltage;
            last_error = EKFError::OK;
        }
        return last_error == EKFError::OK;
    }

    void step() {
        // Update axis offsets
        PWMCoefficients x_ox_coeffs, y_ox_coeffs, z_ox_coeffs;
    
        bool success = true;
        success &= getInterpolatedOffsets(current_voltage, x_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmX.ox; });
        success &= getInterpolatedOffsets(current_voltage, y_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmY.ox; });
        success &= getInterpolatedOffsets(current_voltage, z_ox_coeffs,
            [](const VoltageCoefficients &vc) -> const PWMCoefficients & { return vc.pwmZ.ox; });

        if (!success) {
            last_error = EKFError::OFFSET_CALCULATION_ERROR;
        }

        // Store original measurement
        Eigen::VectorXd original_Z = Z;

        // Apply cumulative offsets and calibration
        for (int i = 0; i < 6; ++i) {
            Z(i) += cumulative_offset[i];
        }

        // Standard EKF step with improved dynamics
        Eigen::VectorXd predicted_state = rk4(state, dt, 0.0, dt);
        Eigen::VectorXd innovation = Z - H_d * predicted_state;
        last_innovation_magnitude = innovation.norm();

        // Update gains based on current dynamics
        update_gains(Z);

        // Perform EKF update
        Eigen::MatrixXd J = CalculateJacobian();
        predict(J);
        correct();

        // Learn from prediction error
        Eigen::VectorXd prediction_error = Z - H_d * state;
        update_error_learning(prediction_error);
        last_prediction_error = prediction_error.norm();

        // Remove offsets from state
        for (int i = 0; i < 6; ++i) {
            state(i) -= cumulative_offset[i];
        }

        // Physical bounds checking
        static const float MAX_MAGNETIC_FIELD = 100.0;
        static const float MAX_ANGULAR_VELOCITY = 10.0;

        bool state_valid = true;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(state(i)) > MAX_MAGNETIC_FIELD || 
                std::abs(state(i + 3)) > MAX_ANGULAR_VELOCITY) {
                state_valid = false;
                break;
            }
        }

        last_error = state_valid ? EKFError::OK : EKFError::STATE_BOUNDS_EXCEEDED;

        // Restore original measurement
        Z = original_Z;
    }
};

#endif // CALIBRATED_EKF_H