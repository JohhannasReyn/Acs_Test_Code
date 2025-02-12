#ifndef RESPONSIVE_EKF_H
#define RESPONSIVE_EKF_H

#pragma once

#include "../constants/constants.h"
#include "../ekf/ekf.hpp"
#include <ArduinoEigen.h>
#include <deque>
#include <array>
#include <cmath>

class ResponsiveEKF : public EKF {
private:
    // Dynamic response parameters
    static constexpr size_t RESPONSE_WINDOW = 5;
    static constexpr double MIN_INNOVATION_GAIN = 0.5;
    static constexpr double MAX_INNOVATION_GAIN = 2.0;
    static constexpr double VELOCITY_THRESHOLD = 0.5;
    static constexpr double BASE_DAMPING = 0.05;

    // History tracking for dynamics
    std::array<std::deque<double>, 6> value_history;
    std::array<std::deque<double>, 6> velocity_history;
    std::array<double, 6> current_gains;
    
    // Base noise matrices
    Eigen::MatrixXd Q_base;
    Eigen::MatrixXd R_base;

    struct DynamicsInfo {
        double velocity;
        double acceleration;
    };

    DynamicsInfo calculate_dynamics(const std::deque<double>& values) {
        DynamicsInfo result = {0.0, 0.0};
        
        if (values.size() < 3) {
            return result;
        }

        // Calculate velocities
        std::vector<double> velocities;
        for (size_t i = 1; i < values.size(); ++i) {
            velocities.push_back(values[i] - values[i-1]);
        }
        
        // Calculate accelerations
        std::vector<double> accelerations;
        for (size_t i = 1; i < velocities.size(); ++i) {
            accelerations.push_back(velocities[i] - velocities[i-1]);
        }

        // Calculate means
        result.velocity = std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();
        result.acceleration = std::accumulate(accelerations.begin(), accelerations.end(), 0.0) / accelerations.size();

        return result;
    }

    void update_gains(const Eigen::VectorXd& measurement) {
        for (int i = 0; i < 6; ++i) {
            // Update value history
            if (value_history[i].size() >= RESPONSE_WINDOW) {
                value_history[i].pop_front();
            }
            value_history[i].push_back(measurement(i));

            // Calculate dynamics
            auto dynamics = calculate_dynamics(value_history[i]);

            // Update velocity history
            if (velocity_history[i].size() >= RESPONSE_WINDOW) {
                velocity_history[i].pop_front();
            }
            velocity_history[i].push_back(dynamics.velocity);

            // Calculate velocity trend
            if (velocity_history[i].size() >= 2) {
                double velocity_trend = 0.0;
                for (size_t j = 1; j < velocity_history[i].size(); ++j) {
                    velocity_trend += velocity_history[i][j] - velocity_history[i][j-1];
                }
                velocity_trend /= (velocity_history[i].size() - 1);

                // Update gain based on dynamics
                if (std::abs(velocity_trend) > VELOCITY_THRESHOLD) {
                    current_gains[i] = MAX_INNOVATION_GAIN;
                } else {
                    current_gains[i] = MIN_INNOVATION_GAIN;
                }

                // Additional boost for acceleration
                if (std::abs(dynamics.acceleration) > VELOCITY_THRESHOLD) {
                    current_gains[i] *= 1.5;
                }
            }
        }
    }

    Eigen::Matrix<double, 6, 1> f(const Eigen::Matrix<double, 6, 1> &x) {
        Eigen::Matrix<double, 6, 1> dxdt;
        
        // Calculate adaptive damping
        std::array<double, 6> damping;
        for (int i = 0; i < 6; ++i) {
            damping[i] = BASE_DAMPING / current_gains[i];
        }

        dxdt(0) = -x(2) * x(4) + x(1) * x(5) - damping[0] * x(0);
        dxdt(1) = x(2) * x(3) - x(0) * x(5) - damping[1] * x(1);
        dxdt(2) = -x(1) * x(3) + x(0) * x(4) - damping[2] * x(2);
        dxdt(3) = -x(4) * x(5) - damping[3] * x(3);
        dxdt(4) = x(3) * x(5) - damping[4] * x(4);
        dxdt(5) = -x(3) * x(4) / 3 - damping[5] * x(5);
        
        return dxdt;
    }

    void predict(const Eigen::MatrixXd &J) {
        // Predict state using RK4
        state = rk4(state, dt, 0.0, dt);
        
        // Adjust process noise based on current gains
        Eigen::MatrixXd Q_adjusted = Q_base;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                Q_adjusted(i,j) *= (current_gains[i] * current_gains[j]);
            }
        }
        
        // Update covariance
        covariance = J * covariance * J.transpose() + Q_adjusted;
        
        // Ensure symmetry
        covariance = (covariance + covariance.transpose()) * 0.5;
    }

    void correct() {
        // Adjust measurement noise based on gains
        Eigen::MatrixXd R_adjusted = R_base;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                R_adjusted(i,j) /= (current_gains[i] * current_gains[j]);
            }
        }
        
        // Calculate Kalman gain
        Eigen::MatrixXd S = H_d * covariance * H_d.transpose() + R_adjusted;
        Eigen::MatrixXd K = covariance * H_d.transpose() * S.inverse();
        
        // Update state
        state = state + K * (Z - H_d * state);
        
        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
        covariance = (I - K * H_d) * covariance;
        
        // Ensure symmetry
        covariance = (covariance + covariance.transpose()) * 0.5;
    }

    Eigen::MatrixXd CalculateJacobian() {
        // Calculate adaptive damping
        std::array<double, 6> damping;
        for (int i = 0; i < 6; ++i) {
            damping[i] = BASE_DAMPING / current_gains[i];
        }

        Eigen::MatrixXd J(6, 6);
        
        const double Bx = state(0), By = state(1), Bz = state(2);
        const double wx = state(3), wy = state(4), wz = state(5);
        
        J << -damping[0], wz, -wy, 0, -Bz, By,
             -wz, -damping[1], wx, Bz, 0, -Bx,
             wy, -wx, -damping[2], -By, Bx, 0,
             0, 0, 0, -damping[3], -wz, -wy,
             0, 0, 0, wz, -damping[4], wx,
             0, 0, 0, -wx/3, -wy/3, -damping[5];
        
        return J;
    }

public:
    ResponsiveEKF() : EKF() {
        // Initialize histories
        for (int i = 0; i < 6; ++i) {
            value_history[i] = std::deque<double>();
            velocity_history[i] = std::deque<double>();
            current_gains[i] = 1.0;
        }
        
        // Initialize base noise matrices
        Q_base = Eigen::MatrixXd::Zero(6, 6);
        Q_base.diagonal() << 0.05, 0.05, 0.05, 0.01, 0.01, 0.01;
        
        R_base = Eigen::MatrixXd::Zero(6, 6);
        R_base.diagonal() << 0.8, 0.8, 0.8, 0.1, 0.1, 0.1;
    }

    void initialize(double delta_t,
                   const Eigen::VectorXd &initial_state,
                   const Eigen::MatrixXd &initial_covariance,
                   const Eigen::MatrixXd &process_noise_covariance,
                   const Eigen::MatrixXd &Rd,
                   const Eigen::MatrixXd &Hd) {
        EKF::initialize(delta_t, initial_state, initial_covariance,
                       process_noise_covariance, Rd, Hd);
        
        // Store base noise matrices
        Q_base = process_noise_covariance;
        R_base = Rd;
    }

    void step() {
        update_gains(Z);
        Eigen::MatrixXd J = CalculateJacobian();
        predict(J);
        correct();
    }
};

#endif // RESPONSIVE_EKF_H