#ifndef EKF_H
#define EKF_H

#include <ArduinoEigen.h>
#include <vector>
#include <deque>
#include <array>
#include <numeric>

class EKF {
public:
    Eigen::VectorXd state;
    Eigen::VectorXd Z;
    Eigen::MatrixXd covariance;

    EKF() {
        initialize_dynamic_tracking();
    }

protected:
    double dt;
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R_d; // Measurement noise covariance
    Eigen::MatrixXd H_d;

    // Dynamic response parameters
    static constexpr size_t RESPONSE_WINDOW = 5;
    static constexpr double BASE_DAMPING = 0.05;  // Reduced from 0.1
    static constexpr double MIN_INNOVATION_GAIN = 0.5;
    static constexpr double MAX_INNOVATION_GAIN = 2.0;
    static constexpr double VELOCITY_THRESHOLD = 0.5;

    // Circular buffers for tracking dynamics
    std::array<std::deque<double>, 6> value_history;
    std::array<std::deque<double>, 6> velocity_history;
    std::array<double, 6> current_gains;

    void initialize_dynamic_tracking() {
        // Initialize histories for each state component
        for (int i = 0; i < 6; i++) {
            value_history[i] = std::deque<double>(RESPONSE_WINDOW);
            velocity_history[i] = std::deque<double>(RESPONSE_WINDOW);
            current_gains[i] = 1.0;
        }
    }

    struct DynamicsInfo {
        double velocity;
        double acceleration;
    };

    DynamicsInfo calculate_dynamics(const std::deque<double>& values) {
        DynamicsInfo result = {0.0, 0.0};
        
        if (values.size() < 3) return result;

        // Calculate velocity
        std::vector<double> velocities;
        for (size_t i = 1; i < values.size(); ++i) {
            velocities.push_back(values[i] - values[i-1]);
        }
        
        // Calculate acceleration
        std::vector<double> accelerations;
        for (size_t i = 1; i < velocities.size(); ++i) {
            accelerations.push_back(velocities[i] - velocities[i-1]);
        }

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

        dxdt << -x(2) * x(4) + x(1) * x(5) - damping[0] * x(0),
                x(2) * x(3) - x(0) * x(5) - damping[1] * x(1),
                -x(1) * x(3) + x(0) * x(4) - damping[2] * x(2),
                -x(4) * x(5) - damping[3] * x(3),
                x(3) * x(5) - damping[4] * x(4),
                -x(3) * x(4) / 3 - damping[5] * x(5);
        return dxdt;
    }
    
    Eigen::Matrix<double, 6, 1> rk4_step(const Eigen::Matrix<double, 6, 1> &x, double step_size) {
        Eigen::Matrix<double, 6, 1> k1 = f(x);
        k1 *= step_size;
    
        Eigen::Matrix<double, 6, 1> half_k1 = 0.5 * k1;
    
        Eigen::Matrix<double, 6, 1> k2 = f(x + half_k1);
        k2 *= step_size;
    
        Eigen::Matrix<double, 6, 1> half_k2 = 0.5 * k2;
    
        Eigen::Matrix<double, 6, 1> k3 = f(x + half_k2);
        k3 *= step_size;
    
        Eigen::Matrix<double, 6, 1> k4 = f(x + k3);
        k4 *= step_size;
    
        Eigen::Matrix<double, 6, 1> result = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        
        // Check for numerical stability
        if (result.array().isNaN().any()) {
            return x;  // Return original state if unstable
        }
        
        return result;
    }

    Eigen::VectorXd rk4(const Eigen::VectorXd &x_initial, double f_step_size, double t_start, double t_end) {
        Eigen::VectorXd x = x_initial;
        int num_steps = static_cast<int>((t_end - t_start) / f_step_size);
    
        for (int i = 0; i < num_steps; ++i) {
            x = rk4_step(x, f_step_size);
        }
    
        return x;
    }

    void predict(const Eigen::MatrixXd &J_k_k) {
        state = rk4(state, dt, 0.0, dt);
        
        // Adjust process noise based on gains
        Eigen::MatrixXd Q_adjusted = Q;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                Q_adjusted(i,j) *= (current_gains[i] * current_gains[j]);
            }
        }
        
        covariance = J_k_k * covariance * J_k_k.transpose() + Q_adjusted;
        
        // Ensure symmetry
        covariance = (covariance + covariance.transpose()) * 0.5;
    }
    
    void correct() {
        // Adjust measurement noise based on gains
        Eigen::MatrixXd R_adjusted = R_d;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                R_adjusted(i,j) /= (current_gains[i] * current_gains[j]);
            }
        }
        
        Eigen::MatrixXd S = H_d * covariance * H_d.transpose() + R_adjusted;
        Eigen::MatrixXd K_k1 = covariance * H_d.transpose() * S.inverse();
    
        // Update state estimate
        state = state + K_k1 * (Z - H_d * state);
    
        // Update covariance matrix
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
        covariance = (I - K_k1 * H_d) * covariance;
        
        // Ensure symmetry
        covariance = (covariance + covariance.transpose()) * 0.5;
    }

    Eigen::MatrixXd CalculateJacobian() {
        // Calculate adaptive damping terms
        std::array<double, 6> damping;
        for (int i = 0; i < 6; ++i) {
            damping[i] = BASE_DAMPING / current_gains[i];
        }

        double Bx = Z(0), By = Z(1), Bz = Z(2), wx = Z(3), wy = Z(4), wz = Z(5);
        Eigen::MatrixXd J(6, 6);
    
        J << -damping[0], wz, -wy, 0, -Bz, By,
             -wz, -damping[1], wx, Bz, 0, -Bx,
             wy, -wx, -damping[2], -By, Bx, 0,
             0, 0, 0, -damping[3], -wz, -wy,
             0, 0, 0, wz, -damping[4], wx,
             0, 0, 0, -wx/3, -wy/3, -damping[5];
    
        return J;
    }

public:
    void initialize(double delta_t, 
            const Eigen::VectorXd &initial_state, 
            const Eigen::MatrixXd &initial_covariance, 
            const Eigen::MatrixXd &process_noise_covariance, 
            const Eigen::MatrixXd &Rd, 
            const Eigen::MatrixXd &Hd) {
        state = initial_state;
        Z = initial_state;
        covariance = initial_covariance;
        Q = process_noise_covariance;
        R_d = Rd;
        H_d = Hd;
        dt = delta_t;
    }

    void step() {
        update_gains(Z);
        Eigen::MatrixXd J = CalculateJacobian();
        predict(J);
        correct();
    }
};

#endif // EKF_H