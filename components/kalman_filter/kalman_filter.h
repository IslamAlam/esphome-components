#include <stdexcept>
#include <algorithm> // for std::clamp
#include <cmath> // for std::isfinite

class KalmanFilter {
public:
    /**
     * @brief Constructs a 1D Kalman filter
     * @param process_noise Process noise (Q) - models unpredictable changes
     * @param measurement_noise Measurement noise (R) - models sensor noise
     * @param estimated_error Initial estimate error covariance (P)
     * @param initial_value Initial state value (X)
     * @throws std::invalid_argument if noise parameters are negative
     */
    explicit KalmanFilter(float process_noise, float measurement_noise, 
                         float estimated_error, float initial_value)
        : process_noise_(process_noise), measurement_noise_(measurement_noise), 
          error_covariance_(estimated_error), state_estimate_(initial_value), kalman_gain_(0.0f) {
        if (process_noise < 0 || measurement_noise < 0 || estimated_error < 0) {
            throw std::invalid_argument("Noise parameters must be non-negative");
        }
    }

    /**
     * @brief Updates the filter with a new measurement
     * @param measurement The measured value
     * @return The filtered estimate
     */
    float update(float measurement) {
        // Prediction Update
        error_covariance_ += process_noise_;

        // Measurement Update
        kalman_gain_ = error_covariance_ / (error_covariance_ + measurement_noise_);
        state_estimate_ += kalman_gain_ * (measurement - state_estimate_);
        error_covariance_ *= (1.0f - kalman_gain_);

        return state_estimate_;
    }

    /**
     * @brief Updates the filter with bounds checking
     * @param measurement The measured value
     * @param min_bound Minimum allowed value
     * @param max_bound Maximum allowed value
     * @return The filtered estimate
     */
    float updateBounded(float measurement, float min_bound, float max_bound) {
        float estimate = update(measurement);
        return std::clamp(estimate, min_bound, max_bound);
    }

    /**
     * @brief Resets the filter to initial conditions
     * @param initial_value New initial state value
     */
    void reset(float initial_value = 0.0f) {
        state_estimate_ = initial_value;
        error_covariance_ = measurement_noise_;
        kalman_gain_ = 0.0f;
    }

    /**
     * @brief Updates the process noise
     * @param process_noise New process noise value
     * @throws std::invalid_argument if noise parameter is negative
     */
    void setProcessNoise(float process_noise) {
        if (process_noise < 0) {
            throw std::invalid_argument("Process noise must be non-negative");
        }
        process_noise_ = process_noise;
    }

    /**
     * @brief Updates the measurement noise
     * @param measurement_noise New measurement noise value
     * @throws std::invalid_argument if noise parameter is negative
     */
    void setMeasurementNoise(float measurement_noise) {
        if (measurement_noise < 0) {
            throw std::invalid_argument("Measurement noise must be non-negative");
        }
        measurement_noise_ = measurement_noise;
    }

    // Getters
    float getEstimate() const { return state_estimate_; }
    float getErrorCovariance() const { return error_covariance_; }
    float getKalmanGain() const { return kalman_gain_; }
    float getProcessNoise() const { return process_noise_; }
    float getMeasurementNoise() const { return measurement_noise_; }

    /**
     * @brief Checks if the current state estimate is valid
     * @return true if the state is valid (not NaN or Inf)
     */
    bool isStateValid() const {
        return std::isfinite(state_estimate_);
    }

private:
    float process_noise_;      // Changed from const to allow adjustment
    float measurement_noise_;  // Changed from const to allow adjustment
    float error_covariance_;        // Estimate error covariance
    float kalman_gain_;             // Kalman gain
    float state_estimate_;          // Current state estimate
};