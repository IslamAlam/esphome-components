#include "kalman_filter.h"
#include <cassert>

void test_kalman_filter() {
    // Create a KalmanFilter instance with initial parameters
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 0.0f);

    // Test the update method with a measurement
    float measurement = 1.0f;
    float estimated_value = kf.update(measurement);
    
    // Check if the estimated value is within an acceptable range
    assert(estimated_value >= 0.0f && estimated_value <= 1.0f);

    // Test with another measurement
    measurement = 2.0f;
    estimated_value = kf.update(measurement);
    
    // Check if the estimated value is updated correctly
    assert(estimated_value >= 0.0f && estimated_value <= 2.0f);
}

int main() {
    test_kalman_filter();
    return 0;
}