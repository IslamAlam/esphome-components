#include "kalman_filter.h"
#include <cassert>
#include <cmath>
#include <iostream>

// Helper function to check if two floats are approximately equal
bool approx_equal(float a, float b, float epsilon = 0.0001f) {
    return std::fabs(a - b) < epsilon;
}

void test_construction() {
    // Test valid construction
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 0.0f);
    assert(kf.getProcessNoise() == 0.1f);
    assert(kf.getMeasurementNoise() == 0.1f);
    assert(kf.getEstimate() == 0.0f);

    // Test invalid construction
    bool exception_caught = false;
    try {
        KalmanFilter invalid(-0.1f, 0.1f, 1.0f, 0.0f);
    } catch (const std::invalid_argument&) {
        exception_caught = true;
    }
    assert(exception_caught);
}

void test_update() {
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 0.0f);
    
    // Test single update
    float est1 = kf.update(1.0f);
    assert(est1 > 0.0f && est1 < 1.0f);
    
    // Test convergence
    for (int i = 0; i < 100; i++) {
        kf.update(10.0f);
    }
    assert(approx_equal(kf.getEstimate(), 10.0f, 0.1f));
}

void test_bounded_update() {
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 0.0f);
    
    // Test bounds
    float est = kf.updateBounded(15.0f, 0.0f, 10.0f);
    assert(est <= 10.0f);
    
    est = kf.updateBounded(-5.0f, 0.0f, 10.0f);
    assert(est >= 0.0f);
}

void test_reset() {
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 5.0f);
    kf.update(10.0f);
    kf.reset(0.0f);
    
    assert(kf.getEstimate() == 0.0f);
    assert(kf.getKalmanGain() == 0.0f);
}

void test_noise_setters() {
    KalmanFilter kf(0.1f, 0.1f, 1.0f, 0.0f);
    
    // Test valid updates
    kf.setProcessNoise(0.2f);
    assert(kf.getProcessNoise() == 0.2f);
    
    kf.setMeasurementNoise(0.3f);
    assert(kf.getMeasurementNoise() == 0.3f);
    
    // Test invalid updates
    bool exception_caught = false;
    try {
        kf.setProcessNoise(-0.1f);
    } catch (const std::invalid_argument&) {
        exception_caught = true;
    }
    assert(exception_caught);
}

int main() {
    try {
        std::cout << "Running Kalman Filter tests..." << std::endl;
        
        test_construction();
        test_update();
        test_bounded_update();
        test_reset();
        test_noise_setters();
        
        std::cout << "All tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
}