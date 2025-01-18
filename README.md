# ESPHome Kalman Filter Component

A Kalman filter component for ESPHome that helps smooth sensor readings.

## Overview

The Kalman Filter is a mathematical algorithm that provides an efficient computational (recursive) means to estimate the state of a process, in a way that minimizes the mean of the squared errors. This project provides a C++ implementation of the Kalman Filter, along with integration for ESPHome.

## Installation

Add this to your ESPHome configuration:

```yaml
external_components:
  - source: github://your-username/esphome-components
    components: [ kalman_filter ]
```

## Usage

```yaml
sensor:
  - platform: kalman_filter
    name: "Filtered Sensor"
    source_id: raw_sensor_id
    process_noise: 0.01      # Optional, default: 0.01
    measurement_noise: 0.1   # Optional, default: 0.1
    estimated_error: 1.0     # Optional, default: 1.0
```

### Configuration variables:

- **source_id** (*Required*): The ID of the source sensor to filter
- **process_noise** (*Optional*): Process noise parameter (Q). Default: 0.01
- **measurement_noise** (*Optional*): Measurement noise parameter (R). Default: 0.1
- **estimated_error** (*Optional*): Initial estimate error (P). Default: 1.0

Plus all other options from the [Sensor Component](https://esphome.io/components/sensor/index.html#config-sensor).

## Components

- **Kalman Filter**: A class that implements the Kalman Filter algorithm.
  - `kalman_filter.h`: Header file declaring the KalmanFilter class.
  - `kalman_filter.cpp`: Source file implementing the KalmanFilter methods.
  - `sensor.py`: Python interface for integrating the Kalman Filter with ESPHome.

## Examples

An example configuration for using the Kalman Filter component in an ESPHome project can be found in `examples/kalman_filter.yaml`.

## Testing

Unit tests for the Kalman Filter class are provided in `tests/test_kalman_filter.cpp`.

## Building

To build the project, use CMake. The configuration file is located at `CMakeLists.txt`.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.