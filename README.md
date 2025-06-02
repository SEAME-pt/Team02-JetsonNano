# Vehicle Control System for Jetson Nano Platform

This repository contains the complete control system for an autonomous vehicle, designed to run on the NVIDIA Jetson Nano platform. The system leverages computer vision, sensor fusion, and advanced control algorithms to enable autonomous driving capabilities.

## System Overview

The Vehicle Control System integrates multiple components that work together to provide autonomous driving functionality:

- **Vehicle System**: Core vehicle management and control
- **Object Detection**: Real-time object detection using TensorRT
- **Lane Detection**: Lane marking detection and tracking
- **Middleware**: Sensor data processing and signal conversion
- **Combined Controller**: Intelligent control switching between manual and autonomous modes

### Key Components:

1. **Vehicle System**
   - Manages overall vehicle state and behavior
   - Implements VSS (Vehicle Signal Specification) data model
   - Coordinates subsystems (Powertrain, Chassis, Body, ...)

2. **ADAS System**
   - Obstacle detection and avoidance
   - Trajectory planning
   - Safety monitoring

3. **Middleware**
   - Converts between communication protocols
   - Handles sensor data acquisition and processing
   - Implements signal routing between components

4. **Controller Systems**
   - PID controller for autonomous driving
   - Xbox controller interface for manual override
   - Controller switching logic

5. **Perception Systems**
   - Object Detection using TensorRT-accelerated CNN
   - Lane Detection with optimized image processing

## Communication

The system uses a sophisticated communication architecture:

- **Zenoh**: High-performance pub/sub middleware for IPC communication with shared-memory
- **Zenoh Router**: Connects vehicle systems to cloud services
- **CAN Bus**: Communication with vehicle hardware components (Micro Controller and Raspberry Pi)
- **VSS**: Standardized vehicle signal specification for data exchange

## Tools
   In this directory you can find:
   - VSS files.
   - Camera calibration tools.
   - The System monitor.
   - Zenoh Router.
   - InfluxDB and Graphana (Database and visualization).

---

## Building and Running

### Prerequisites

- NVIDIA Jetson Nano with JetPack 4.6 or later
- CUDA Toolkit 10.2 or later
- OpenCV 4.1.1 or later
- Zenoh C/C++ libraries
- CMake 3.16 or later

### Build Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/SEAME-pt/Team02-JetsonNano.git
   cd Team02-JetsonNano
   ```

2. Create a build directory:
   ```bash
   mkdir build && cd build
   ```

3. Configure and build:
   ```bash
   cmake ..
   make -j4
   ```

### Running the System

#### Vehicle System
```bash
./VehicleSystem
```

#### Object Detector
```bash
./ObjectDetector
```

#### Middleware
```bash
./MiddleWare
```

#### Combined Controller
```bash
./CombinedController
```

## Components in Detail

### Vehicle System

The core vehicle management system implementing a complete vehicle model following automotive standards:

- **Powertrain**: Electric motor, transmission, and battery management
- **Chassis**: Accelerator, brake, steering, and axle control
- **Body**: Exterior components, lights, and accessories
- **Vehicle**: Main vehicle state, connectivity, and motion management

### Object Detector

Computer vision system for detecting and classifying road objects:

- Uses TensorRT-optimized neural networks for efficient inference
- Processes camera input in real-time
- Identifies vehicles, pedestrians, traffic signs, etc.
- Publishes detection results via Zenoh

### Lane Detector

Vision system for lane detection and tracking:

- Processes camera frames to identify lane markings
- Calculates road geometry and vehicle position
- Provides lane keeping assistance data
- Optimized for Jetson Nano using CUDA acceleration

### Middleware

Communication and signal processing system:

- Interfaces with physical sensors (battery, etc.)
- Converts between communication protocols
- Implements signal routing and filtering
- Provides hardware abstraction layer

### Combined Controller

Intelligent control system with manual and autonomous capabilities:

- Xbox controller interface for manual control
- PID controller implementation for autonomous driving
- Trajectory planning and following
- Transition between control modes

## Testing

The system includes comprehensive unit tests using Catch2:

```bash
cd build
cmake -DENABLE_TESTING=ON ..
make
ctest
```

For generating code coverage:
```bash
cmake -DENABLE_TESTING=ON -DENABLE_COVERAGE=ON ..
make coverage
```

## Dependencies

The system depends on several external libraries:

- **CUDA**: GPU acceleration for computer vision
- **OpenCV**: Computer vision algorithms
- **Zenoh**: Communication middleware
- **TensorRT**: Neural network inference acceleration
- **Team02-Libs**: Custom libraries for hardware interface

## License

This project is licensed under the MIT License.

## Team

This project is developed by Team02 at SEAME Portugal 2025.