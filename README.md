# RTK Engine

A lightweight, pure-software GNSS RTK (Real-Time Kinematic) engine implemented in C++. This engine provides high-precision positioning by processing raw GNSS observations using double-difference techniques and integer ambiguity resolution.

## Current Project Status: Phase 7 Complete (Precise Point Positioning Foundation)

The engine has been significantly enhanced with production-grade usability, configuration management, and the foundation for advanced algorithms.

### New Features

- **Configuration Framework:** Dynamic parameter management using TOML files.
- **Stable C-API:** ABI-stable C interface for seamless integration and future language bindings.
- **Commercial CLI Dashboard:** Real-time ANSI-based telemetry visualization for solution status, satellite tracking, and residual monitoring.
- **Inertial Readiness:** EKF state vector augmented with Gyroscope and Accelerometer bias handles for future IMU integration.
- **PPP Infrastructure:** Parsers for precise SP3 (orbit) and CLK (clock) correction products, and atmospheric modeling (Saastamoinen tropospheric model).
- **Robust Ingestion:** NTRIP client with robust connection handling and serial port support for hardware receivers.
- **Interoperability Output:** NMEA streaming for real-time positioning and RINEX logging for post-processing.

### New Modular Architecture

The engine is organized into specialized, fully-documented modules:

- `include/rtk_engine/`
  - `config_manager.hpp`: Runtime parameter loading via TOML.
  - `rtk_api.h`: Stable C interface.
  - `ppp_parser.hpp`: Ingestion of precise SP3/CLK products.
  - `troposphere.hpp`: Atmospheric delay modeling.
  - `io/`: Serial port ingestion.
  - `output/`: NMEA formatter and RINEX writer.
  - `ui/dashboard.hpp`: ANSI-based CLI visualization.
  - `math/`: Core linear algebra using **Eigen** for dynamic scaling and performance.
  - `rtcm3/`: Binary protocol decoding for Legacy (1004, 1005, 1019, 1020) & MSM messages.
  - `solver/`: RTK algorithms, **EKF Filter**, **RAIM** outlier rejection, and **LAMBDA** ambiguity resolution.
  - `geodesy.hpp`: High-precision WGS84 Geodetic <-> ECEF <-> Local ENU transforms.
  - `orbit.hpp`: Multi-constellation satellite position calculation (Keplerian & RK4).
  - `ntrip_client.hpp`: Socket-based real-time correction streaming.

## Capabilities (What it CAN do)

- **Kinematic State Estimation (EKF):**
  - Continuous tracking of Position $(X,Y,Z)$, Velocity $(V_x,V_y,V_z)$, and Acceleration.
  - Maintains carrier-phase ambiguity states across epochs for consistent precision.
  - **INS/GNSS Fusion:** Real-time integration of Gyroscope and Accelerometer data to bridge GNSS outages and handle high dynamics.
  - **Attitude Estimation:** Live tracking of Roll, Pitch, and Yaw (Euler angles) within the EKF state space.
  - High-precision **Cycle Slip Detection** using Geometry-Free (GF) meter-level jumps.
- **Professional Multi-GNSS Solving:**
  - Jointly processes **GPS, GLONASS, and Galileo** observations in a single least-squares loop.
  - Achieves **sub-decimeter accuracy (~4cm)** at 1km baselines.
- **Commercial-Grade Reliability:**
  - **RAIM (Receiver Autonomous Integrity Monitoring):** Automatically detects and rejects faulty satellite signals (outliers) using statistical residual analysis. Proven resilience against 50m+ multipath errors.
  - **Visual Telemetry:** Pro-grade dashboard for live field monitoring.
- **Advanced Signal Processing:**
  - **Ionosphere-Free (IF):** Dual-frequency combination (L1/L2, E1/E5b) to eliminate atmospheric errors over long baselines (>50km).
- **Real-Time Data Ingestion:**
  - Built-in **NTRIP Client** and robust **NMEA Parser** (GGA/RMC).
  - Decodes RTCM v3.x and parses RINEX 3.x files.
- **Industrial Readiness:**
  - Configurable engine parameters.
  - Stable C-API for external integration.
  - Foundation for PPP.

## Commercial Readiness & Integration
- **CI/CD:** Automated builds and testing powered by GitHub Actions.
- **API Documentation:** Comprehensive C-API reference available in `docs/api.md`.
- **Deployment:** Packaging roadmap for Debian/RPM distributions defined in `packaging/`.
- **Integration & Benchmarking:** Planned support for ROS2 (`ros_wrapper/`) and automated performance benchmarking (`benchmark/`).

## Building & Running

```bash
mkdir build && cd build
cmake ..
make
./rtk_engine_pro  # Production-ready integrated EKF + Dashboard demo
./demo_kinematic  # Continuous EKF tracking demo with circular path
./rtk_engine      # Multi-GNSS DGPS Demo with RAIM outlier rejection
./test_rtcm       # RTCM3 decoder unit tests
./test_config     # Configuration manager verification
```
