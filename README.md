# RTK Engine

A lightweight, pure-software GNSS RTK (Real-Time Kinematic) engine implemented in C++. This engine provides high-precision positioning by processing raw GNSS observations using double-difference techniques and integer ambiguity resolution.

## Current Project Status: Phase 4 Complete (Continuous Kinematic Engine)

The engine has reached a professional-grade milestone with full support for Multi-GNSS constellations, dual-frequency signal processing, and a recursive **Extended Kalman Filter (EKF)** for smooth kinematic tracking.

### New Modular Architecture

The engine is organized into specialized, fully-documented modules:

- `include/rtk_engine/`
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
  - High-precision **Cycle Slip Detection** using Geometry-Free (GF) meter-level jumps.
- **Professional Multi-GNSS Solving:**
  - Jointly processes **GPS, GLONASS, and Galileo** observations in a single least-squares loop.
  - Achieves **sub-decimeter accuracy (~4cm)** at 1km baselines.
- **Commercial-Grade Reliability:**
  - **RAIM (Receiver Autonomous Integrity Monitoring):** Automatically detects and rejects faulty satellite signals (outliers) using statistical residual analysis. Proven resilience against 50m+ multipath errors.
- **Advanced Signal Processing:**
  - **Ionosphere-Free (IF):** Dual-frequency combination (L1/L2, E1/E5b) to eliminate atmospheric errors over long baselines (>50km).
- **Real-Time Data Ingestion:**
  - Built-in **NTRIP Client** and robust **NMEA Parser** (GGA/RMC).
  - Decodes RTCM v3.x and parses RINEX 3.x files.

## Limitations (What it CANT do - yet)

- **Inertial Fusion (IMU):** While the EKF supports acceleration states, direct IMU (gyro/accel) data ingestion is planned for Phase 5.
- **Hardware Integration:** Does not include direct UART drivers; assumes data is piped from a system source.

## Building & Running

```bash
mkdir build && cd build
cmake ..
make
./demo_kinematic # Continuous EKF tracking demo with circular path
./rtk_engine    # Multi-GNSS DGPS Demo with RAIM outlier rejection
./test_rtcm     # RTCM3 decoder unit tests
```
