# RTK Engine Roadmap

This document outlines the development phases for the Lightweight Pure-Software RTK Engine.

## Phase 1: Foundation & Refinement (Complete)
- [x] Basic Geodesy (ECEF/Geodetic/ENU).
- [x] NMEA 0183 Parser (GGA, RMC).
- [x] Klobuchar Ionospheric Model.
- [x] Double-Difference Pseudorange Solver (DGPS).
- [x] LAMBDA Integer Ambiguity Resolution.

## Phase 2: Real Data Ingestion & Modularization (Complete)
- [x] **RTCM3 Decoder:** Full framing and decoding for legacy/MSM messages.
- [x] **RINEX Parser:** Support for Observation and Navigation (GPS/GLO) files.
- [x] **NTRIP Client:** Socket-based client for real-time correction ingestion.
- [x] **Orbit Engine:** Satellite position calculation for GPS/GAL/BDS and GLONASS.

## Phase 3: Multi-GNSS & Multi-Frequency (Complete)
- [x] **Eigen Integration:** High-performance dynamic matrix math.
- [x] **Multi-Constellation Solver:** Simultaneous GPS, GLONASS, and Galileo processing.
- [x] **RAIM Implementation:** Residual-based outlier detection and rejection.
- [x] **Dual-Frequency Processing:** Ionosphere-Free (IF) combinations.

## Phase 4: Kinematic Processing & Filtering (Complete)
- [x] **Extended Kalman Filter (EKF):** 
  - Transition from snapshot solving to recursive state estimation.
  - Tracking of Position, Velocity, and Float Ambiguities across epochs.
- [x] **Cycle Slip Detection:** Geometry-Free (GF) meter-level detection to manage ambiguity resets.
- [x] **Kinematic Validation:** Successful tracking of moving trajectories with rapid recovery.

## Phase 5: Optimization & Commercial Polish (Complete)
- [x] **Inertial Readiness:** Augmented EKF state vector with gyro/accel bias handles.
- [x] **Pro Dashboard:** ANSI-based CLI interface for real-time telemetry and visualization.
- [x] **Unit Testing:** Basic test suite for protocol decoding and coordinate math.
- [x] **Performance Profiling:** Verified 10Hz-100Hz real-time processing capabilities.

## Phase 6: Commercial Readiness (Complete)
- [x] **Configuration Framework:** TOML-based parameter management using `toml.hpp`.
- [x] **Stable C-API:** ABI-stable C interface (`rtk_api.h`) for integration with other languages.
- [x] **INS/GNSS Fusion:** Full integration of IMU (Gyro/Accel) data into the EKF prediction loop.
- [x] **Attitude Tracking:** Real-time Roll, Pitch, Yaw estimation.
- [x] **Modular Refactoring:** Clean separation of solver, protocol, and UI layers.

## Phase 7: PPP & Future (In Progress)
- [x] **Precise Products:** Ingestion of SP3 (orbits) and CLK (clocks) precise correction products.
- [x] **Atmospheric Modeling:** Saastamoinen tropospheric delay model implementation.
- [x] **Ionospheric Refinement:** Integration of Klobuchar and dual-frequency IF combinations.
- [ ] **PPP EKF Solver:** Full integration of precise products into the EKF state space for decimeter-level global positioning.
- [ ] **Moving Base RTK:** Support for relative positioning between two moving platforms.
