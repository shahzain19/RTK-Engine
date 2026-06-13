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

## Phase 5: Optimization & Commercial Polish (Active)
- [ ] **Inertial Integration (Optional):** Loosely or tightly coupled integration with IMU data.
- [ ] **Web Dashboard:** Simple CLI or Web interface to visualize residuals and position tracks.
- [ ] **Unit Testing:** Comprehensive test suite using a framework like GTest.
- [ ] **Performance Profiling:** Optimize EKF update steps for 10Hz+ real-time operation.
