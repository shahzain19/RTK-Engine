# RTK Engine Roadmap

This document outlines the development phases for the Lightweight Pure-Software RTK Engine.

## Phase 1: Foundation & Refinement (Complete)
- [x] Basic Geodesy (ECEF/Geodetic/ENU).
- [x] NMEA 0183 Parser (GGA, RMC).
- [x] Klobuchar Ionospheric Model.
- [x] Double-Difference Pseudorange Solver (DGPS).
- [x] LAMBDA Integer Ambiguity Resolution.
- [x] Single-epoch RTK Fixed Solver (GPS L1).
- [x] Mock observation generator for validation.

## Phase 2: Real Data Ingestion & Modularization (Complete)
- [x] **RTCM3 Decoder:** Full framing, CRC-24, and decoding for messages 1004, 1005, 1019, and 1020.
- [x] **RINEX Parser:** Support for RINEX 3.x observation and navigation (GPS/GLO) files.
- [x] **NTRIP Client:** Socket-based client for real-time correction ingestion.
- [x] **Orbit Engine:** ECEF satellite position calculation for GPS/GAL/BDS (Keplerian) and GLONASS (RK4).
- [x] **Codebase Documentation:** Comprehensive Doxygen-style comments across all modules.

## Phase 3: Multi-GNSS & Multi-Frequency (Active)
- [ ] **Multi-Constellation Solver:** Extend `RtkSolver` to include GLONASS, Galileo, and BeiDou in the double-difference engine simultaneously.
- [x] **RAIM Implementation:** Residual-based outlier detection and rejection for commercial reliability.
- [ ] **Dual-Frequency Processing:** Implement Ionosphere-Free (IF) combinations in the main solving loop to eliminate ionospheric error.
- [ ] **Ambiguity Resolution Enhancements:** Use Wide-Lane (WL) combinations to resolve ambiguities faster over long baselines.
- [ ] **Advanced Tropospheric Models:** Integrate Saastamoinen or Hopfield models with mapping functions (NMF/VMF).

## Phase 4: Kinematic Processing & Filtering
- [ ] **Extended Kalman Filter (EKF):** 
  - Transition from epoch-by-epoch least squares to a recursive EKF state estimator.
  - State vector including Position (XYZ), Velocity (VxVyVz), and Float Ambiguities (N1...Nn).
- [ ] **Cycle Slip Detection:** Implement Geometry-Free (GF) or Doppler-based slip detection to reset ambiguity states in the EKF.
- [ ] **Inertial Integration (Optional):** Loosely or tightly coupled integration with IMU data for dead-reckoning during GNSS outages.

## Phase 5: Optimization & Tooling
- [ ] **Library Integration:** Replace custom matrix math with Eigen for performance and flexibility.
- [ ] **Web Dashboard:** Simple CLI or Web interface to visualize residuals, satellite skyplot, and position tracks.
- [ ] **Unit Testing:** Comprehensive test suite for every module using a framework like GTest.
