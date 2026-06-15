# Commercialization Roadmap

This document outlines the final engineering and business-readiness tasks required before the commercial launch of the RTK Engine.

## 1. API Stabilization & Extension
The current C-API is a foundation but needs "Product-Grade" features to support diverse integrations.
- [ ] **State Accessors:** Functions to retrieve the current EKF state (Position, Velocity, Attitude, Covariance).
- [ ] **Injection Interfaces:** Explicit functions for injecting IMU data and RTCM3 corrections from external sources.
- [ ] **Event System:** Callback registration for solution updates and system alerts.
- [ ] **Versioning:** `rtk_get_version()` and API versioning to ensure backward compatibility.

## 2. Professional Documentation
Commercial users require high-signal documentation to reduce integration friction.
- [ ] **Integration Manual:** Detailed guide on embedding the engine into embedded systems (ARM/Linux) and mobile platforms.
- [ ] **Protocol Reference:** Full documentation of supported RTCM3 messages, NMEA sentences, and proprietary outputs.
- [ ] **Performance Whitepaper:** Summary of benchmark results (RMS, TTFF) across various environments (Urban, Foliage, Open Sky).

## 3. Quality Assurance (Product Grade)
- [ ] **CI/CD Hardening:** Integration of `AddressSanitizer` (ASan) and `UndefinedBehaviorSanitizer` (UBSan) into the build pipeline.
- [ ] **Fuzz Testing:** Targeted fuzzing of the `Rtcm3Parser` and `NmeaParser` to ensure robustness against malformed data.
- [ ] **Long-Haul Stability:** 72-hour continuous processing stress tests to identify memory leaks or drift issues.

## 4. Security & Intellectual Property
- [ ] **Licensing Enforcement:** Implementation of a signature-based license file validator linked to hardware IDs (UUID/MAC).
- [ ] **Binary Obfuscation:** Evaluation of symbol stripping and obfuscation for the shared library (`librtk_engine.so`).
- [ ] **Vulnerability Management:** Formalization of the security response process in `SECURITY.md`.

## 5. Deployment & Support
- [ ] **Stable Packaging:** Production-ready `.deb` and `.rpm` repositories with GPG-signed packages.
- [ ] **Customer Support Portal:** Setup of a private issue tracker and knowledge base for commercial licensees.
- [ ] **SLA Definitions:** Standardized Support Level Agreements for enterprise customers.

## 6. Regulatory & Compliance
- [ ] **Standard Compliance:** Verification and documentation of NMEA 0183 and RTCM 3.3 compliance levels.
- [ ] **Certification Path:** (Optional) Investigation into ISO 26262 (Automotive) or DO-178C (Avionics) for high-safety sectors.
