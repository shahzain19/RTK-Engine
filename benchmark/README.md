# RTK Engine Benchmarking Suite

## Overview
The RTK Engine Benchmarking Suite is a professional-grade performance evaluation framework designed to quantify the accuracy, reliability, and convergence characteristics of the GNSS processing engine. It follows industry standards for GNSS performance metrics (ISO 17123-8).

## Architecture
The suite is composed of three modular layers:
1.  **Data Layer (`ingest.py`):** Handles automated retrieval, decompression, and organization of public GNSS datasets (RINEX, SP3, CLK).
2.  **Execution Layer (`benchmark_runner.py`):** Orchestrates the execution of `rtk_tool` across multiple datasets using a configuration-driven approach.
3.  **Analysis Layer (`analyze.py`):** A modular analysis engine that computes RMS errors, convergence times, and fix availability.

## Directory Structure
- `data/`: Staging area for downloaded GNSS datasets.
- `scripts/`: Implementation of the benchmarking pipeline.
  - `ingest.py`: Dataset ingestion module.
  - `analyze.py`: Performance analysis module.
  - `runner.py`: Orchestration logic.
- `reports/`: (Auto-generated) JSON/CSV reports and performance plots.

## Performance Metrics
- **Horizontal/Vertical RMS:** Root Mean Square error in North, East, and Up directions.
- **CEP95 / R95:** Circular Error Probable (95th percentile).
- **Time to First Fix (TTFF):** Time from cold start to any valid GNSS solution.
- **Time to RTK Fixed:** Time required for the EKF to resolve integer ambiguities.
- **Fix Availability:** Percentage of epochs where a high-precision solution is available.

## Usage

### 1. Ingest Data
Download a specific dataset using the ingestion tool:
```bash
python3 benchmark/scripts/ingest.py --dataset urban_canyon --url <RINEX_URL>
```

### 2. Run Benchmarks
Execute the benchmarking pipeline:
```bash
./benchmark/scripts/run_benchmarks.sh
```

### 3. Analyze Results
The runner automatically invokes the analysis module, but it can be used independently:
```bash
python3 benchmark/scripts/analyze.py --solution output.nmea --truth truth.nmea --format json
```

## Contributing
When adding new datasets, please ensure they include a high-precision "truth" trajectory (e.g., from a high-end reference receiver) for valid comparison.
