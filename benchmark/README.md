# Benchmarking Suite Plan

## Objective
To provide standardized performance reports (accuracy vs. convergence time) using public GNSS datasets.

## Dataset Selection
- Focus on high-dynamic and challenging environments (e.g., urban canyon, foliage).
- Recommended: [IGN GNSS Data](https://data.ign.fr/) or [NASA Crustal Dynamics Data](https://cddis.nasa.gov/).

## Benchmarking Pipeline
1.  **Ingestion Script (`benchmark/scripts/ingest.py`):** Convert downloaded RINEX/SP3/CLK files into formats compatible with `rtk_tool`.
2.  **Execution:** Run `rtk_tool` against the prepared dataset.
3.  **Analysis Script (`benchmark/scripts/analyze.py`):** Compare the output solution against the truth trajectory provided in the dataset.
4.  **Reporting:** Generate a summary report highlighting:
    - RMS Error (North, East, Up).
    - Convergence time to "RTK Fixed".
    - Outlier rejection frequency.
