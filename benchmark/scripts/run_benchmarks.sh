#!/bin/bash
# Professional RTK Benchmarking Orchestrator.
# This script manages the high-level execution flow of the benchmarking suite.

set -e

# Configuration
SCRIPTS_DIR="benchmark/scripts"
DATA_DIR="benchmark/data"
DATASET_NAME="urban_canyon"
TRUTH_FILE="$DATA_DIR/$DATASET_NAME/truth.nmea"

echo "--------------------------------------------------------"
echo "  GEMINI-RTK: PROFESSIONAL BENCHMARKING SUITE"
echo "--------------------------------------------------------"

# 1. Environment Check
if [ ! -f "build/rtk_tool" ]; then
    echo "[ERROR] Engine binary (build/rtk_tool) not found. Please build the project."
    exit 1
fi

# 2. Data Preparation (Ensure truth file exists for demo)
if [ ! -f "$TRUTH_FILE" ]; then
    echo "[INFO] Creating mock truth data for demonstration..."
    mkdir -p "$(dirname "$TRUTH_FILE")"
    # Create a small dummy truth file with GGA sentences
    echo "\$GPGGA,120000.00,3725.3220,N,12205.0415,W,1,08,1.0,30.0,M,0.0,M,,*6A" > "$TRUTH_FILE"
    echo "\$GPGGA,120001.00,3725.3221,N,12205.0416,W,1,08,1.0,30.1,M,0.0,M,,*6B" >> "$TRUTH_FILE"
fi

# 3. Execute Pipeline
python3 "$SCRIPTS_DIR/runner.py" \
    --tool "build/rtk_tool" \
    --dataset "$DATA_DIR/$DATASET_NAME" \
    --truth "$TRUTH_FILE"

echo "--------------------------------------------------------"
echo "  BENCHMARKING PIPELINE COMPLETE"
echo "--------------------------------------------------------"
