#!/usr/bin/env python3
"""
GNSS Benchmark Runner.
Orchestrates the execution of rtk_tool against various datasets and manages reports.
"""

import os
import logging
import argparse
import subprocess
import time
from pathlib import Path
from typing import Dict, List

# Setup professional logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger("BenchmarkRunner")

class BenchmarkRunner:
    """
    Orchestrates the end-to-end benchmarking pipeline.
    """
    
    def __init__(self, rtk_tool_path: str = "build/rtk_tool", output_dir: str = "benchmark/reports"):
        self.rtk_tool = Path(rtk_tool_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if not self.rtk_tool.exists():
            raise FileNotFoundError(f"Engine binary not found at {self.rtk_tool}")

    def run_case(self, dataset_path: Path, config_template: str = "config.toml") -> Path:
        """
        Runs the RTK engine for a single benchmark case.
        """
        case_name = dataset_path.name
        report_file = self.output_dir / f"{case_name}_output.nmea"
        
        logger.info(f"Running benchmark case: {case_name}")
        
        # In a real scenario, we might dynamically generate a config.toml here
        # For now, we assume rtk_tool is configured to read from the dataset folder
        start_time = time.time()
        try:
            with open(report_file, "w") as out:
                subprocess.run(
                    [str(self.rtk_tool), config_template],
                    stdout=out,
                    stderr=subprocess.PIPE,
                    check=True,
                    timeout=300 # 5-minute safety timeout
                )
            duration = time.time() - start_time
            logger.info(f"Execution complete in {duration:.1f}s. Results: {report_file}")
            return report_file
        except subprocess.TimeoutExpired:
            logger.error(f"Benchmark {case_name} timed out.")
            raise
        except subprocess.CalledProcessError as e:
            logger.error(f"Engine failed with exit code {e.returncode}")
            raise

    def analyze(self, solution_file: Path, truth_file: Path):
        """
        Invokes the analysis script on the resulting solution.
        """
        logger.info(f"Analyzing {solution_file} against {truth_file}...")
        subprocess.run([
            "python3", "benchmark/scripts/analyze.py",
            "--solution", str(solution_file),
            "--truth", str(truth_file)
        ])

def main():
    parser = argparse.ArgumentParser(description="Professional GNSS Benchmark Runner")
    parser.add_argument("--tool", default="build/rtk_tool", help="Path to rtk_tool binary")
    parser.add_argument("--dataset", required=True, help="Path to the dataset directory")
    parser.add_argument("--truth", required=True, help="Path to the truth NMEA file")
    
    args = parser.parse_args()
    
    try:
        runner = BenchmarkRunner(args.tool)
        solution = runner.run_case(Path(args.dataset))
        runner.analyze(solution, Path(args.truth))
    except Exception as e:
        logger.error(f"Pipeline failed: {e}")
        exit(1)

if __name__ == "__main__":
    main()
