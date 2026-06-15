#!/usr/bin/env python3
"""
GNSS Performance Analysis Engine.
Calculates high-precision metrics by comparing engine output against truth trajectories.
"""

import os
import logging
import argparse
import json
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger("Analyzer")

@dataclass
class GpsEpoch:
    """Represents a single GNSS solution epoch."""
    timestamp: str
    lat: float
    lon: float
    alt: float
    fix_quality: int

class NmeaParser:
    """Robust parser for NMEA 0183 sentences (GGA focus)."""
    
    @staticmethod
    def parse_gga(line: str) -> Optional[GpsEpoch]:
        """Parses a single GGA sentence into a GpsEpoch."""
        if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
            return None
            
        parts = line.split(',')
        if len(parts) < 10:
            return None
            
        try:
            # Timestamp (UTC)
            time_str = parts[1]
            
            # Latitude: DDMM.MMMM -> Decimal Degrees
            lat_raw = float(parts[2])
            lat_deg = int(lat_raw / 100) + (lat_raw % 100) / 60.0
            if parts[3] == 'S': lat_deg = -lat_deg
            
            # Longitude: DDDMM.MMMM -> Decimal Degrees
            lon_raw = float(parts[4])
            lon_deg = int(lon_raw / 100) + (lon_raw % 100) / 60.0
            if parts[5] == 'W': lon_deg = -lon_deg
            
            fix_quality = int(parts[6])
            altitude = float(parts[9])
            
            return GpsEpoch(time_str, lat_deg, lon_deg, altitude, fix_quality)
        except (ValueError, IndexError):
            return None

class MetricsCalculator:
    """Computes industry-standard GNSS performance metrics."""
    
    LAT_TO_METERS = 111319.5 # Approx meters per degree latitude

    def __init__(self, solution: List[GpsEpoch], truth: List[GpsEpoch]):
        self.solution = solution
        self.truth_map = {e.timestamp: e for e in truth}
        
    def calculate(self) -> Dict:
        """Runs the metric calculation pipeline."""
        errors_n, errors_e, errors_u = [], [], []
        matched_epochs = 0
        
        first_rtk_fixed = None
        start_time = self.solution[0].timestamp if self.solution else None
        
        for sol in self.solution:
            if first_rtk_fixed is None and sol.fix_quality == 4:
                first_rtk_fixed = sol.timestamp
                
            if sol.timestamp in self.truth_map:
                truth = self.truth_map[sol.timestamp]
                matched_epochs += 1
                
                # Simple ENU error approximation
                dn = (sol.lat - truth.lat) * self.LAT_TO_METERS
                de = (sol.lon - truth.lon) * self.LAT_TO_METERS * np.cos(np.radians(truth.lat))
                du = sol.alt - truth.alt
                
                errors_n.append(dn)
                errors_e.append(de)
                errors_u.append(du)

        if not errors_n:
            return {"error": "No overlapping epochs found for analysis."}

        # RMS Calculation
        rms_n = np.sqrt(np.mean(np.array(errors_n)**2))
        rms_e = np.sqrt(np.mean(np.array(errors_e)**2))
        rms_u = np.sqrt(np.mean(np.array(errors_u)**2))
        
        # Convergence
        convergence = self._calc_time_diff(start_time, first_rtk_fixed) if first_rtk_fixed else None

        return {
            "summary": {
                "matched_epochs": matched_epochs,
                "total_epochs": len(self.solution),
                "fix_availability": (matched_epochs / len(self.solution)) * 100 if self.solution else 0
            },
            "accuracy": {
                "rms_north_m": round(rms_n, 4),
                "rms_east_m": round(rms_e, 4),
                "rms_up_m": round(rms_u, 4),
                "rms_horizontal_m": round(np.sqrt(rms_n**2 + rms_e**2), 4)
            },
            "convergence": {
                "time_to_rtk_fixed_s": convergence
            }
        }

    def _calc_time_diff(self, t1: str, t2: str) -> float:
        """Calculates difference in seconds between two HHMMSS.SS strings."""
        def to_sec(s):
            return int(s[:2])*3600 + int(s[2:4])*60 + float(s[4:])
        return to_sec(t2) - to_sec(t1)

def main():
    parser = argparse.ArgumentParser(description="Professional GNSS Analysis Tool")
    parser.add_argument("--solution", required=True, help="NMEA output from engine")
    parser.add_argument("--truth", required=True, help="Reference truth NMEA file")
    parser.add_argument("--format", choices=['text', 'json'], default='text', help="Output format")
    
    args = parser.parse_args()

    # Load data
    sol_epochs = []
    with open(args.solution, 'r') as f:
        for line in f:
            epoch = NmeaParser.parse_gga(line)
            if epoch: sol_epochs.append(epoch)

    truth_epochs = []
    with open(args.truth, 'r') as f:
        for line in f:
            epoch = NmeaParser.parse_gga(line)
            if epoch: truth_epochs.append(epoch)

    # Calculate
    calc = MetricsCalculator(sol_epochs, truth_epochs)
    results = calc.calculate()

    # Report
    if args.format == 'json':
        print(json.dumps(results, indent=2))
    else:
        print("\n" + "="*40)
        print("        GNSS PERFORMANCE REPORT")
        print("="*40)
        if "error" in results:
            print(f"ERROR: {results['error']}")
        else:
            s, a, c = results['summary'], results['accuracy'], results['convergence']
            print(f"Matched Epochs:    {s['matched_epochs']} / {s['total_epochs']}")
            print(f"Fix Availability:  {s['fix_availability']:.1f}%")
            print("-" * 40)
            print(f"RMS Horizontal:    {a['rms_horizontal_m']:.3f} m")
            print(f"RMS Vertical:      {a['rms_up_m']:.3f} m")
            print("-" * 40)
            conv = f"{c['time_to_rtk_fixed_s']:.1f} s" if c['time_to_rtk_fixed_s'] else "N/A"
            print(f"Time to RTK Fix:   {conv}")
        print("="*40 + "\n")

if __name__ == "__main__":
    main()
