#!/usr/bin/env python3
"""
GNSS Data Ingestion Module.
Provides professional-grade tools for downloading and managing benchmarking datasets.
"""

import os
import logging
import argparse
import requests
import gzip
import shutil
from pathlib import Path

# Configure professional logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger("Ingestor")

class DatasetIngestor:
    """
    Manages the lifecycle of GNSS datasets including downloading and extraction.
    """
    
    def __init__(self, base_dir: str = "benchmark/data"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def download(self, url: str, dataset_name: str) -> Path:
        """
        Downloads a file from a URL and stores it in a dataset-specific folder.
        
        Args:
            url: The remote URL of the GNSS data file.
            dataset_name: Name of the dataset (e.g., 'urban_canyon_01').
            
        Returns:
            Path to the downloaded (and potentially decompressed) file.
        """
        dest_folder = self.base_dir / dataset_name
        dest_folder.mkdir(exist_ok=True)
        
        file_name = url.split('/')[-1]
        local_path = dest_folder / file_name
        
        logger.info(f"Downloading {url} to {local_path}...")
        
        try:
            with requests.get(url, stream=True) as r:
                r.raise_for_status()
                with open(local_path, 'wb') as f:
                    for chunk in r.iter_content(chunk_size=1024 * 1024):
                        f.write(chunk)
            
            # Post-processing: Handle compression
            if local_path.suffix == '.gz':
                return self._decompress_gzip(local_path)
            
            return local_path
            
        except Exception as e:
            logger.error(f"Failed to download dataset: {e}")
            raise

    def _decompress_gzip(self, file_path: Path) -> Path:
        """Decompresses a GZIP file and removes the original."""
        decompressed_path = file_path.with_suffix('')
        logger.info(f"Decompressing {file_path}...")
        
        with gzip.open(file_path, 'rb') as f_in:
            with open(decompressed_path, 'wb') as f_out:
                shutil.copyfileobj(f_in, f_out)
        
        file_path.unlink() # Remove compressed version to save space
        return decompressed_path

def main():
    parser = argparse.ArgumentParser(description="Professional GNSS Data Ingestion Tool")
    parser.add_argument("--url", help="URL of the GNSS dataset file")
    parser.add_argument("--dataset", default="default", help="Descriptive name for the dataset")
    parser.add_argument("--output_dir", default="benchmark/data", help="Root directory for datasets")
    
    args = parser.parse_args()
    
    if not args.url:
        logger.info("No URL provided. Initializing directory structure.")
        DatasetIngestor(args.output_dir)
        return

    ingestor = DatasetIngestor(args.output_dir)
    try:
        final_path = ingestor.download(args.url, args.dataset)
        logger.info(f"Ingestion successful: {final_path}")
    except Exception:
        exit(1)

if __name__ == "__main__":
    main()
