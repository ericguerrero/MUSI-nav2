#!/usr/bin/env python3
"""
SLAM Quality Metrics Evaluation Script

Analyzes generated maps to provide quantitative quality metrics.
Helps students objectively compare different SLAM configurations.

Usage:
    python3 slam_metrics.py <map_yaml_file>

Example:
    python3 slam_metrics.py /shared/maps/high_detail.yaml
"""

import sys
import yaml
import numpy as np
from PIL import Image
from pathlib import Path


def load_map(yaml_file):
    """Load occupancy grid map from YAML and PGM files."""
    yaml_path = Path(yaml_file)

    if not yaml_path.exists():
        raise FileNotFoundError(f"Map YAML file not found: {yaml_file}")

    with open(yaml_path, 'r') as f:
        map_config = yaml.safe_load(f)

    # Load PGM image
    pgm_file = yaml_path.parent / map_config['image']

    if not pgm_file.exists():
        raise FileNotFoundError(f"Map image file not found: {pgm_file}")

    img = Image.open(pgm_file)
    occupancy_grid = np.array(img)

    return occupancy_grid, map_config


def calculate_map_entropy(occupancy_grid):
    """
    Calculate Shannon entropy of the occupancy grid.
    Higher entropy = more information density.

    Occupancy values:
    - 0 (black): Occupied
    - 100-254 (white): Free
    - 205 (gray): Unknown
    """
    # Normalize to probability distribution
    unique, counts = np.unique(occupancy_grid, return_counts=True)
    probabilities = counts / counts.sum()

    # Calculate Shannon entropy: H = -sum(p * log2(p))
    entropy = -np.sum(probabilities * np.log2(probabilities + 1e-10))

    return entropy


def calculate_map_coverage(occupancy_grid):
    """
    Calculate percentage of map that is known (not unknown).
    Higher coverage = more area explored.
    """
    # Unknown is typically value 205 (gray in PGM)
    unknown_pixels = np.sum(occupancy_grid == 205)
    total_pixels = occupancy_grid.size

    known_percentage = ((total_pixels - unknown_pixels) / total_pixels) * 100

    return known_percentage


def calculate_obstacle_density(occupancy_grid):
    """
    Calculate density of obstacles in the map.
    Helps identify if SLAM is over-detecting or under-detecting obstacles.
    """
    # Occupied is typically value 0 (black in PGM)
    occupied_pixels = np.sum(occupancy_grid == 0)

    # Free space is typically 254 (white in PGM)
    free_pixels = np.sum(occupancy_grid == 254)

    # Calculate ratio of obstacles to known space
    known_space = occupied_pixels + free_pixels

    if known_space > 0:
        obstacle_ratio = (occupied_pixels / known_space) * 100
    else:
        obstacle_ratio = 0.0

    return obstacle_ratio


def calculate_map_efficiency(occupancy_grid, resolution):
    """
    Calculate map efficiency: information per storage byte.
    Higher = more efficient representation.
    """
    # Map file size in KB
    map_size_kb = occupancy_grid.nbytes / 1024

    # Known cells (not unknown)
    known_cells = np.sum(occupancy_grid != 205)

    # Efficiency: known cells per KB
    efficiency = known_cells / map_size_kb if map_size_kb > 0 else 0

    return efficiency, map_size_kb


def analyze_map_consistency(occupancy_grid):
    """
    Detect potential mapping artifacts or inconsistencies.
    Looks for isolated occupied cells (noise) or gaps in walls.
    """
    # Count isolated occupied pixels (potential noise)
    # This is a simple check - real noise detection would use morphological operations

    occupied_mask = (occupancy_grid == 0).astype(np.uint8)

    # Simple artifact detection: occupied pixels with no occupied neighbors
    from scipy import ndimage

    # Dilate occupied regions
    dilated = ndimage.binary_dilation(occupied_mask, iterations=1)
    eroded = ndimage.binary_erosion(dilated, iterations=1)

    # Pixels that disappeared after erosion are likely noise
    noise_pixels = np.sum(occupied_mask != eroded)
    noise_percentage = (noise_pixels / np.sum(occupied_mask)) * 100 if np.sum(occupied_mask) > 0 else 0

    return noise_percentage


def print_metrics(yaml_file):
    """Print all calculated metrics for a map."""

    print("=" * 60)
    print(f"SLAM Map Quality Metrics")
    print("=" * 60)
    print(f"Map: {yaml_file}")
    print()

    try:
        occupancy_grid, map_config = load_map(yaml_file)
        resolution = map_config['resolution']

        # Calculate metrics
        entropy = calculate_map_entropy(occupancy_grid)
        coverage = calculate_map_coverage(occupancy_grid)
        obstacle_density = calculate_obstacle_density(occupancy_grid)
        efficiency, file_size = calculate_map_efficiency(occupancy_grid, resolution)
        noise = analyze_map_consistency(occupancy_grid)

        # Print results
        print(f"Resolution:           {resolution} m/pixel")
        print(f"Map dimensions:       {occupancy_grid.shape[1]} x {occupancy_grid.shape[0]} pixels")
        print(f"Map size:             {file_size:.2f} KB")
        print()
        print(f"Entropy:              {entropy:.3f} bits")
        print(f"  → Information density (higher = more detail captured)")
        print()
        print(f"Coverage:             {coverage:.1f}%")
        print(f"  → Percentage of map explored (not unknown)")
        print()
        print(f"Obstacle density:     {obstacle_density:.1f}%")
        print(f"  → Ratio of obstacles to free space")
        print()
        print(f"Map efficiency:       {efficiency:.0f} cells/KB")
        print(f"  → Information per storage unit (higher = better)")
        print()
        print(f"Consistency score:    {100 - noise:.1f}%")
        print(f"  → Lower noise = more consistent mapping")
        print()

        # Overall quality score (simple weighted average)
        # Normalize metrics to 0-100 scale
        norm_entropy = min(entropy / 8.0 * 100, 100)  # Typical max entropy ~8 bits
        norm_coverage = coverage
        norm_efficiency = min(efficiency / 1000 * 100, 100)  # Typical range
        norm_consistency = 100 - noise

        quality_score = (norm_entropy * 0.3 + norm_coverage * 0.3 +
                        norm_efficiency * 0.2 + norm_consistency * 0.2)

        print("=" * 60)
        print(f"Overall Quality Score: {quality_score:.1f}/100")
        print("=" * 60)

        # Interpretation
        print()
        print("Interpretation:")
        if quality_score >= 80:
            print("  ✓ Excellent map quality")
        elif quality_score >= 60:
            print("  ✓ Good map quality")
        elif quality_score >= 40:
            print("  ⚠ Acceptable map quality, consider tuning parameters")
        else:
            print("  ✗ Poor map quality, significant tuning needed")

    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error analyzing map: {e}")
        sys.exit(1)


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 slam_metrics.py <map_yaml_file>")
        print("Example: python3 slam_metrics.py /shared/maps/my_map.yaml")
        sys.exit(1)

    yaml_file = sys.argv[1]
    print_metrics(yaml_file)


if __name__ == "__main__":
    main()
