#!/usr/bin/env python3
"""
Forward Dynamics Algorithm Benchmark Visualization

This script reads benchmark results from CSV and generates plots comparing
the performance of ABA (Articulated Body Algorithm) and CRBA (Composite
Rigid Body Algorithm) on different kinematic tree structures.

Handles:
- Skipped CRBA values (negative values in CSV)
- Timeout values (negative ABA values)
- Large body counts up to 2000+

Usage:
    python plot_benchmark.py [--input INPUT_CSV] [--output OUTPUT_DIR]
"""

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Set up matplotlib style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 12
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['legend.fontsize'] = 12


def load_benchmark_data(csv_path: str) -> pd.DataFrame:
    """Load benchmark results from CSV file."""
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)
    
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} benchmark results from {csv_path}")
    
    # Filter out timeout entries (negative ABA time)
    valid_df = df[df['aba_time_us'] >= 0].copy()
    if len(valid_df) < len(df):
        print(f"  Filtered out {len(df) - len(valid_df)} timeout entries")
    
    return valid_df


def plot_time_vs_bodies(df: pd.DataFrame, output_dir: str):
    """
    Plot execution time vs number of bodies for all tree types.
    Creates two separate plots: one for ABA and one for CRBA.
    """
    tree_types = df['tree_type'].unique()
    colors = {'chain': '#1f77b4', 'binary_tree': '#ff7f0e', 'star': '#2ca02c'}
    markers = {'chain': 'o', 'binary_tree': 's', 'star': '^'}
    labels = {'chain': 'Chain (Serial)', 'binary_tree': 'Binary Tree', 'star': 'Star (Depth-1)'}
    
    # Plot ABA times
    fig, ax = plt.subplots(figsize=(10, 6))
    for tree_type in tree_types:
        data = df[df['tree_type'] == tree_type]
        ax.plot(data['num_bodies'], data['aba_time_us'], 
                marker=markers.get(tree_type, 'o'),
                color=colors.get(tree_type, 'blue'),
                label=labels.get(tree_type, tree_type),
                linewidth=2, markersize=8)
    
    # Add O(n) reference line fitted to data
    n_range = np.array([df['num_bodies'].min(), df['num_bodies'].max()])
    # Fit to the chain data as reference
    chain_data = df[df['tree_type'] == 'chain']
    if len(chain_data) > 0:
        slope = chain_data['aba_time_us'].iloc[-1] / chain_data['num_bodies'].iloc[-1]
        ax.plot(n_range, n_range * slope, 'k--', alpha=0.8, linewidth=2.5, label='O(n) reference')

    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Execution Time (us)')
    ax.set_title('ABA (Articulated Body Algorithm)')
    ax.legend()
    ax.set_xscale('log', base=2)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'time_vs_bodies_aba.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()
    
    # Plot CRBA times (only valid values)
    fig, ax = plt.subplots(figsize=(10, 6))
    df_crba = df[df['crba_time_us'] >= 0]
    
    for tree_type in tree_types:
        data = df_crba[df_crba['tree_type'] == tree_type]
        if len(data) > 0:
            ax.plot(data['num_bodies'], data['crba_time_us'], 
                    marker=markers.get(tree_type, 'o'),
                    color=colors.get(tree_type, 'blue'),
                    label=labels.get(tree_type, tree_type),
                    linewidth=2, markersize=8)
    
    # Add O(n^3) reference line fitted to data
    if len(df_crba) > 0:
        n_range = np.array([df_crba['num_bodies'].min(), df_crba['num_bodies'].max()])
        # Fit to the chain data as reference
        chain_crba_data = df_crba[df_crba['tree_type'] == 'chain']
        if len(chain_crba_data) > 0:
            # Fit using the last data point
            scale = chain_crba_data['crba_time_us'].iloc[-1] / (chain_crba_data['num_bodies'].iloc[-1] ** 3)
            ax.plot(n_range, n_range ** 3 * scale, 'k--', alpha=0.8, linewidth=2.5, label='O(n³) reference')

    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Execution Time (us)')
    ax.set_title('CRBA (Composite Rigid Body Algorithm)')
    ax.legend()
    ax.set_xscale('log', base=2)
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'time_vs_bodies_crba.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_algorithm_comparison(df: pd.DataFrame, output_dir: str):
    """
    Plot ABA vs CRBA comparison for each tree type (only where both are available).
    """
    # Filter to entries with valid CRBA
    df_valid = df[df['crba_time_us'] >= 0].copy()
    
    tree_types = df_valid['tree_type'].unique()
    fig, axes = plt.subplots(1, len(tree_types), figsize=(6 * len(tree_types), 6))
    
    if len(tree_types) == 1:
        axes = [axes]
    
    titles = {'chain': 'Chain (Serial)', 'binary_tree': 'Binary Tree', 'star': 'Star (Depth-1)'}
    
    for idx, tree_type in enumerate(tree_types):
        ax = axes[idx]
        data = df_valid[df_valid['tree_type'] == tree_type]
        
        x = np.arange(len(data))
        width = 0.35
        
        bars1 = ax.bar(x - width/2, data['aba_time_us'], width, 
                       label='ABA', color='#1f77b4', alpha=0.8)
        bars2 = ax.bar(x + width/2, data['crba_time_us'], width, 
                       label='CRBA', color='#ff7f0e', alpha=0.8)
        
        ax.set_xlabel('Number of Bodies')
        ax.set_ylabel('Execution Time (us)')
        ax.set_title(titles.get(tree_type, tree_type))
        ax.set_xticks(x)
        ax.set_xticklabels(data['num_bodies'])
        ax.legend()
        ax.set_yscale('log')
        ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, 'algorithm_comparison.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_speedup_ratio(df: pd.DataFrame, output_dir: str):
    """
    Plot the speedup ratio (CRBA time / ABA time) for each tree type.
    Values > 1 indicate ABA is faster, < 1 indicate CRBA is faster.
    """
    # Filter to entries with valid CRBA
    df_valid = df[df['crba_time_us'] >= 0].copy()
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    tree_types = df_valid['tree_type'].unique()
    colors = {'chain': '#1f77b4', 'binary_tree': '#ff7f0e', 'star': '#2ca02c'}
    markers = {'chain': 'o', 'binary_tree': 's', 'star': '^'}
    labels = {'chain': 'Chain (Serial)', 'binary_tree': 'Binary Tree', 'star': 'Star (Depth-1)'}
    
    for tree_type in tree_types:
        data = df_valid[df_valid['tree_type'] == tree_type].copy()
        data['speedup'] = data['crba_time_us'] / data['aba_time_us']
        
        ax.plot(data['num_bodies'], data['speedup'], 
                marker=markers.get(tree_type, 'o'),
                color=colors.get(tree_type, 'blue'),
                label=labels.get(tree_type, tree_type),
                linewidth=2, markersize=8)
    
    ax.axhline(y=1.0, color='red', linestyle='--', linewidth=1.5, 
               label='Equal Performance', alpha=0.7)
    
    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Speedup Ratio (CRBA / ABA)')
    ax.set_title('Algorithm Speedup: Values > 1 indicate ABA is faster')
    ax.legend()
    ax.set_xscale('log', base=2)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, 'speedup_ratio.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_complexity_analysis(df: pd.DataFrame, output_dir: str):
    """
    Plot time normalized by theoretical complexity.
    ABA: O(n) - linear in number of bodies
    CRBA: O(n^3) for solving, O(n^2) for mass matrix construction
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    tree_types = df['tree_type'].unique()
    colors = {'chain': '#1f77b4', 'binary_tree': '#ff7f0e', 'star': '#2ca02c'}
    markers = {'chain': 'o', 'binary_tree': 's', 'star': '^'}
    labels = {'chain': 'Chain (Serial)', 'binary_tree': 'Binary Tree', 'star': 'Star (Depth-1)'}
    
    # ABA: time/n (should be roughly constant if O(n))
    ax = axes[0, 0]
    for tree_type in tree_types:
        data = df[df['tree_type'] == tree_type].copy()
        data['normalized'] = data['aba_time_us'] / data['num_bodies']
        ax.plot(data['num_bodies'], data['normalized'], 
                marker=markers.get(tree_type, 'o'),
                color=colors.get(tree_type, 'blue'),
                label=labels.get(tree_type, tree_type),
                linewidth=2, markersize=8)
    
    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Time / n (us)')
    ax.set_title('ABA: Time normalized by n (O(n) analysis)')
    ax.legend()
    ax.set_xscale('log', base=2)
    ax.grid(True, alpha=0.3)
    
    # CRBA: time/n^3 (should be roughly constant if O(n^2))
    ax = axes[0, 1]
    df_crba = df[df['crba_time_us'] >= 0]
    for tree_type in tree_types:
        data = df_crba[df_crba['tree_type'] == tree_type].copy()
        data = data[4:]
        if len(data) > 0:
            data['normalized'] = data['crba_time_us'] / (0.01 * data['num_bodies'] ** 3)
            ax.plot(data['num_bodies'], data['normalized'], 
                    marker=markers.get(tree_type, 'o'),
                    color=colors.get(tree_type, 'blue'),
                    label=labels.get(tree_type, tree_type),
                    linewidth=2, markersize=8)
    
    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Time / n^3 (us)')
    ax.set_title('CRBA: Time normalized by n^3 (O(n^3) analysis)')
    ax.legend()
    ax.set_xscale('log', base=2)
    ax.grid(True, alpha=0.3)
    
    # Log-log plot for ABA
    ax = axes[1, 0]
    for tree_type in tree_types:
        data = df[df['tree_type'] == tree_type]
        ax.plot(data['num_bodies'], data['aba_time_us'], 
                marker=markers.get(tree_type, 'o'),
                color=colors.get(tree_type, 'blue'),
                label=labels.get(tree_type, tree_type),
                linewidth=2, markersize=8)
    
    # Add reference lines
    n_max = df['num_bodies'].max()
    n = np.array([4, n_max])
    # Scale reference lines to fit the data
    aba_min = df['aba_time_us'].min()
    ax.plot(n, n * (aba_min / 4), 'k--', alpha=0.5, label='O(n)')
    ax.plot(n, n**2 * (aba_min / 16), 'k:', alpha=0.5, label='O(n^2)')
    ax.set_xscale('log', base=2)
    ax.set_yscale('log')
    
    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Execution Time (us)')
    ax.set_title('ABA: Log-Log Plot with Reference Lines')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Log-log plot for CRBA
    ax = axes[1, 1]
    for tree_type in tree_types:
        data = df_crba[df_crba['tree_type'] == tree_type]
        if len(data) > 0:
            ax.plot(data['num_bodies'], data['crba_time_us'], 
                    marker=markers.get(tree_type, 'o'),
                    color=colors.get(tree_type, 'blue'),
                    label=labels.get(tree_type, tree_type),
                    linewidth=2, markersize=8)
    
    # Add reference lines
    if len(df_crba) > 0:
        crba_min = df_crba['crba_time_us'].min()
        n_crba_max = df_crba['num_bodies'].max()
        n = np.array([4, n_crba_max])
        ax.plot(n, n**2 * (crba_min / 16), 'k--', alpha=0.5, label='O(n^2)')
        ax.plot(n, n**3 * (crba_min / 64), 'k:', alpha=0.5, label='O(n^3)')
    ax.set_xscale('log', base=2)
    ax.set_yscale('log')
    
    ax.set_xlabel('Number of Bodies')
    ax.set_ylabel('Execution Time (us)')
    ax.set_title('CRBA: Log-Log Plot with Reference Lines')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, 'complexity_analysis.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_tree_structure_comparison(df: pd.DataFrame, output_dir: str):
    """
    Compare how tree structure affects algorithm performance.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # ABA comparison by depth
    ax = axes[0]
    
    # Plot time vs tree depth
    for tree_type in ['chain', 'binary_tree', 'star']:
        data = df[df['tree_type'] == tree_type]
        if len(data) > 0:
            ax.scatter(data['tree_depth'], data['aba_time_us'], 
                       s=np.clip(data['num_bodies'] * 0.5, 20, 200),  # Size by number of bodies
                       alpha=0.6,
                       label=tree_type.replace('_', ' ').title())
    
    ax.set_xlabel('Tree Depth')
    ax.set_ylabel('ABA Execution Time (us)')
    ax.set_title('ABA Time vs Tree Depth\n(point size ~ number of bodies)')
    ax.legend()
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    
    # CRBA comparison by depth
    ax = axes[1]
    df_crba = df[df['crba_time_us'] >= 0]
    
    for tree_type in ['chain', 'binary_tree', 'star']:
        data = df_crba[df_crba['tree_type'] == tree_type]
        if len(data) > 0:
            ax.scatter(data['tree_depth'], data['crba_time_us'], 
                       s=np.clip(data['num_bodies'] * 0.5, 20, 200),
                       alpha=0.6,
                       label=tree_type.replace('_', ' ').title())
    
    ax.set_xlabel('Tree Depth')
    ax.set_ylabel('CRBA Execution Time (us)')
    ax.set_title('CRBA Time vs Tree Depth\n(point size ~ number of bodies)')
    ax.legend()
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.join(output_dir, 'tree_structure_comparison.pdf')
    plt.savefig(output_path, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def create_summary_table(df: pd.DataFrame, output_dir: str):
    """
    Create a summary table and save it as CSV.
    """
    # Create summary for ABA (all data)
    aba_summary = df.groupby('tree_type').agg({
        'num_bodies': ['min', 'max'],
        'aba_time_us': ['min', 'max', 'mean']
    }).round(2)
    
    # Create summary for CRBA (only valid data)
    df_crba = df[df['crba_time_us'] >= 0]
    crba_summary = df_crba.groupby('tree_type').agg({
        'num_bodies': ['min', 'max'],
        'crba_time_us': ['min', 'max', 'mean']
    }).round(2)
    
    # Save to CSV
    aba_summary_path = os.path.join(output_dir, 'aba_summary_statistics.csv')
    aba_summary.to_csv(aba_summary_path)
    print(f"Saved: {aba_summary_path}")
    
    crba_summary_path = os.path.join(output_dir, 'crba_summary_statistics.csv')
    crba_summary.to_csv(crba_summary_path)
    print(f"Saved: {crba_summary_path}")
    
    # Print to console
    print("\n" + "=" * 80)
    print("ABA BENCHMARK SUMMARY")
    print("=" * 80)
    print(aba_summary.to_string())
    print("\n" + "=" * 80)
    print("CRBA BENCHMARK SUMMARY (only where computed)")
    print("=" * 80)
    print(crba_summary.to_string())
    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(
        description='Plot Forward Dynamics Algorithm Benchmark Results')
    parser.add_argument('--input', '-i', type=str, default='output/benchmark_results.csv',
                        help='Path to input CSV file')
    parser.add_argument('--output', '-o', type=str, default='output',
                        help='Output directory for plots')
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    # Load data
    df = load_benchmark_data(args.input)
    
    if len(df) == 0:
        print("Error: No valid benchmark data to plot")
        sys.exit(1)
    
    # Generate all plots
    print("\nGenerating plots...")

    plot_time_vs_bodies(df, args.output)
    plot_algorithm_comparison(df, args.output)
    plot_speedup_ratio(df, args.output)
    plot_complexity_analysis(df, args.output)
    plot_tree_structure_comparison(df, args.output)
    create_summary_table(df, args.output)
    
    print(f"\nAll plots saved to: {args.output}/")
    print("\nGenerated files:")
    print("  - time_vs_bodies.pdf: Execution time vs number of bodies")
    print("  - algorithm_comparison.pdf: Side-by-side ABA vs CRBA comparison")
    print("  - speedup_ratio.pdf: CRBA/ABA speedup ratio")
    print("  - complexity_analysis.pdf: Complexity analysis with O(n) reference")
    print("  - tree_structure_comparison.pdf: Impact of tree structure on performance")
    print("  - aba_summary_statistics.csv: ABA summary statistics")
    print("  - crba_summary_statistics.csv: CRBA summary statistics")


if __name__ == '__main__':
    main()
