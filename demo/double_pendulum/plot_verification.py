#!/usr/bin/env python3
"""
Double Pendulum Dynamics Verification - Plotting Script

This script reads CSV data from C++ simulation and generates separate PDF plots
for inclusion in a LaTeX verification report.

Requirements:
    pip install numpy matplotlib pandas

Usage:
    python plot_verification.py [output_dir]

Output files:
    - joint_positions.pdf
    - joint_velocities.pdf
    - joint_accelerations.pdf
    - total_energy.pdf
    - phase_space.pdf
    - position_errors.pdf
    - velocity_errors.pdf
    - acceleration_errors.pdf
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Configure matplotlib for publication quality
plt.rcParams.update({
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'lines.linewidth': 1.0,
})


def load_simulation_data(csv_path):
    """Load RBD simulation data from C++ output CSV file."""
    print(f"Loading RBD simulation data from {csv_path}...")
    df = pd.read_csv(csv_path)
    print(f"  Loaded {len(df)} data points")
    print(f"  Time range: {df['time'].min():.3f} to {df['time'].max():.3f} s")
    return df


def load_reference_data(csv_path):
    """Load reference (Lagrangian) simulation data from C++ output CSV file."""
    print(f"Loading reference data from {csv_path}...")
    df = pd.read_csv(csv_path)
    print(f"  Loaded {len(df)} data points")
    return df


def plot_joint_positions(rbd_df, ref_df, output_path):
    """
    Plot joint positions (q1, q2) vs. time for ABA/CRBA and reference.
    Overlays curves from all methods to verify trajectory agreement.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Joint 1 position
    ax = axes[0]
    ax.plot(rbd_df['time'], rbd_df['q1'], 'b-', label='RBD (ABA/CRBA)', linewidth=1.2)
    ax.plot(ref_df['time'], ref_df['q1'], 'r--', label='Reference (Lagrangian)',
            linewidth=1.0, alpha=0.8)
    ax.set_ylabel(r'$q_1$ (rad)')
    ax.set_title('Joint 1 Position')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    # Joint 2 position
    ax = axes[1]
    ax.plot(rbd_df['time'], rbd_df['q2'], 'b-', label='RBD (ABA/CRBA)', linewidth=1.2)
    ax.plot(ref_df['time'], ref_df['q2'], 'r--', label='Reference (Lagrangian)',
            linewidth=1.0, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$q_2$ (rad)')
    ax.set_title('Joint 2 Position')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    fig.suptitle('Joint Positions vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_joint_velocities(rbd_df, ref_df, output_path):
    """
    Plot joint velocities (qdot1, qdot2) vs. time for ABA/CRBA and reference.
    Checks velocity evolution and forward dynamics computation.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Joint 1 velocity
    ax = axes[0]
    ax.plot(rbd_df['time'], rbd_df['qdot1'], 'b-', label='RBD (ABA/CRBA)', linewidth=1.2)
    ax.plot(ref_df['time'], ref_df['qdot1'], 'r--', label='Reference (Lagrangian)',
            linewidth=1.0, alpha=0.8)
    ax.set_ylabel(r'$\dot{q}_1$ (rad/s)')
    ax.set_title('Joint 1 Velocity')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    # Joint 2 velocity
    ax = axes[1]
    ax.plot(rbd_df['time'], rbd_df['qdot2'], 'b-', label='RBD (ABA/CRBA)', linewidth=1.2)
    ax.plot(ref_df['time'], ref_df['qdot2'], 'r--', label='Reference (Lagrangian)',
            linewidth=1.0, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\dot{q}_2$ (rad/s)')
    ax.set_title('Joint 2 Velocity')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    fig.suptitle('Joint Velocities vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_joint_accelerations(rbd_df, ref_df, output_path):
    """
    Plot joint accelerations from ABA, CRBA, and reference vs. time.
    Includes subplots for ABA-CRBA differences.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))

    # Joint 1 accelerations - main comparison
    ax = axes[0, 0]
    ax.plot(rbd_df['time'], rbd_df['qddot1_aba'], 'b-', label='ABA', linewidth=1.0)
    ax.plot(rbd_df['time'], rbd_df['qddot1_crba'], 'g--', label='CRBA', linewidth=1.0, alpha=0.8)
    ax.plot(ref_df['time'], ref_df['qddot1'], 'r:', label='Reference', linewidth=1.2, alpha=0.8)
    ax.set_ylabel(r'$\ddot{q}_1$ (rad/s²)')
    ax.set_title('Joint 1 Acceleration')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    # Joint 2 accelerations - main comparison
    ax = axes[0, 1]
    ax.plot(rbd_df['time'], rbd_df['qddot2_aba'], 'b-', label='ABA', linewidth=1.0)
    ax.plot(rbd_df['time'], rbd_df['qddot2_crba'], 'g--', label='CRBA', linewidth=1.0, alpha=0.8)
    ax.plot(ref_df['time'], ref_df['qddot2'], 'r:', label='Reference', linewidth=1.2, alpha=0.8)
    ax.set_ylabel(r'$\ddot{q}_2$ (rad/s²)')
    ax.set_title('Joint 2 Acceleration')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)

    # Joint 1 ABA-CRBA difference
    ax = axes[1, 0]
    ax.plot(rbd_df['time'], rbd_df['qddot1_diff'], 'purple', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\ddot{q}_1^{\mathrm{ABA}} - \ddot{q}_1^{\mathrm{CRBA}}$ (rad/s²)')
    ax.set_title('Joint 1: ABA − CRBA Difference')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    # Joint 2 ABA-CRBA difference
    ax = axes[1, 1]
    ax.plot(rbd_df['time'], rbd_df['qddot2_diff'], 'purple', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\ddot{q}_2^{\mathrm{ABA}} - \ddot{q}_2^{\mathrm{CRBA}}$ (rad/s²)')
    ax.set_title('Joint 2: ABA − CRBA Difference')
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    fig.suptitle('Joint Accelerations vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_total_energy(rbd_df, ref_df, output_path):
    """
    Plot energy conservation error (drift) vs. time for ABA/CRBA and reference.
    For an energy-conserving system, the drift should be zero.
    """
    print(f"Generating: {output_path}")

    fig, ax = plt.subplots(1, 1, figsize=(8, 4))

    rbd_e0 = rbd_df['total_energy'].iloc[0]
    ref_e0 = ref_df['total_energy'].iloc[0]

    # Energy drift (conservation error)
    ax.plot(rbd_df['time'], rbd_df['total_energy'] - rbd_e0, 'b-',
            label=f'RBD (max: {(rbd_df["total_energy"] - rbd_e0).abs().max():.2e} J)',
            linewidth=1.0)
    ax.plot(ref_df['time'], ref_df['total_energy'] - ref_e0, 'r--',
            label=f'Reference (max: {(ref_df["total_energy"] - ref_e0).abs().max():.2e} J)',
            linewidth=1.0, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Energy Drift (J)')
    ax.set_title('Energy Conservation Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    fig.suptitle('Energy Conservation vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_phase_space(rbd_df, ref_df, output_path):
    """
    Plot phase space trajectories (q vs qdot) for both joints.
    Compares RBD and reference to ensure consistency.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))

    # Joint 1 phase portrait
    ax = axes[0, 0]
    ax.plot(rbd_df['q1'], rbd_df['qdot1'], 'b-', label='RBD (ABA/CRBA)',
            linewidth=0.6, alpha=0.9)
    ax.plot(ref_df['q1'], ref_df['qdot1'], 'r--', label='Reference (Lagrangian)',
            linewidth=0.6, alpha=0.7)
    ax.set_xlabel(r'$q_1$ (rad)')
    ax.set_ylabel(r'$\dot{q}_1$ (rad/s)')
    ax.set_title('Joint 1 Phase Portrait')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.set_aspect('auto')

    # Joint 2 phase portrait
    ax = axes[0, 1]
    ax.plot(rbd_df['q2'], rbd_df['qdot2'], 'b-', label='RBD (ABA/CRBA)',
            linewidth=0.6, alpha=0.9)
    ax.plot(ref_df['q2'], ref_df['qdot2'], 'r--', label='Reference (Lagrangian)',
            linewidth=0.6, alpha=0.7)
    ax.set_xlabel(r'$q_2$ (rad)')
    ax.set_ylabel(r'$\dot{q}_2$ (rad/s)')
    ax.set_title('Joint 2 Phase Portrait')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.set_aspect('auto')

    # Configuration space trajectory (q1 vs q2)
    ax = axes[1, 0]
    ax.plot(rbd_df['q1'], rbd_df['q2'], 'b-', label='RBD (ABA/CRBA)',
            linewidth=0.6, alpha=0.9)
    ax.plot(ref_df['q1'], ref_df['q2'], 'r--', label='Reference (Lagrangian)',
            linewidth=0.6, alpha=0.7)
    ax.set_xlabel(r'$q_1$ (rad)')
    ax.set_ylabel(r'$q_2$ (rad)')
    ax.set_title('Configuration Space Trajectory')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.set_aspect('auto')

    # Velocity space trajectory (qdot1 vs qdot2)
    ax = axes[1, 1]
    ax.plot(rbd_df['qdot1'], rbd_df['qdot2'], 'b-', label='RBD (ABA/CRBA)',
            linewidth=0.6, alpha=0.9)
    ax.plot(ref_df['qdot1'], ref_df['qdot2'], 'r--', label='Reference (Lagrangian)',
            linewidth=0.6, alpha=0.7)
    ax.set_xlabel(r'$\dot{q}_1$ (rad/s)')
    ax.set_ylabel(r'$\dot{q}_2$ (rad/s)')
    ax.set_title('Velocity Space Trajectory')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.set_aspect('auto')

    fig.suptitle('Phase Space Trajectories', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_position_errors(rbd_df, ref_df, output_path):
    """
    Plot position errors (q1, q2) between all three method pairs vs. time.
    Three lines per joint: ABA-CRBA, ABA-Reference, CRBA-Reference.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Joint 1 position errors
    ax = axes[0]
    # ABA and CRBA use same positions (integrated from same ODE), so ABA-CRBA = 0
    # We compare RBD (which uses ABA) vs Reference
    err_rbd_ref = rbd_df['q1'] - ref_df['q1']
    ax.plot(rbd_df['time'], err_rbd_ref, 'b-', label='RBD − Reference', linewidth=1.0)
    ax.set_ylabel(r'$q_1$ Error (rad)')
    ax.set_title('Joint 1 Position Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    # Joint 2 position errors
    ax = axes[1]
    err_rbd_ref = rbd_df['q2'] - ref_df['q2']
    ax.plot(rbd_df['time'], err_rbd_ref, 'b-', label='RBD − Reference', linewidth=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$q_2$ Error (rad)')
    ax.set_title('Joint 2 Position Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    fig.suptitle('Position Errors vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_velocity_errors(rbd_df, ref_df, output_path):
    """
    Plot velocity errors (qdot1, qdot2) between all three method pairs vs. time.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Joint 1 velocity errors
    ax = axes[0]
    err_rbd_ref = rbd_df['qdot1'] - ref_df['qdot1']
    ax.plot(rbd_df['time'], err_rbd_ref, 'b-', label='RBD − Reference', linewidth=1.0)
    ax.set_ylabel(r'$\dot{q}_1$ Error (rad/s)')
    ax.set_title('Joint 1 Velocity Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    # Joint 2 velocity errors
    ax = axes[1]
    err_rbd_ref = rbd_df['qdot2'] - ref_df['qdot2']
    ax.plot(rbd_df['time'], err_rbd_ref, 'b-', label='RBD − Reference', linewidth=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\dot{q}_2$ Error (rad/s)')
    ax.set_title('Joint 2 Velocity Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    fig.suptitle('Velocity Errors vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def plot_acceleration_errors(rbd_df, ref_df, output_path):
    """
    Plot acceleration errors (qddot1, qddot2) between all three method pairs vs. time.
    Three lines per joint: ABA-CRBA, ABA-Reference, CRBA-Reference.
    """
    print(f"Generating: {output_path}")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Joint 1 acceleration errors
    ax = axes[0]
    err_aba_crba = rbd_df['qddot1_aba'] - rbd_df['qddot1_crba']
    err_aba_ref = rbd_df['qddot1_aba'] - ref_df['qddot1']
    err_crba_ref = rbd_df['qddot1_crba'] - ref_df['qddot1']
    ax.plot(rbd_df['time'], err_aba_crba, 'g-', label='ABA − CRBA', linewidth=1.0)
    ax.plot(rbd_df['time'], err_aba_ref, 'b--', label='ABA − Reference', linewidth=1.0)
    ax.plot(rbd_df['time'], err_crba_ref, 'r:', label='CRBA − Reference', linewidth=1.2)
    ax.set_ylabel(r'$\ddot{q}_1$ Error (rad/s²)')
    ax.set_title('Joint 1 Acceleration Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    # Joint 2 acceleration errors
    ax = axes[1]
    err_aba_crba = rbd_df['qddot2_aba'] - rbd_df['qddot2_crba']
    err_aba_ref = rbd_df['qddot2_aba'] - ref_df['qddot2']
    err_crba_ref = rbd_df['qddot2_crba'] - ref_df['qddot2']
    ax.plot(rbd_df['time'], err_aba_crba, 'g-', label='ABA − CRBA', linewidth=1.0)
    ax.plot(rbd_df['time'], err_aba_ref, 'b--', label='ABA − Reference', linewidth=1.0)
    ax.plot(rbd_df['time'], err_crba_ref, 'r:', label='CRBA − Reference', linewidth=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\ddot{q}_2$ Error (rad/s²)')
    ax.set_title('Joint 2 Acceleration Error')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(-3, 3))

    fig.suptitle('Acceleration Errors vs. Time', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    fig.savefig(output_path, format='pdf')
    plt.close(fig)


def compute_and_print_errors(rbd_df, ref_df):
    """Compute and print error summary between RBD and reference simulations."""
    print("\nError Summary (RBD vs Reference):")
    print("-" * 65)
    print(f"  {'Variable':20s}  {'Max Error':>15s}  {'RMS Error':>15s}")
    print("-" * 65)

    comparisons = [
        ('q1', rbd_df['q1'], ref_df['q1'], 'rad'),
        ('q2', rbd_df['q2'], ref_df['q2'], 'rad'),
        ('qdot1', rbd_df['qdot1'], ref_df['qdot1'], 'rad/s'),
        ('qdot2', rbd_df['qdot2'], ref_df['qdot2'], 'rad/s'),
        ('qddot1 (ABA)', rbd_df['qddot1_aba'], ref_df['qddot1'], 'rad/s²'),
        ('qddot2 (ABA)', rbd_df['qddot2_aba'], ref_df['qddot2'], 'rad/s²'),
        ('total_energy', rbd_df['total_energy'], ref_df['total_energy'], 'J'),
    ]

    for name, rbd_col, ref_col, unit in comparisons:
        error = rbd_col - ref_col
        max_err = error.abs().max()
        rms_err = np.sqrt((error ** 2).mean())
        print(f"  {name:20s}  {max_err:12.3e} {unit:>3s}  {rms_err:12.3e} {unit:>3s}")

    print("-" * 65)

    # ABA vs CRBA difference
    if 'qddot1_diff' in rbd_df.columns:
        max_diff1 = rbd_df['qddot1_diff'].abs().max()
        max_diff2 = rbd_df['qddot2_diff'].abs().max()
        print(f"\nABA vs CRBA Difference:")
        print(f"  qddot1: max = {max_diff1:.3e} rad/s²")
        print(f"  qddot2: max = {max_diff2:.3e} rad/s²")


def main():
    """Main entry point."""
    # Determine paths
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Check for command line argument for output directory
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        # Default to build output directory
        output_dir = os.path.join(script_dir, '../../build/output')

    output_dir = os.path.abspath(output_dir)

    rbd_csv_path = os.path.join(output_dir, 'simulation_data.csv')
    ref_csv_path = os.path.join(output_dir, 'reference_data.csv')

    print("=" * 65)
    print("Double Pendulum Dynamics Verification - Plot Generation")
    print("=" * 65)
    print()
    print(f"Output directory: {output_dir}")
    print()

    # Check required files exist
    if not os.path.exists(rbd_csv_path):
        print(f"ERROR: RBD simulation data not found at {rbd_csv_path}")
        print("Please run the C++ demo first:")
        print("  cd build && ./demo/double_pendulum/demo_double_pendulum")
        return 1

    if not os.path.exists(ref_csv_path):
        print(f"ERROR: Reference data not found at {ref_csv_path}")
        print("Please run the C++ demo first:")
        print("  cd build && ./demo/double_pendulum/demo_double_pendulum")
        return 1

    # Load data
    rbd_df = load_simulation_data(rbd_csv_path)
    ref_df = load_reference_data(ref_csv_path)
    print()

    # Verify same number of data points
    if len(rbd_df) != len(ref_df):
        print(f"WARNING: Different number of data points ({len(rbd_df)} vs {len(ref_df)})")
        print("  Truncating to minimum length")
        n = min(len(rbd_df), len(ref_df))
        rbd_df = rbd_df.iloc[:n].reset_index(drop=True)
        ref_df = ref_df.iloc[:n].reset_index(drop=True)

    # Compute and print errors
    compute_and_print_errors(rbd_df, ref_df)
    print()

    # Generate separate PDF plots
    print("Generating PDF plots for LaTeX report...")
    print()

    plot_joint_positions(rbd_df, ref_df,
                         os.path.join(output_dir, 'joint_positions.pdf'))

    plot_joint_velocities(rbd_df, ref_df,
                          os.path.join(output_dir, 'joint_velocities.pdf'))

    plot_joint_accelerations(rbd_df, ref_df,
                             os.path.join(output_dir, 'joint_accelerations.pdf'))

    plot_total_energy(rbd_df, ref_df,
                      os.path.join(output_dir, 'total_energy.pdf'))

    plot_phase_space(rbd_df, ref_df,
                     os.path.join(output_dir, 'phase_space.pdf'))

    plot_position_errors(rbd_df, ref_df,
                         os.path.join(output_dir, 'position_errors.pdf'))

    plot_velocity_errors(rbd_df, ref_df,
                         os.path.join(output_dir, 'velocity_errors.pdf'))

    plot_acceleration_errors(rbd_df, ref_df,
                             os.path.join(output_dir, 'acceleration_errors.pdf'))

    print()
    print("=" * 65)
    print("Plot generation complete!")
    print("=" * 65)
    print()
    print("Generated PDF files:")
    print(f"  - {os.path.join(output_dir, 'joint_positions.pdf')}")
    print(f"  - {os.path.join(output_dir, 'joint_velocities.pdf')}")
    print(f"  - {os.path.join(output_dir, 'joint_accelerations.pdf')}")
    print(f"  - {os.path.join(output_dir, 'total_energy.pdf')}")
    print(f"  - {os.path.join(output_dir, 'phase_space.pdf')}")
    print(f"  - {os.path.join(output_dir, 'position_errors.pdf')}")
    print(f"  - {os.path.join(output_dir, 'velocity_errors.pdf')}")
    print(f"  - {os.path.join(output_dir, 'acceleration_errors.pdf')}")
    print()
    print("Include in LaTeX with:")
    print(r"  \includegraphics[width=\textwidth]{joint_positions.pdf}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
