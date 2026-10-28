"""Master Reproduction Harness (Rule 3 & Deliverable 1).

Executes all experimental benchmarks E1 to E6, writes raw data with hardware/git metadata
to results/*.json, regenerates publication vector figures, updates LaTeX table fragments,
and compiles CLAIMS_AUDIT.md.
"""

from __future__ import annotations
import os
import sys
import time
import subprocess

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
PIPELINE_ROOT = os.path.join(WORKSPACE_ROOT, "Bridging Black-Box Physics Simulators and Analytical Mechanics", "URDF2DT-Py Full pipeline")
BENCH_DIR = os.path.join(PIPELINE_ROOT, "benchmarks")


def run_command(cmd, desc):
    print("\n" + "=" * 80)
    print(f"  RUNNING: {desc}")
    print("=" * 80)
    t0 = time.perf_counter()
    res = subprocess.run(cmd, shell=True)
    dt = time.perf_counter() - t0
    if res.returncode != 0:
        print(f"[ERROR] {desc} failed with exit code {res.returncode}")
        sys.exit(res.returncode)
    print(f"[SUCCESS] {desc} finished in {dt:.2f} s")


def main():
    print("=" * 80)
    print("     STARTING FULL WORKFLOW REPRODUCTION PIPELINE (E1 to E6)       ")
    print("=" * 80)
    t_start = time.perf_counter()

    # 1. Benchmark E1: Derivative Capability (H1, H2)
    e1_script = os.path.join(BENCH_DIR, "bench_e1_derivatives.py")
    run_command(f'"{sys.executable}" "{e1_script}"', "Benchmark E1: Derivative Capability (H1, H2)")

    # 2. Benchmark E2: Offline Trajectory Optimization (H3)
    e2_script = os.path.join(BENCH_DIR, "bench_e2_trajectory_opt.py")
    run_command(f'"{sys.executable}" "{e2_script}"', "Benchmark E2: Offline Trajectory Optimization (H3)")

    # 3. Benchmark E3: Mismatch Control (H4)
    e3_script = os.path.join(BENCH_DIR, "bench_e3_mismatch_control.py")
    run_command(f'"{sys.executable}" "{e3_script}"', "Benchmark E3: Mismatch Control on Plant (H4)")

    # 4. Benchmark E4: Real-Time NMPC (H5)
    e4_script = os.path.join(BENCH_DIR, "bench_e4_realtime_nmpc.py")
    run_command(f'"{sys.executable}" "{e4_script}"', "Benchmark E4: Real-Time Receding-Horizon NMPC (H5)")

    # 5. Benchmark E5: Workflow Turnaround (H6)
    e5_script = os.path.join(BENCH_DIR, "bench_e5_turnaround.py")
    run_command(f'"{sys.executable}" "{e5_script}"', "Benchmark E5: End-to-End Workflow Turnaround (H6)")

    # 6. Benchmark E6: Core Scalability & Inconsistencies
    e6_script = os.path.join(BENCH_DIR, "bench_e6_scalability.py")
    run_command(f'"{sys.executable}" "{e6_script}"', "Benchmark E6: Core Scalability & Inconsistencies")

    # 7. Generate Publication Figures & Table Fragments
    plot_script = os.path.join(PIPELINE_ROOT, "generate_paper_artifacts.py")
    if os.path.exists(plot_script):
        run_command(f'"{sys.executable}" "{plot_script}"', "Generate Publication Figures & Tables")

    # 8. Generate Claims Audit
    audit_script = os.path.join(WORKSPACE_ROOT, "generate_claims_audit.py")
    if os.path.exists(audit_script):
        run_command(f'"{sys.executable}" "{audit_script}"', "Generate CLAIMS_AUDIT.md")

    total_time = time.perf_counter() - t_start
    print("\n" + "=" * 80)
    print(f"  ALL BENCHMARKS COMPLETED SUCCESSFULLY IN {total_time:.2f} s ({total_time/60.0:.2f} min)")
    print("=" * 80)


if __name__ == "__main__":
    main()
