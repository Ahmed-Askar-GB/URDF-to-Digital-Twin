"""Track 3: Python/CasADi + MuJoCo Real-Time Nonlinear Control and Verification.

This track prioritizes open-source scalability, zero-licensing deployment,
and real-time closed-loop nonlinear control synthesis (> 1 kHz).
"""

from __future__ import annotations
import os
import sys
import time
import numpy as np

# Add workspace path to access urdf2dt package
this_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(os.path.dirname(this_dir))
sys.path.insert(0, os.path.join(root_dir, "URDF2DT-Py Full pipeline"))

from urdf2dt import (
    urdf_parse,
    urdf_build_chain,
    urdf_extract_dh,
    urdf_extract_inertia,
    dynamics_casadi,
    MuJoCoTwin,
    ComputedTorqueController,
    verify_invariants,
    verify_kinematics,
    verify_dynamics_sequential,
)
from urdf2dt.utils import traj_poly5, detect_joint_signs


def run_track3(urdf_path: str | None = None, verbose: bool = True):
    """Run Track 3 complete pipeline on the specified URDF."""
    if urdf_path is None:
        urdf_path = os.path.join(root_dir, "universalUR5.urdf")

    print("\n" + "=" * 76)
    print("  TRACK 3: PYTHON / CASADI + MUJOCO DIGITAL TWIN & REAL-TIME CONTROL")
    print("=" * 76)
    print(f"  Robot URDF : {urdf_path}")

    # 1. Parse URDF and extract DH chain
    t0 = time.time()
    robot = urdf_parse(urdf_path)
    chain = urdf_build_chain(robot, verbose=False)
    dh = urdf_extract_dh(chain, verbose=False)
    inertia = urdf_extract_inertia(robot, chain, dh, verbose=False)
    t_parse = time.time() - t0
    print(f"  [OK] Parsed {chain.n}-DOF chain and extracted DH parameters in {t_parse:.3f} s")

    # 2. Compile CasADi Analytical Dynamics
    t1 = time.time()
    model = dynamics_casadi(dh, inertia, chain, verbose=False)
    t_model = time.time() - t1
    print(f"  [OK] Algorithmic Differentiation (AD) model compiled in {t_model:.3f} s")

    # 3. Initialize Headless MuJoCo Digital Twin
    t2 = time.time()
    twin = MuJoCoTwin(urdf_path, chain=chain, dh=dh, dt=0.001, verbose=False)
    t_twin = time.time() - t2
    print(f"  [OK] MuJoCo Digital Twin initialized in {t_twin:.3f} s")

    # 4. Invariant and Kinematic Verification
    inv_res = verify_invariants(robot, dh, inertia, model)
    print(f"  [VERIFY] M(q) Positive-Definite: {inv_res.get('mass_matrix_spd', True)} | SO(3) Invariant: {inv_res.get('so3_valid', True)}")

    kin_res = verify_kinematics(model, twin, chain, num_samples=50)
    print(f"  [VERIFY] Kinematic Pos Error Max: {kin_res.get('max_pos_err', 0.0):.2e} m | Quat Error Max: {kin_res.get('max_quat_err', 0.0):.2e}")

    # 5. Inverse Dynamics Parity with mj_inverse
    q_test = np.array([0.2, -0.3, 0.4, -0.2, 0.3, -0.1][:chain.n])
    dq_test = np.array([0.1, -0.1, 0.2, 0.0, -0.1, 0.05][:chain.n])
    ddq_test = np.array([0.5, -0.5, 1.0, -0.2, 0.4, -0.3][:chain.n])

    joint_signs = detect_joint_signs(chain, model.fk_link_funs)
    tau_ana = joint_signs * np.array(model.tau_fun(joint_signs * q_test, joint_signs * dq_test, joint_signs * ddq_test)).flatten()
    tau_mujoco = twin.inverse_dynamics(q_test, dq_test, ddq_test)
    id_diff = np.max(np.abs(tau_ana - tau_mujoco))
    print(f"  [VERIFY] Inverse Dynamics Parity vs MuJoCo (mj_inverse): {id_diff:.2e} N*m")

    # 6. Sequential Dynamic Isolation
    print("\n  >>> Running Sequential Link-by-Link Dynamic Isolation Protocol <<<")
    seq_res = verify_dynamics_sequential(chain, dh, inertia, twin, t_dur=1.5, dt=0.005, verbose=False)
    for st in seq_res:
        print(f"    Stage {st.stage}: Joint [{st.joint_name}] max torque diff = {st.max_err:.4f} N*m (Pass={st.passed})")

    # 7. Closed-Loop Computed Torque Control Simulation
    print("\n  >>> Running Closed-Loop Computed Torque Control Simulation <<<")
    controller = ComputedTorqueController(model, kp=300.0, kd=40.0, joint_signs=joint_signs)
    q0 = np.zeros(chain.n)
    qf = np.array([0.5, -0.6, 0.5, -0.3, 0.4, -0.2][:chain.n])
    traj_time, q_ref, dq_ref, ddq_ref = traj_poly5(q0, qf, t_dur=2.0, dt=0.001)

    sim_out = twin.simulate_trajectory(traj_time, q_ref, dq_ref, ddq_ref, controller=controller)
    pos_err = np.linalg.norm(sim_out["q_sim"] - q_ref, axis=1)
    print(f"  [CONTROL] CTC Max Joint Error: {np.max(pos_err):.4e} rad | RMS Error: {np.sqrt(np.mean(pos_err**2)):.4e} rad")

    # Single-step latency benchmark
    t_bench = time.time()
    for _ in range(1000):
        _ = model.M_fun(q_test)
        _ = model.C_fun(q_test, dq_test)
        _ = model.G_fun(q_test)
        _ = model.tau_fun(q_test, dq_test, ddq_test)
    latency_us = ((time.time() - t_bench) / 1000.0) * 1e6
    print(f"  [BENCHMARK] AD Single-Step Evaluation Latency: {latency_us:.2f} microseconds")
    print("=" * 76 + "\n")

    return {
        "chain_n": chain.n,
        "t_model": t_model,
        "max_pos_err": kin_res.get('max_pos_err', 0.0),
        "id_diff": id_diff,
        "ctc_max_err": float(np.max(pos_err)),
        "latency_us": latency_us
    }


if __name__ == "__main__":
    urdf = sys.argv[1] if len(sys.argv) > 1 else None
    run_track3(urdf)
