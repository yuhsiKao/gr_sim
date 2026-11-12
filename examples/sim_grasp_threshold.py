"""Make the gripper's grasp detection work in the simulators.

The SDK detects a grasp when the gripper current exceeds WRIST_CLOSEPCT * WRIST_GRASPTAU (0.7 * 14).
The simulators report 8 while closing, so it never triggers. This lowers WRIST_GRASPTAU to 10
(threshold 7) and clears the WG override. The grip is weaker (10 instead of 14 Nm) and the values
are not saved: run it after every simulator start.

    scripts/run_sim.sh arm 1                                               # terminal A
    source scripts/env.sh && $GSIM_PY -B examples/sim_grasp_threshold.py   # terminal B
"""
import argparse, os, sys, time

# grpy (MBLink) ships with the SDK; never leave __pycache__ there.
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.environ.get("CONTROLS_INSTALL_DIR", ""), "share", "gr", "mblink", "examples"))
from grpy import mb80v2

SIM_GRASP_TAU = 10
PARAMS = ("WRIST_GRASPTAU", "WRIST_CLOSEPCT", "WG")

def waitForTelemetry(mb, timeout=10.0):
    t0 = time.time()
    while "y" not in mb.getLatest():
        if time.time() - t0 > timeout:
            raise RuntimeError("No telemetry: is a simulator running with the arm (scripts/run_sim.sh arm 1)?")
        mb.sendPing()  # the simulator starts streaming once it hears from us
        time.sleep(0.1)

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Lower WRIST_GRASPTAU so the simulators detect gripper grasps")
    ap.add_argument("--tau", type=float, default=SIM_GRASP_TAU, help=f"WRIST_GRASPTAU to set (default {SIM_GRASP_TAU}; SDK default 14)")
    args = ap.parse_args()

    mb = mb80v2.MB80v2(sim=True, verbose=False, log=False, use_blocking_get=False)
    try:
        waitForTelemetry(mb)
        before = {p: mb.readParam(p, True) for p in PARAMS}
        mb.setRetry("WRIST_GRASPTAU", args.tau)
        mb.setRetry("WG", 0)
        tau, pct, wg = (mb.readParam(p, True) for p in PARAMS)
        print(f"WRIST_GRASPTAU {before['WRIST_GRASPTAU']:g} -> {tau:g}, WG {before['WG']:g} -> {wg:g}")
        print(f"grasp threshold WRIST_CLOSEPCT * WRIST_GRASPTAU = {pct:g} * {tau:g} = {pct * tau:g} (the sim reports 8 while closing)")
    finally:
        mb.rxstop(save=False)
