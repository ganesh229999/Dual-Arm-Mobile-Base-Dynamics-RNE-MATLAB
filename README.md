## Dual-Arm-Mobile-Base-Dynamics-RNE-MATLAB

OVERVIEW
This repository implements Recursive Newton-Euler (RNE) dynamics for a dual-arm robot with both fixed-base and mobile-base variants. It includes trajectory generation, torque computation, base-coupling (Khalil–Dombre style), object wrench models, visualization, and utilities for exporting results. Top-level scripts (those with "CALL" in the name) are entry points you run directly.

QUICK START (3 steps)

1.Place your input Excel file in the same folder as the CALL script (or provide full path).

2.Open the appropriate CALL script and set these variables at the top if needed:

I.filename (or full path),

II.baseState.T_w0, baseState.v0, baseState.w0,

III.objParams (mass, inertia).

3.Run one of the CALL scripts from MATLAB Command Window.

If you want a one-shot run (mobile base):
callLiftingTrajectory1610 -: generate trajectory Excel (IK trajectory generation)
rnecombinedmobilebasecall -:  runs rnemobilebasec and saves outputs

If you want fixed-base experiments:
callLiftingTrajectory1610 -:  generate trajectory Excel (IK trajectory generation)
callrneFixedBase1610 -: run fixed-base RNE
newcoordinatedTasks : for bottleopening taks (IK trajectory generation) 
rnebottleopeningCall : for rne torques of bottleopening taks

MAIN CONCEPTS 

1.Two modes:

I.Fixed-base RNE: rnedynamicFixedBase1610.m
II.Mobile-base (coupled dual-arm) RNE: rnemobilebasec.m

2.The mobile base acceleration a_base is computed by coupling both arms' reaction Jacobians and effective base inertias. Use compute_base_accel.m which internally calls sum_base_coupling.m, compute_betarc_vel.m, and compute_betarc_ext.m.

3.armType is either 'A' or 'B'. It affects relevant DH offsets (notably d1 sign).

4.Jacobians and transforms come from get_robot_params.m (fixed) and get_robot_paramsmobile.m (mobile)

5.External wrenches: supported via manual input or Excel tables. Format described below.

FILES & PURPOSE

1.CALL SCRIPTS (Run These):

callLiftingTrajectory1610.m — generate lifting trajectories and save Excel.
callrneFixedBase1610.m — run fixed-base dynamics on data or generated Excel.
rnecombinedmobilebasecall.m — call mobile-base dual-arm RNE  on data or generated Excel.
visualisationMobilebaseLifting.m — 3D animation & trace visualization.
newcoordinatedtasks.m, rnebottleopeningCALL.m — Bottle opening task.

2.CORE ALGORITHM FILES:

rnedynamicFixedBase1610.m — fixed-base RNE main routine.
rnemobilebasec.m — mobile-base coupled dual-arm RNE main.

3.HELPERS (fixed base):

get_robot_params.m — DH, masses, COM, inertia, transforms.(change only in these if the robot test setup is different)
get_partial_rotation.m — rotation to a joint frame.
jacobianOfArmA.m, jacobianOfArmB.m — Jacobians using get_robot_params.
computePartialJacobiaN.m — partial Jacobian for external wrench.
compute_object_dynamics.m — object/EE wrench model.
exportAndPlotTau.m — export and plot torque results.

4.HELPERS (mobile base):

get_robot_paramsmobile.m — DH + base coupling (Mbase, Jrc).
get_partial_rotationm.m — rotation for mobile base frames.
compute_base_accel.m — compute shared a_base using both arms.
compute_betarc_vel.m — β_rc due to joint velocities.
compute_betarc_ext.m — β_rc due to external base wrenches.
sum_base_coupling.m — combine Mbase, Jrc, betarc for both arms.
compute_object_dynamicsm.m — mobile version of object wrench.

DATA FORMATS 
1.Below data will be created by IK solver callLiftingTrajectory1610.m, newcoordinatedtasks.m
I.Arm A:
Sheet 1: th_A (7 x n_steps) — joint positions (rows = joints)
Sheet 2: thdot_A (7 x n_steps) — velocities
Sheet 3: thddot_A (7 x n_steps) — accelerations
II.Arm B:
Sheet 4: th_B
Sheet 5: thdot_B
Sheet 6: thddot_B

2.External wrench Excel (if using method=Excel):
Each row: [joint_num, fx, fy, fz, mx, my, mz]
joint_num in 1..8 .


3.MATLAB INPUT STRUCTS:
I.baseState:
T_w0 — 4×4 transform (use eye(4) if none).
v0 — 3×1 base linear velocity.
w0 — 3×1 base angular velocity.
II.objParams:
mass — scalar.
I — 3×3 object inertia (about COM).
g_obj gravity vector.

OUTPUTS

.mat files: contain tauA, tauB, M_A, M_B, C_A, C_B, G_A, G_B, a_base_all.
Optional Excel sheets: ArmA_q, ArmA_qdot, ArmA_qddot, ArmB_q, ArmB_qdot, ArmB_qddot, ObjectPc, BaseAcc tauA & tauB.



