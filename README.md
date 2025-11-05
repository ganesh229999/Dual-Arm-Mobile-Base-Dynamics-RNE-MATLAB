# Dual-Arm-Mobile-Base-Dynamics-RNE-MATLAB

## OVERVIEW
This repository implements Recursive Newton-Euler (RNE) dynamics for a dual-arm robot with both fixed-base and mobile-base variants. It includes trajectory generation, torque computation, base-coupling (Khalil–Dombre style), object wrench models, visualization, and utilities for exporting results. Top-level scripts (those with "CALL" in the name) are entry points you run directly.

## QUICK START (3 steps)

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

## MAIN CONCEPTS 

1.Two modes:

I.Fixed-base RNE: rnedynamicFixedBase1610.m <br />
II.Mobile-base (coupled dual-arm) RNE: rnemobilebasec.m

2.The mobile base acceleration a_base is computed by coupling both arms' reaction Jacobians and effective base inertias. Use compute_base_accel.m which internally calls sum_base_coupling.m, compute_betarc_vel.m, and compute_betarc_ext.m.

3.armType is either 'A' or 'B'. It affects relevant DH offsets (notably d1 sign).

4.Jacobians and transforms come from get_robot_params.m (fixed) and get_robot_paramsmobile.m (mobile)

5.External wrenches: supported via manual input or Excel tables. Format described below.

## FILES & PURPOSE

1.CALL SCRIPTS (Run These):

callLiftingTrajectory1610.m — generate lifting trajectories and save Excel.<br />
callrneFixedBase1610.m — run fixed-base dynamics on data or generated Excel.<br />
rnecombinedmobilebasecall.m — call mobile-base dual-arm RNE  on data or generated Excel.<br />
visualisationMobilebaseLifting.m — 3D animation & trace visualization.<br />
newcoordinatedtasks.m, rnebottleopeningCALL.m — Bottle opening task.

2.CORE ALGORITHM FILES:

rnedynamicFixedBase1610.m — fixed-base RNE main routine.<br />
rnemobilebasec.m — mobile-base coupled dual-arm RNE main.

3.HELPERS (fixed base):

get_robot_params.m — DH, masses, COM, inertia, transforms.(change only in these if the robot test setup is different)<br />
get_partial_rotation.m — rotation to a joint frame.<br />
jacobianOfArmA.m, jacobianOfArmB.m — Jacobians using get_robot_params.<br />
computePartialJacobiaN.m — partial Jacobian for external wrench.<br />
compute_object_dynamics.m — object/EE wrench model.<br />
exportAndPlotTau.m — export and plot torque results.

4.HELPERS (mobile base):

get_robot_paramsmobile.m — DH + base coupling (Mbase, Jrc).
get_partial_rotationm.m — rotation for mobile base frames.<br />
compute_base_accel.m — compute shared a_base using both arms.<br />
compute_betarc_vel.m — β_rc due to joint velocities.<br />
compute_betarc_ext.m — β_rc due to external base wrenches.<br />
sum_base_coupling.m — combine Mbase, Jrc, betarc for both arms.<br />
compute_object_dynamicsm.m — mobile version of object wrench.

## DATA FORMATS 
1.Below data will be created by IK solver callLiftingTrajectory1610.m, newcoordinatedtasks.m
I.Arm A:<br />
Sheet 1: th_A (7 x n_steps) — joint positions (rows = joints)<br />
Sheet 2: thdot_A (7 x n_steps) — velocities<br />
Sheet 3: thddot_A (7 x n_steps) — accelerations
II.Arm B:<br />
Sheet 4: th_B<br />
Sheet 5: thdot_B<br />
Sheet 6: thddot_B<br />

2.External wrench Excel (if using method=Excel):<br />
Each row: [joint_num, fx, fy, fz, mx, my, mz]<br />
joint_num in 1..8 .<br />


3.MATLAB INPUT STRUCTS:<br />
I.baseState:<br />
T_w0 — 4×4 transform (use eye(4) if none).<br />
v0 — 3×1 base linear velocity.<br />
w0 — 3×1 base angular velocity.<br />
II.objParams:<br />
mass — scalar.<br />
I — 3×3 object inertia (about COM).<br />
g_obj gravity vector.

## OUTPUTS

.mat files: contain tauA, tauB, M_A, M_B, C_A, C_B, G_A, G_B, a_base_all.<br />
Optional Excel sheets: ArmA_q, ArmA_qdot, ArmA_qddot, ArmB_q, ArmB_qdot, ArmB_qddot, ObjectPc, BaseAcc tauA & tauB.



