FUNCTION_BLOCK FB_MotionFinal2
(*
    ===========================================================================
     FB_MotionFinal2_V4_5_ECAT   |  Build-ID: motion-final-450
    ===========================================================================
     Revision History
     --------------------------------------------------------------------------
     V4.5.0  |  2025-10-27  |  Kenneth Bazzett / ChatGPT (GPT-5)
     --------------------------------------------------------------------------
     - Added B-Axis torque-limit fallback logic (SoftMotion Mode A).
     - No functional changes to A-Axis or Idle/Run hysteresis logic.
     - Integrated bumpless torque ramp and pressure dwell release.
    ===========================================================================
*)

VAR CONSTANT
    REV_MAJOR  : UINT := 4;
    REV_MINOR  : UINT := 5;
    REV_PATCH  : UINT := 0;

    MIN_DT_S   : LREAL := 0.004;      // 4 ms floor for integrators
    PULSE_PT   : TIME  := T#250MS;    // RunCmd pulse width
    STARTUP_DELAY_TIME : TIME := T#2S;// Fixed-Idle dwell per RunCmd

    // --- B-Axis torque-limit constants ---
    P_LIMIT_B_psi    : LREAL := 800.0;     // pressure threshold
    P_HYST_B_psi     : LREAL := 80.0;      // hysteresis band
    DISP_B_mlPerRev  : LREAL := 7.9;       // displacement per rev
    GEAR_B           : LREAL := 10.0;      // motor:pump ratio
    ETA_MECH         : LREAL := 0.85;      // mechanical efficiency
    TORQUE_SLEW_Nm_s : LREAL := 0.5;       // torque slew rate
    DWELL_RELEASE_B  : TIME  := T#250MS;   // release dwell
END_VAR


// ============================================================================
//  CORE DECLARATIONS
// ============================================================================

VAR

// ============================================================================
//  CST-Idle torque control internal variables
// ============================================================================

// --- Feed-forward & computed torque values ---
A_TorqueFF_Nm       : LREAL := 0.0;    // feed-forward Nm from idle pressure
B_TorqueFF_Nm       : LREAL := 0.0;

A_TorqueTarget_Nm   : LREAL := 0.0;    // target torque from PID + feed-forward
B_TorqueTarget_Nm   : LREAL := 0.0;

A_TorqueCmd_Nm      : LREAL := 0.0;    // current commanded torque (slewed)
B_TorqueCmd_Nm      : LREAL := 0.0;

// --- PID and pressure control ---
errIdleA_psi        : LREAL := 0.0;    // instantaneous pressure error (psi)
errIdleB_psi        : LREAL := 0.0;

iTermIdleA_pct      : LREAL := 0.0;    // integrator for A-side PID (%)
iTermIdleB_pct      : LREAL := 0.0;

outIdleA_pct        : LREAL := 0.0;    // PID output before conversion (%)
outIdleB_pct        : LREAL := 0.0;

// --- Absolute error magnitudes for convergence check ---
errA_abs            : LREAL := 0.0;
errB_abs            : LREAL := 0.0;

// --- Arm & hysteresis flags ---
idleA_armed         : BOOL := FALSE;    // A side has dropped below Idle SP
idleB_armed         : BOOL := FALSE;    // B side has dropped below Idle SP


allowHysteresisSwitch: BOOL := FALSE;   // enables run/idle switching once armed

	

    // --- Revision / initialization ---
    initOnceDone   : BOOL := FALSE;
    ovrInitSeeded  : BOOL := FALSE;
    RevString      : STRING(32) := '';
    RevOk          : BOOL := FALSE;

    // --- Axis enables / power state ---
    axisA_EnableReq, axisB_EnableReq : BOOL := FALSE;
    cmdPower, cmdPowerA, cmdPowerB    : BOOL := FALSE;
    powerA_Ready, powerB_Ready, allAxesPowered : BOOL := FALSE;

    // --- Base motion parameters ---
    baseA_RPM_Clamped, baseB_RPM_Clamped : LREAL := 0.0;
    velA_revPerSec, velB_revPerSec : LREAL := 0.0;
    distA_revolutions, distB_revolutions : LREAL := 0.0;

    gearRatioA_MotorPerPump, gearRatioB_MotorPerPump : LREAL := 1.0;

    // --- Task timing / override slew ---
    task_dt_s : LREAL := 0.0;
    ovrSlew_per_s : LREAL := 2.0;
    overrideA_TargetFactor, overrideB_TargetFactor : LREAL := 1.0;
    overrideA_Factor, overrideB_Factor : LREAL := 1.0;

    // --- RPM windows for Idle / Run modes ---
    idleMinRPM_A, idleMaxRPM_A : LREAL;
    idleMinRPM_B, idleMaxRPM_B : LREAL;
    runMinRPM_A,  runMaxRPM_A  : LREAL;
    runMinRPM_B,  runMaxRPM_B  : LREAL;

    // --- Safety clamps ---
    factorSoftMin : LREAL := 0.0;
    factorSoftMax : LREAL := 1.0;

    // --- PID terms ---
    runCmdPrev, runCmdRiseEdge : BOOL := FALSE;
    tIdleToRun, tRunToIdle : TON;

     iTermRun_pct : LREAL := 0.0;
    errRunLead_psi, outRun_pct : LREAL := 0.0;

    // --- Flow / ratio / slip ---
    slipA_frac, slipB_frac : LREAL := 0.0;
    dispA_mlPerRev, dispB_mlPerRev : LREAL := 0.0;
    ratioA_req_frac : LREAL := 0.0;
    flowA_mlMin, flowB_mlMin : LREAL := 0.0;
    ratioAdjA_factor, ratioAdjB_factor : LREAL := 1.0;

    // --- RPM intermediates ---
    rpmA_target_unclamped, rpmB_target_unclamped : LREAL := 0.0;
    rpmA_target, rpmB_target : LREAL := 0.0;
    baseA_rpm_safe, baseB_rpm_safe : LREAL := 1.0;

    // --- Phase flag ---
    isRunPhase : BOOL := FALSE;
END_VAR


VAR
    // --- MC Function Blocks ---
    fbMoveVelA, fbMoveVelB : MC_MoveVelocity;
    fbMoveRelA, fbMoveRelB : MC_MoveRelative;
    fbStopA, fbStopB       : MC_Stop;
    fbResetA, fbResetB     : MC_Reset;
    fbOverrideA, fbOverrideB : MC_SetOverride;
    fbPowerA, fbPowerB     : MC_Power;

    // --- Command flags ---
    cmdMoveVelocity, cmdMoveRelative, cmdReset, doStopExec, pulseRunCmd : BOOL := FALSE;

    // --- State machine support ---
    tPower, tStart, tStopPulse, tResetPulse, tRunCmdPulse : TON;
    stateId, prevStateId : INT := 0;
    stateText : STRING(20) := 'STOPPED';
    AxisFault, anyMotionActive, opTimedOut, resetHoldLatched : BOOL := FALSE;
    doMoveVel, doMoveRel, doReset, doStop : BOOL := FALSE;
END_VAR


VAR
    // --- One-shot re-arm delay ---
    tOneShotRearm : TON;
END_VAR


// ============================================================================
//  MASTER / SLAVE & PID INTERMEDIATES
// ============================================================================

VAR
    leadIsA, leadIsA_tmp : BOOL := TRUE;
    ratioAdjFollow : LREAL := 1.0;
    rpmLead_unclamped, rpmFollower_unclamped, rpmLead_clamped, rpmFollower_clamped : LREAL := 0.0;
    rpmLead_safe, rpmFollower_safe : LREAL := 1.0;
    intRun_pct, intIdleA_pct, intIdleB_pct : LREAL := 0.0;
    APowerStatus, BPowerStatus : BOOL := FALSE;
END_VAR


// ============================================================================
//  STARTUP / IDLE MODE CONTROL VARIABLES
// ============================================================================

VAR
    rpmLead, rpmFollower    : LREAL := 0.0;
    idleA_ready, idleB_ready : BOOL := FALSE;

END_VAR


// ============================================================================
//  B-AXIS TORQUE LIMIT CONTROL VARIABLES
// ============================================================================

VAR
    TorqueMode_B       : BOOL := FALSE;
    TorqueCmd_B_Nm     : LREAL := 0.0;
    torqueTarget_Nm    : LREAL := 0.0;
    torqueNow_Nm       : LREAL := 0.0;
    PressureLimiting_B : BOOL := FALSE;
    dwellRelease_B     : TON;
	
	fbModeA, fbModeB   : SMC_SetControllerMode;
    fbVelA, fbVelB     : MC_MoveVelocity;
END_VAR
(*
    // ============================================================
    //  >>> B-AXIS TORQUE-MODE FALLBACK <<<
    // ============================================================
    VAR
        torqueFromPressure_Nm : LREAL;
    END_VAR
	(*
	VAR
    fbSetMode_B           : SMC_SetControllerMode;
    modeTarget_B          : SMC_CONTROLLER_MODE := SMC_velocity;
    modeSwitchInProgress_B: BOOL := FALSE;
    modeSwitchError_B     : BOOL := FALSE;
END_VAR *)
*)
// ---------- Local math constants ----------
VAR CONSTANT
    PI_LREAL : LREAL := 3.14159265358979323846;
END_VAR

VAR
    //tStopPulse : TON;
    //fbStopA, fbStopB : MC_Stop;
   // fbVelA, fbVelB : MC_MoveVelocity;
   // fbModeA, fbModeB : SMC_SetControllerMode;
    stopTrig : R_TRIG;     // you already use this pattern elsewhere
END_VAR

// ============================================================================
//  INTERNAL VARIABLES – TORQUE SCALING / CONVERSION
// ============================================================================
VAR
    A_Torque_Percent : LREAL := 0.0;       // A-axis torque scaled 0–1000 for PDO
    B_Torque_Percent : LREAL := 0.0;       // B-axis torque scaled 0–1000 for PDO
    target_torque_A  : INT   := 0;         // Final INT command mapped to PDO 16#1602
    target_torque_B  : INT   := 0;         // Final INT command mapped to PDO 16#1602
END_VAR

// ============================================================
//  PERSISTENT VARIABLE DEFAULTS – FOAM SYSTEM
// ============================================================

	// --- System and timing ---
	PersistentVars.TaskPeriod_s                := 0.004;     // 44 ms task loop
	PersistentVars.Override_Slew_per_s         := 2.0;      // 2 % per s slew
	PersistentVars.PidConvergenceWindow_psi    := 15.0;      // ±5 psi convergence
	
	// --- Axis enable flags ---
	PersistentVars.EnablePumpA                 := TRUE;
	PersistentVars.EnablePumpB                 := TRUE;
	
	// --- Mechanical / gear parameters ---
	PersistentVars.Gear_MotorPerPump_A         := 10.0;     // 10:1 reducer
	PersistentVars.Gear_MotorPerPump_B         := 10.0;
	PersistentVars.A_Displ_mlPerRev            := 7.9;      // mL/rev at pump shaft
	PersistentVars.B_Displ_mlPerRev            := 7.9;
	
	// --- Idle speed windows ---
	//PersistentVars.A_IdleRPM_Min               := 0.01;
	PersistentVars.A_IdleRPM_Max               := 10;
	//PersistentVars.B_IdleRPM_Min               := 0.01;
	PersistentVars.B_IdleRPM_Max               := 2;
	
	// --- Run speed windows ---
	PersistentVars.A_RunRPM_Min                := 16.0;
	PersistentVars.A_RunRPM_Max                := 280.0;
	PersistentVars.B_RunRPM_Min                := 11.0;
	PersistentVars.B_RunRPM_Max                := 220.0;
	
	// --- Base RPM setpoints ---
	PersistentVars.A_MinRPM                    := 0.02;
	PersistentVars.A_RPM_Setpoint              := 250.0;
	PersistentVars.A_SideMaxRPM                := 280;
	PersistentVars.B_MinRPM                    := 0.02;
	PersistentVars.B_RPM_Setpoint              := 200.0;
	PersistentVars.B_SideMaxRPM                := 240;
	
	// --- Accel / Decel parameters ---
	PersistentVars.A_Accel_rpmps               := 300.0;
	PersistentVars.A_Decel_rpmps               := 5000.0;
	PersistentVars.B_Accel_rpmps               := 300.0;
	PersistentVars.B_Decel_rpmps               := 5000.0;
	
	// --- Volume / motion parameters ---
	PersistentVars.A_RelMoveVolume_ml          := 100.0;
	PersistentVars.B_RelMoveVolume_ml          := 100.0;
	
	// --- Slip compensation ---
	PersistentVars.A_SlipGain_perPsi           := 0.0001;   // fractional slip per psi
	PersistentVars.B_SlipGain_perPsi           := 0.0001;
	
	// --- Mode and enable selectors ---
	PersistentVars.Mode                        := 0;        // 0=velocity/manual, 1=relative
	PersistentVars.RatioEnable                 := FALSE;
	PersistentVars.RatioMode                   := 0;        // 0=A master, 1=B master
	PersistentVars.A_RatioFraction             := 0.5;      // 50:50
	PersistentVars.PID_LeadSide                := TRUE;    // A leads (FALSE=A, TRUE=B)
	PersistentVars.PID_RunLeadEnable           := TRUE;
	
	// --- PID Run loop parameters ---
	PersistentVars.PID_Run_SP_psi              := 100.0;
	PersistentVars.PID_Bias_Pct                := 50.0;
	PersistentVars.A_Spray_PID_Kp_PctPerPsi    := 0.2;
	PersistentVars.A_Spray_PID_Ki_PctPerPsiPerS:= 0.02;
	PersistentVars.B_Spray_PID_Kp_PctPerPsi    := 0.25;
	PersistentVars.B_Spray_PID_Ki_PctPerPsiPerS:= 0.02;
	
	// --- Idle pressure PID setpoints ---
	PersistentVars.PID_Idle_SP_A_psi           := 40.0;
	PersistentVars.PID_Idle_SP_B_psi           := 60.0;
	PersistentVars.PID_Idle_Bias_Pct           := 20.0;
	
	// --- Idle PID gains ---
	PersistentVars.A_Idle_PID_Kp_PctPerPsi     := 0.25;
	PersistentVars.A_Idle_PID_Ki_PctPerPsiPerS := 0.02;
	PersistentVars.B_Idle_PID_Kp_PctPerPsi     := 0.25;
	PersistentVars.B_Idle_PID_Ki_PctPerPsiPerS := 0.02;
	
	// --- Idle torque limits and behavior ---
	PersistentVars.A_IdleTorque_Max_Nm         := 0.5;
	PersistentVars.A_IdleTorque_Slew_Nm_s      := 0.4;
	PersistentVars.A_Idle_FricComp_Nm          := 0.05;
	PersistentVars.A_Idle_Eta_Mech             := 0.85;
	PersistentVars.B_IdleTorque_Max_Nm         := 0.5;
	PersistentVars.B_IdleTorque_Slew_Nm_s      := 0.4;
	PersistentVars.B_Idle_FricComp_Nm          := 0.05;
	PersistentVars.B_Idle_Eta_Mech             := 0.85;
	
	// --- Pressure limits and hysteresis ---
	PersistentVars.A_PressureRunLimit_psi      := 200.0;
	PersistentVars.B_PressureRunLimit_psi      := 300.0;
	PersistentVars.A_PressureIdleHyst_psi      := 5.0;
	PersistentVars.B_PressureIdleHyst_psi      := 10.0;
	
	// --- Miscellaneous HMI-linked parameters ---
	//PersistentVars.A_IdleRPM                   := 40.0;
	//PersistentVars.B_IdleRPM                   := 50.0;
	
	// --- Diagnostic / reserved (optional default placeholders) ---
	//PersistentVars.PID_RunLeadEnable           := TRUE;
	//PersistentVars.PID_IdleMode                := TRUE;

// ============================================================================
//  LOAD PARAMETERS AND BASE INITIALIZATION
// ============================================================================

	IF NOT initOnceDone THEN
	    RevString := FN_RevString(REV_MAJOR, REV_MINOR, REV_PATCH);
	    RevOk     := FN_RevCompatible(REV_MAJOR, REV_MINOR, REV_PATCH, 1, 0);
	    initOnceDone := TRUE;
	END_IF;
	
	IF NOT ovrInitSeeded THEN
	    overrideA_Factor := 1.0;
	    overrideB_Factor := 1.0;
	    overrideA_TargetFactor := 1.0;
	    overrideB_TargetFactor := 1.0;
	    factorSoftMin := 0.0;
	    factorSoftMax := 1.0;
	    ovrSlew_per_s := 20.0;
	    ovrInitSeeded := TRUE;
	END_IF;
	
	// --- Refresh run RPM windows ---
	
	runMinRPM_A  := PersistentVars.A_RunRPM_Min;
	runMaxRPM_A  := PersistentVars.A_RunRPM_Max;
	runMinRPM_B  := PersistentVars.B_RunRPM_Min;
	runMaxRPM_B  := PersistentVars.B_RunRPM_Max;
	
	// --- Task period clamp ---
	task_dt_s := MAX(REAL_TO_LREAL(PersistentVars.TaskPeriod_s), MIN_DT_S);
	ovrSlew_per_s := LIMIT(LREAL#0.0, REAL_TO_LREAL(PersistentVars.Override_Slew_per_s), LREAL#20.0);
	
	// --- Axis enable flags / power FBs ---
	axisA_EnableReq := PersistentVars.EnablePumpA;
	axisB_EnableReq := PersistentVars.EnablePumpB;
	
	fbPowerA(Axis := Drive_A, Enable := cmdPower, bRegulatorOn := cmdPower, bDriveStart := cmdPower, Status => APowerStatus);
	fbPowerB(Axis := Drive_B, Enable := cmdPower, bRegulatorOn := cmdPower, bDriveStart := cmdPower, Status => BPowerStatus);
	allAxesPowered := APowerStatus AND BPowerStatus;
	
	// --- Gear ratios / base RPM clamps ---
	gearRatioA_MotorPerPump := MAX(PersistentVars.Gear_MotorPerPump_A, 1E-9);
	gearRatioB_MotorPerPump := MAX(PersistentVars.Gear_MotorPerPump_B, 1E-9);
	
	baseA_RPM_Clamped := LIMIT(REAL_TO_LREAL(PersistentVars.A_MinRPM),
	                            REAL_TO_LREAL(PersistentVars.A_RPM_Setpoint),
	                            REAL_TO_LREAL(PersistentVars.A_SideMaxRPM));
	baseB_RPM_Clamped := LIMIT(REAL_TO_LREAL(PersistentVars.B_MinRPM),
	                            REAL_TO_LREAL(PersistentVars.B_RPM_Setpoint),
	                            REAL_TO_LREAL(PersistentVars.B_SideMaxRPM));
	
	// --- Convert to rev/s and compute distances ---
	velA_revPerSec := baseA_RPM_Clamped * gearRatioA_MotorPerPump / 60.0;
	velB_revPerSec := baseB_RPM_Clamped * gearRatioB_MotorPerPump / 60.0;
	
	distA_revolutions := REAL_TO_LREAL(PersistentVars.A_RelMoveVolume_ml)
	                     / MAX(REAL_TO_LREAL(PersistentVars.A_Displ_mlPerRev), 1E-9)
	                     * gearRatioA_MotorPerPump;
	distB_revolutions := REAL_TO_LREAL(PersistentVars.B_RelMoveVolume_ml)
	                     / MAX(REAL_TO_LREAL(PersistentVars.B_Displ_mlPerRev), 1E-9)
	                     * gearRatioB_MotorPerPump;
	
	// --- HMI mirrors ---
	gAxes.DeBugA_BaseRPM_Clamped := LREAL_TO_REAL(baseA_RPM_Clamped);
	gAxes.DeBugB_BaseRPM_Clamped := LREAL_TO_REAL(baseB_RPM_Clamped);
	gAxes.DeBugA_Vel_RevPerSec   := LREAL_TO_REAL(velA_revPerSec);
	gAxes.DeBugB_Vel_RevPerSec   := LREAL_TO_REAL(velB_revPerSec);

// ============================================================================
//  PID / RATIO / OVERRIDE COMPUTATION  (Idle and Run)
// ============================================================================

	ratioAdjA_factor := 1.0;
	ratioAdjB_factor := 1.0;
	
	// --- Slip compensation ---
	slipA_frac := LIMIT(0.0,
	                    PersistentVars.A_SlipGain_perPsi *
	                    REAL_TO_LREAL(gAxes.A_PressureHP_psi), 0.9);
	slipB_frac := LIMIT(0.0,
	                    PersistentVars.B_SlipGain_perPsi *
	                    REAL_TO_LREAL(gAxes.B_PressureHP_psi), 0.9);
	
	// --- Displacements / flows ---
	dispA_mlPerRev := MAX(PersistentVars.A_Displ_mlPerRev, 0.001);
	dispB_mlPerRev := MAX(PersistentVars.B_Displ_mlPerRev, 0.001);
	flowA_mlMin := baseA_RPM_Clamped * dispA_mlPerRev * (1.0 - slipA_frac);
	flowB_mlMin := baseB_RPM_Clamped * dispB_mlPerRev * (1.0 - slipB_frac);
	
	// --- Determine PID lead ---
	leadIsA := (NOT PersistentVars.PID_LeadSide);
	

// ============================================================================
//  CST-IDLE TORQUE CONTROL  (Independent per axis)
// ============================================================================

	// --- A-axis feed-forward torque from target pressure ---
	A_TorqueFF_Nm :=
	    (REAL_TO_LREAL(PersistentVars.PID_Idle_SP_A_psi) * 6894.76 *
	     (REAL_TO_LREAL(PersistentVars.A_Displ_mlPerRev) * 1.0E-6))
	     / ((2.0 * PI_LREAL) * REAL_TO_LREAL(PersistentVars.A_Idle_Eta_Mech)
	        * gearRatioA_MotorPerPump);
	
	// --- B-axis feed-forward torque ---
	B_TorqueFF_Nm :=
	    (REAL_TO_LREAL(PersistentVars.PID_Idle_SP_B_psi) * 6894.76 *
	     (REAL_TO_LREAL(PersistentVars.B_Displ_mlPerRev) * 1.0E-6))
	     / ((2.0 * PI_LREAL) * REAL_TO_LREAL(PersistentVars.B_Idle_Eta_Mech)
	        * gearRatioB_MotorPerPump);
	
	// --- Error terms (psi) ---
	errIdleA_psi := REAL_TO_LREAL(PersistentVars.PID_Idle_SP_A_psi)
	                - REAL_TO_LREAL(gAxes.A_PressureHP_psi);
	errIdleB_psi := REAL_TO_LREAL(PersistentVars.PID_Idle_SP_B_psi)
	                - REAL_TO_LREAL(gAxes.B_PressureHP_psi);
	
	// --- Integrators (anti-windup, no negative accumulation) ---
	IF (errIdleA_psi > 0.0) THEN
	    iTermIdleA_pct := LIMIT(0.0, iTermIdleA_pct +
	        REAL_TO_LREAL(PersistentVars.A_Idle_PID_Ki_PctPerPsiPerS)
	        * errIdleA_psi * task_dt_s, 100.0);
	ELSIF (errIdleA_psi < 0.0) THEN
	    // decay integral back toward zero instead of winding negative
	    iTermIdleA_pct := iTermIdleA_pct * 0.9;
	END_IF;
	
	IF (errIdleB_psi > 0.0) THEN
	    iTermIdleB_pct := LIMIT(0.0, iTermIdleB_pct +
	        REAL_TO_LREAL(PersistentVars.B_Idle_PID_Ki_PctPerPsiPerS)
	        * errIdleB_psi * task_dt_s, 100.0);
	ELSIF (errIdleB_psi < 0.0) THEN
	    iTermIdleB_pct := iTermIdleB_pct * 0.9;
	END_IF;
	
	// --- PID outputs (% of torque span, clamped ≥ 0) ---
	outIdleA_pct := LIMIT(0.0,
	    REAL_TO_LREAL(PersistentVars.PID_Idle_Bias_Pct)
	  + REAL_TO_LREAL(PersistentVars.A_Idle_PID_Kp_PctPerPsi) * errIdleA_psi
	  + iTermIdleA_pct,
	    100.0);
	
	outIdleB_pct := LIMIT(0.0,
	    REAL_TO_LREAL(PersistentVars.PID_Idle_Bias_Pct)
	  + REAL_TO_LREAL(PersistentVars.B_Idle_PID_Kp_PctPerPsi) * errIdleB_psi
	  + iTermIdleB_pct,
	    100.0);
	
	// --- Convert % → Nm and add small hold bias ---
	A_TorqueTarget_Nm := MAX(REAL_TO_LREAL(PersistentVars.A_Idle_FricComp_Nm),
	    A_TorqueFF_Nm + (outIdleA_pct / 100.0)
	    * REAL_TO_LREAL(PersistentVars.A_IdleTorque_Max_Nm));
	
	B_TorqueTarget_Nm := MAX(REAL_TO_LREAL(PersistentVars.B_Idle_FricComp_Nm),
	    B_TorqueFF_Nm + (outIdleB_pct / 100.0)
	    * REAL_TO_LREAL(PersistentVars.B_IdleTorque_Max_Nm));
	
	// --- Slew and clamp (same as before) ---
	A_TorqueCmd_Nm := LIMIT(0.0,
	    A_TorqueCmd_Nm + LIMIT(A_TorqueTarget_Nm - A_TorqueCmd_Nm,
	          -REAL_TO_LREAL(PersistentVars.A_IdleTorque_Slew_Nm_s) * task_dt_s,
	           REAL_TO_LREAL(PersistentVars.A_IdleTorque_Slew_Nm_s) * task_dt_s),
	    REAL_TO_LREAL(PersistentVars.A_IdleTorque_Max_Nm));
	
	B_TorqueCmd_Nm := LIMIT(0.0,
	    B_TorqueCmd_Nm + LIMIT(B_TorqueTarget_Nm - B_TorqueCmd_Nm,
	          -REAL_TO_LREAL(PersistentVars.B_IdleTorque_Slew_Nm_s) * task_dt_s,
	           REAL_TO_LREAL(PersistentVars.B_IdleTorque_Slew_Nm_s) * task_dt_s),
	    REAL_TO_LREAL(PersistentVars.B_IdleTorque_Max_Nm));

// ============================================================================
//  TORQUE OUTPUT SCALING  (0 … 1000 INT for JMC EtherCAT drive)
// ============================================================================

	// Normalize Nm command → 0-1000 integer range (1000 = full rated torque)
	A_Torque_Percent := LIMIT(0.0,
	    (A_TorqueCmd_Nm / REAL_TO_LREAL(PersistentVars.A_IdleTorque_Max_Nm)) * 1000.0,
	    1000.0);
	
	B_Torque_Percent := LIMIT(0.0,
	    (B_TorqueCmd_Nm / REAL_TO_LREAL(PersistentVars.B_IdleTorque_Max_Nm)) * 1000.0,
	    1000.0);
	
	// Force clamp again for safety (no negatives slip through)
	A_Torque_Percent := MAX(0.0, A_Torque_Percent);
	B_Torque_Percent := MAX(0.0, B_Torque_Percent);
	
	// Cast to INT for PDO (16#1602 target_torque)
	target_torque_A := REAL_TO_INT(A_Torque_Percent);
	target_torque_B := REAL_TO_INT(B_Torque_Percent);
	
	
	// --- Write to PDOs ---
	gAxes.CST_A_TorqueCmd_Req_Nm := target_torque_A;
	gAxes.CST_B_TorqueCmd_Req_Nm := target_torque_B;
	
	(*
	// --- Switch both axes to torque (CST) while idle ---
	fbDualModeSwitch(
	    Enable      := TRUE,
	    ModeTorque  := TRUE,
	    Stop        := FALSE,
	    Velocity_rpm:= PersistentVars.A_IdleRPM
	);
	*)

// ============================================================================
//  RUN MODE LOGIC
// ============================================================================

	IF gAxes.RunCmd AND isRunPhase THEN
	    baseA_rpm_safe := MAX(baseA_RPM_Clamped, 1E-9);
	    baseB_rpm_safe := MAX(baseB_RPM_Clamped, 1E-9);
	
	    // Select lead side
	    IF PersistentVars.PID_LeadSide THEN
	        leadIsA := FALSE;  // B leads
	    ELSE
	        leadIsA := TRUE;   // A leads
	    END_IF;
	
	    // Ratio adjustment
	    IF PersistentVars.RatioEnable THEN
	        ratioA_req_frac := LIMIT(0.0, REAL_TO_LREAL(PersistentVars.A_RatioFraction), 1.0);
	        IF (ratioA_req_frac <= 0.0) OR (ratioA_req_frac >= 1.0) THEN
	            ratioAdjA_factor := 1.0;
	            ratioAdjB_factor := 1.0;
	        ELSE
	            IF PersistentVars.RatioMode = 0 THEN
	                ratioAdjA_factor := 1.0;
	                ratioAdjB_factor := (1.0 - ratioA_req_frac) / ratioA_req_frac;
	            ELSE
	                ratioAdjA_factor := (ratioA_req_frac) / (1.0 - ratioA_req_frac);
	                ratioAdjB_factor := 1.0;
	            END_IF;
	        END_IF;
	    ELSE
	        ratioAdjA_factor := 1.0;
	        ratioAdjB_factor := 1.0;
	    END_IF;
	
	    // Manual / PID run
	    IF PersistentVars.Mode = 0 THEN
	        IF NOT PersistentVars.PID_RunLeadEnable THEN
	            rpmA_target := LIMIT(runMinRPM_A, baseA_RPM_Clamped * ratioAdjA_factor, runMaxRPM_A);
	            rpmB_target := LIMIT(runMinRPM_B, baseB_RPM_Clamped * ratioAdjB_factor, runMaxRPM_B);
	        ELSE
	            IF leadIsA THEN
	                errRunLead_psi := REAL_TO_LREAL(PersistentVars.PID_Run_SP_psi)
	                                - REAL_TO_LREAL(gAxes.A_PressureHP_psi);
	                outRun_pct := REAL_TO_LREAL(PersistentVars.PID_Bias_Pct)
	                            + REAL_TO_LREAL(PersistentVars.A_Spray_PID_Kp_PctPerPsi) * errRunLead_psi
	                            + iTermRun_pct;
	                rpmA_target_unclamped := baseA_RPM_Clamped * (outRun_pct / 100.0);
	                IF (rpmA_target_unclamped > runMinRPM_A) AND (rpmA_target_unclamped < runMaxRPM_A) THEN
	                    iTermRun_pct := iTermRun_pct +
	                        REAL_TO_LREAL(PersistentVars.A_Spray_PID_Ki_PctPerPsiPerS) *
	                        errRunLead_psi * task_dt_s;
	                END_IF;
	                rpmA_target := LIMIT(runMinRPM_A, rpmA_target_unclamped, runMaxRPM_A);
	                rpmB_target := LIMIT(runMinRPM_B, rpmA_target * ratioAdjB_factor, runMaxRPM_B);
	            ELSE
	                errRunLead_psi := REAL_TO_LREAL(PersistentVars.PID_Run_SP_psi)
	                                - REAL_TO_LREAL(gAxes.B_PressureHP_psi);
	                outRun_pct := REAL_TO_LREAL(PersistentVars.PID_Bias_Pct)
	                            + REAL_TO_LREAL(PersistentVars.B_Spray_PID_Kp_PctPerPsi) * errRunLead_psi
	                            + iTermRun_pct;
	                rpmB_target_unclamped := baseB_RPM_Clamped * (outRun_pct / 100.0);
	                IF (rpmB_target_unclamped > runMinRPM_B) AND (rpmB_target_unclamped < runMaxRPM_B) THEN
	                    iTermRun_pct := iTermRun_pct +
	                        REAL_TO_LREAL(PersistentVars.B_Spray_PID_Ki_PctPerPsiPerS) *
	                        errRunLead_psi * task_dt_s;
	                END_IF;
	                rpmB_target := LIMIT(runMinRPM_B, rpmB_target_unclamped, runMaxRPM_B);
	                rpmA_target := LIMIT(runMinRPM_A, rpmB_target * ratioAdjA_factor, runMaxRPM_A);
	            END_IF;
	        END_IF;
	    END_IF;
	
	    // Overrides
	    IF overrideA_TargetFactor <> 0.0 THEN
	        overrideA_TargetFactor := LIMIT(0.0, rpmA_target / baseA_rpm_safe, 1.0);
	    END_IF;
	    IF overrideB_TargetFactor <> 0.0 THEN
	        overrideB_TargetFactor := LIMIT(0.0, rpmB_target / baseB_rpm_safe, 1.0);
	    END_IF;
	END_IF;
	
	

// ============================================================================
//  MODE BEHAVIOR – AUTO TOGGLE BETWEEN CST (Idle) AND CSV (Run)
//  • Handles normal idle/run switching (isRunPhase)
//  • Also handles trigger release (NOT gAxes.RunCmd)
//  • Ensures torque → velocity mode transition before stop
// ============================================================================


// ============================================================
// RUN CMD RELEASED  →  Switch to VELOCITY mode + STOP
// ============================================================
			
			IF NOT gAxes.RunCmd THEN
			    // --- 1. Zero torque outputs if still in CST ---
			    IF (Drive_A.byRealControllerMode = SMC_torque)
			       OR (Drive_B.byRealControllerMode = SMC_torque) THEN
			        gAxes.CST_A_TorqueCmd_Req_Nm := 0;
			        gAxes.CST_B_TorqueCmd_Req_Nm := 0;
					gAxes.CST_A_TorqueMode_Motor_Max_Speed := 0; 
    				gAxes.CST_B_TorqueMode_Motor_Max_Speed := 0; 
			    END_IF;
			
			    // --- 2. Request VELOCITY mode on both drives ---
			    fbModeA(Axis := Drive_A, bExecute := TRUE, nControllerMode := SMC_velocity);
			    fbModeB(Axis := Drive_B, bExecute := TRUE, nControllerMode := SMC_velocity);
			    fbModeA(Axis := Drive_A, bExecute := FALSE);
			    fbModeB(Axis := Drive_B, bExecute := FALSE);
			
			    // --- 3. When both drives confirm velocity mode, command stop ---
			    IF (Drive_A.byRealControllerMode = SMC_VELOCITY)
			       AND (Drive_B.byRealControllerMode = SMC_VELOCITY) THEN
			        fbVelA(Axis := Drive_A, Execute := TRUE, Velocity := 0.0,
			               Acceleration := PersistentVars.A_Accel_rpmps,
			               Deceleration := PersistentVars.A_Decel_rpmps);
			        fbVelB(Axis := Drive_B, Execute := TRUE, Velocity := 0.0,
			               Acceleration := PersistentVars.B_Accel_rpmps,
			               Deceleration := PersistentVars.B_Decel_rpmps);
			
			        // Signal stop sequence to state machine
			        doStopExec := TRUE;
			    END_IF;
		
	// ---------------------------------------------------------------------------
	// Normal CST / CSV switching based on idle/run phase
	// ---------------------------------------------------------------------------
	ELSIF NOT isRunPhase THEN
		
	    // ============================================================
	    // IDLE TORQUE MODE (CST)
	    // ============================================================
	    IF (Drive_A.byRealControllerMode = SMC_VELOCITY)
	       AND (Drive_B.byRealControllerMode = SMC_VELOCITY) THEN
	        fbVelA(Axis := Drive_A, Execute := FALSE, Velocity := 0.0);
	        fbVelB(Axis := Drive_B, Execute := FALSE, Velocity := 0.0);
	    END_IF;
	
	    fbModeA(Axis := Drive_A, bExecute := TRUE, nControllerMode := SMC_torque);
	    fbModeB(Axis := Drive_B, bExecute := TRUE, nControllerMode := SMC_torque);
	    fbModeA(Axis := Drive_A, bExecute := FALSE);
	    fbModeB(Axis := Drive_B, bExecute := FALSE);
	
	    IF (Drive_A.byRealControllerMode = SMC_torque)
	       AND (Drive_B.byRealControllerMode = SMC_torque) THEN
	        gAxes.CST_A_TorqueCmd_Req_Nm := target_torque_A;
	        gAxes.CST_B_TorqueCmd_Req_Nm := target_torque_B;
			gAxes.CST_A_TorqueMode_Motor_Max_Speed := PersistentVars.A_IdleRPM_Max*10; 
    		gAxes.CST_B_TorqueMode_Motor_Max_Speed := PersistentVars.B_IdleRPM_Max*10;
	    	overrideA_TargetFactor := 1.0;  //added to keep torque mode moving 
            overrideB_TargetFactor := 1.0;  //added to keep torque mode moving 
		END_IF;
	
	ELSE
	    // ============================================================
	    // RUN VELOCITY MODE (CSV)
	    // ============================================================
	    IF (Drive_A.byRealControllerMode = SMC_torque)
	       AND (Drive_B.byRealControllerMode = SMC_torque) THEN
	        gAxes.CST_A_TorqueCmd_Req_Nm := 0;
	        gAxes.CST_B_TorqueCmd_Req_Nm := 0;
	    END_IF;
	
	    fbModeA(Axis := Drive_A, bExecute := TRUE, nControllerMode := SMC_velocity);
	    fbModeB(Axis := Drive_B, bExecute := TRUE, nControllerMode := SMC_velocity);
	    fbModeA(Axis := Drive_A, bExecute := FALSE);
	    fbModeB(Axis := Drive_B, bExecute := FALSE);
	
	    IF (Drive_A.byRealControllerMode = SMC_VELOCITY)
	       AND (Drive_B.byRealControllerMode = SMC_VELOCITY) THEN
	        gAxes.CST_A_TorqueMode_Motor_Max_Speed := PersistentVars.A_SideMaxRPM*10; 
    		gAxes.CST_B_TorqueMode_Motor_Max_Speed := PersistentVars.B_SideMaxRPM*10;			
			fbVelA(
	            Axis := Drive_A,
	            Execute := TRUE,
	            Velocity := baseA_RPM_Clamped / 60.0,
	            Acceleration := PersistentVars.A_Accel_rpmps,
	            Deceleration := PersistentVars.A_Decel_rpmps
	        );
	        fbVelB(
	            Axis := Drive_B,
	            Execute := TRUE,
	            Velocity := baseB_RPM_Clamped / 60.0,
	            Acceleration := PersistentVars.B_Accel_rpmps,
	            Deceleration := PersistentVars.B_Decel_rpmps
	        );
	    END_IF;
	END_IF;

// ============================================================================
//  OVERRIDE FACTOR SLEW
// ============================================================================
	
	IF overrideA_TargetFactor > overrideA_Factor THEN
	    overrideA_Factor := MIN(overrideA_TargetFactor, overrideA_Factor + ovrSlew_per_s * task_dt_s);
	ELSE
	    overrideA_Factor := overrideA_TargetFactor;
	END_IF;
	
	IF overrideB_TargetFactor > overrideB_Factor THEN
	    overrideB_Factor := MIN(overrideB_TargetFactor, overrideB_Factor + ovrSlew_per_s * task_dt_s);
	ELSE
	    overrideB_Factor := overrideB_TargetFactor;
	END_IF;


// ============================================================================
//  MOTION EXECUTION
// ============================================================================

	 IF (Drive_A.byRealControllerMode = SMC_VELOCITY)
	       AND (Drive_B.byRealControllerMode = SMC_VELOCITY) THEN
		   
		fbOverrideA(Axis := Drive_A, Enable := TRUE, VelFactor := overrideA_Factor);
		fbOverrideB(Axis := Drive_B, Enable := TRUE, VelFactor := overrideB_Factor);
	
		fbMoveVelA(
	    	Axis := Drive_A,
	    	Execute := cmdMoveVelocity AND axisA_EnableReq,
	    	Velocity := velA_revPerSec,
	    	Acceleration := PersistentVars.A_Accel_rpmps,
	    	Deceleration := PersistentVars.A_Decel_rpmps,
	    	Direction := MC_Direction.current,
	    	BufferMode := MC_BUFFER_MODE.Aborting
		);
		
		fbMoveVelB(
	   		 Axis := Drive_B,
	   		 Execute := cmdMoveVelocity AND axisB_EnableReq,
	   		 Velocity := velB_revPerSec,
	   		 Acceleration := PersistentVars.B_Accel_rpmps,
	   		 Deceleration := PersistentVars.B_Decel_rpmps,
	   		 Direction := MC_Direction.current,
	   		 BufferMode := MC_BUFFER_MODE.Aborting
		);
	
	END_IF

// ============================================================================
//  STOP / RESET COMMANDS
// ============================================================================
	
	   IF (Drive_A.byRealControllerMode <> SMC_torque)
	       AND (Drive_B.byRealControllerMode <> SMC_torque) THEN
	 
	 
	
	fbStopA(Axis := Drive_A, Execute := doStopExec,
	         Deceleration := PersistentVars.A_Decel_rpmps);
	fbStopB(Axis := Drive_B, Execute := doStopExec,
	         Deceleration := PersistentVars.B_Decel_rpmps);
	fbResetA(Axis := Drive_A, Execute := cmdReset);
	fbResetB(Axis := Drive_B, Execute := cmdReset);
	
	   END_IF;


// ============================================================================
//  HIGH-LEVEL STATE MACHINE  –  V4.5 _ECAT
// ============================================================================

CASE stateId OF

0: // STOPPED
    stateText := 'STOPPED';
    cmdPower := FALSE;
    cmdMoveVelocity := FALSE;
    cmdMoveRelative := FALSE;
    cmdReset := FALSE;
    doStopExec := FALSE;

    allowHysteresisSwitch := FALSE;
  

    IF AxisFault THEN
        doStopExec := TRUE;
        stateId := 5;
    ELSIF gAxes.RunCmd THEN
        cmdPower := TRUE;
        iTermIdleA_pct := 0.0;
        iTermIdleB_pct := 0.0;
        iTermRun_pct := 0.0;
        stateId := 1;
    END_IF;

1: // POWERING
    stateText := 'POWERING';
    cmdPower := TRUE;
    allAxesPowered := fbPowerA.Status AND fbPowerB.Status;
    IF AxisFault THEN
        stateId := 5;
    ELSIF allAxesPowered THEN
        cmdMoveVelocity := TRUE;
        stateId := 3;
    END_IF;

// ----------------------------------------------------------------------------
// 3 – RUNNING
// ----------------------------------------------------------------------------
3:
    stateText := 'RUNNING';
    cmdPower  := TRUE;

    // ============================================================
    //  SMART STARTUP READINESS  (runs each RunCmd activation)
    // ============================================================
  
        // PID-Idle convergence
        errA_abs := ABS(REAL_TO_LREAL(PersistentVars.PID_Idle_SP_A_psi)
                     - REAL_TO_LREAL(gAxes.A_PressureHP_psi));
        errB_abs := ABS(REAL_TO_LREAL(PersistentVars.PID_Idle_SP_B_psi)
                     - REAL_TO_LREAL(gAxes.B_PressureHP_psi));
        IF (errA_abs <= REAL_TO_LREAL(PersistentVars.PidConvergenceWindow_psi)) AND
           (errB_abs <= REAL_TO_LREAL(PersistentVars.PidConvergenceWindow_psi)) THEN
             allowHysteresisSwitch := TRUE;
        END_IF;
      
    // ============================================================
    //  RUN → IDLE / IDLE → RUN  WITH PER-AXIS ARMING
    // ============================================================
    IF allowHysteresisSwitch THEN

        // --- Run → Idle  (either axis above its Idle SP) ---
        IF (gAxes.A_PressureHP_psi >= PersistentVars.A_PressureRunLimit_psi) OR
           (gAxes.B_PressureHP_psi >= PersistentVars.B_PressureRunLimit_psi) THEN
            overrideA_TargetFactor := 0.0;
            overrideB_TargetFactor := 0.0;    
            idleA_armed := FALSE;
            idleB_armed := FALSE;
            isRunPhase  := FALSE;
        END_IF;

        // --- Arm once pressure falls back below Idle SP ---
        IF (gAxes.A_PressureHP_psi <= PersistentVars.PID_Idle_SP_A_psi) THEN
            idleA_armed := TRUE;
        END_IF;
        IF (gAxes.B_PressureHP_psi <= PersistentVars.PID_Idle_SP_B_psi) THEN
            idleB_armed := TRUE;
        END_IF;

        // --- Enter RUN when both armed and any pressure ≤ hysteresis ---
        IF (NOT isRunPhase)
           AND (idleA_armed AND idleB_armed)
           AND ((gAxes.A_PressureHP_psi <= PersistentVars.A_PressureIdleHyst_psi) OR
                (gAxes.B_PressureHP_psi <= PersistentVars.B_PressureIdleHyst_psi)) THEN
            isRunPhase := TRUE;
            iTermIdleA_pct := 0.0;
            iTermIdleB_pct := 0.0;
        END_IF;
    ELSE
        // Startup hold → force idle until ready
        isRunPhase := FALSE;
        tIdleToRun(IN := FALSE);
        tRunToIdle(IN := FALSE);
    END_IF;

    // ============================================================
    //  NORMAL RUN SUPERVISION  (unchanged)
    // ============================================================
    IF AxisFault THEN
        doStopExec := TRUE;
        stateId := 5;                       // → STOPPING_FOR_RESET
    ELSIF NOT gAxes.RunCmd THEN
        doStopExec := TRUE;
        stateId := 0;                       // trigger released → STOPPED
        idleA_armed := FALSE;
        idleB_armed := FALSE;
        isRunPhase  := FALSE;
    ELSIF (PersistentVars.Mode = 1)
          AND (NOT fbMoveRelA.Active OR NOT fbMoveRelB.Active) THEN
        doStopExec := TRUE;
        pulseRunCmd := TRUE;
        stateId := 0;
    END_IF;


5: // STOPPING_FOR_RESET
    stateText := 'STOPPING_FOR_RESET';
    cmdPower := TRUE;
	doStopExec := TRUE;
    IF fbStopA.Done AND fbStopB.Done THEN
        doStopExec := FALSE;
        cmdReset := TRUE;
        stateId := 6;
    END_IF;

6: // RESETTING
    stateText := 'RESETTING';
    cmdPower := TRUE;
    IF NOT AxisFault THEN
        cmdReset := FALSE;
        resetHoldLatched := TRUE;
        stateId := 0;
    END_IF;

ELSE
    stateId := 0;
END_CASE;

// ============================================================================
//  ONE-SHOT & DIAGNOSTIC UPDATES
// ============================================================================
	
	IF pulseRunCmd THEN
	    gAxes.RunCmd := FALSE;
	    tRunCmdPulse(IN := TRUE, PT := PULSE_PT);
	    IF tRunCmdPulse.Q THEN
	        tRunCmdPulse(IN := FALSE);
	        pulseRunCmd := FALSE;
	    END_IF;
	ELSE
	    tRunCmdPulse(IN := FALSE);
	END_IF;
	
	tOneShotRearm(IN := NOT gAxes.RunCmd, PT := T#250MS);
	IF tOneShotRearm.Q AND NOT pulseRunCmd THEN
	    runCmdPrev := FALSE;
	    resetHoldLatched := FALSE;
	END_IF;
	
	prevStateId := stateId;
	gAxes.StateID := stateId;
	gAxes.StateText := stateText;
	gAxes.HysteresisSwitchEnabled := allowHysteresisSwitch;

