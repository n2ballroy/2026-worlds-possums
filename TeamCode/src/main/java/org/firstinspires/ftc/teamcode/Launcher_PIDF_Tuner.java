package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Automatic PIDF Tuner for the launcher velocity controller.
 *
 * Press START to run a full 4-phase auto-tune sequence. The robot must be
 * stationary with the launchers free to spin. No drive inputs are used.
 *
 * ── Algorithm overview ──────────────────────────────────────────────────────
 *
 * Phase 1 — F (feedforward): P=I=D=0. Spin up to near-setpoint, settle, then
 *   measure steady-state RPM. Adjust F proportionally (70 % damped) until RPM
 *   lands within 3 % of setpoint. Repeats up to 7 iterations.
 *
 * Phase 2 — P (proportional): Sweep P through a geometric ladder. For each
 *   candidate: spin down → wait until RPM enters the ±150 RPM band → measure
 *   2 s of data. Score = stdDev + 0.3 × overshoot. Stop when oscillation or
 *   >20 % overshoot is detected; keep the last candidate with the best score.
 *
 * Phase 3 — D (derivative): Skipped if overshoot < 5 % of setpoint AND
 *   stdDev < 80 RPM. Same spin-up-to-band approach as Phase 2. Keep the
 *   candidate that minimises overshoot + 0.5 × stdDev. Stop if D makes things
 *   worse by >30 % or induces oscillation.
 *
 * Phase 4 — I (integral): Always runs. F is reduced to 85 % to simulate a
 *   partially-discharged battery. A baseline measurement (I=0, stressed F)
 *   captures the resulting steady-state error; from it a dynamic minimum I is
 *   derived — the I required to correct 90 % of that error in 2 seconds.
 *   I is then swept from that floor upward; the best (lowest |error|) stable
 *   value is kept. If every I at the current P oscillates, P is stepped down
 *   through the sweep table and the I sweep repeats. Optimal F is always
 *   restored; P may end up lower than its Phase 2 value.
 *
 * After all phases: final step-response verification, then the tuned values
 * are displayed continuously. Copy the values into Demo_teleop.
 *
 * ── Oscillation detection ───────────────────────────────────────────────────
 *   stdDev > 200 RPM over the measurement window, OR
 *   hysteresis band crossings ≥ 4 within that window.
 *   A band crossing is counted only when RPM traverses the full ±150 RPM band
 *   (3000 ± 150 RPM), so ≤50 RPM quantization jitter cannot generate false
 *   positives.
 */
@TeleOp(name = "Launcher_PIDF_Tuner", group = "Demo")
//@Disabled
public class Launcher_PIDF_Tuner extends LinearOpMode {

    // ── PIDF coefficients under test (start from Demo_teleop values) ─────────
    private double tuneP = 25.0;
    private double tuneI =  0.1;
    private double tuneD =  1.0;
    private double tuneF = 12.0;

    // ── Test target ──────────────────────────────────────────────────────────
    private static final double TEST_RPM      = 3000.0;
    private static final double TICKS_PER_REV = 28.0;

    // ── Candidate sweep tables ───────────────────────────────────────────────
    private static final double[] P_SWEEP = {1, 2, 4, 8, 12, 16, 20, 25, 32, 40, 50, 65, 80, 100};
    private static final double[] D_SWEEP = {0.05, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0};
    private static final double[] I_SWEEP = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2};

    // ── Detection thresholds ─────────────────────────────────────────────────
    private static final double F_ERR_DONE_PCT    =   3.0;  // % of setpoint
    private static final double OSC_STDDEV         = 200.0;  // RPM std-dev threshold
    private static final double OSC_BAND_RPM       = 150.0;  // hysteresis half-band (> 50 RPM quantization)
    private static final double OSC_BAND_CROSSINGS =   4;    // full ±band crossings / window = oscillating
    private static final double OVERSHOOT_LIMIT_PCT=  20.0;  // % of setpoint
    private static final double D_NEEDED_OVER_RPM  =  TEST_RPM * 0.05;  // 5 % = 150 RPM
    private static final double D_NEEDED_STDDEV    =  80.0;
    // I tuning: reduce F to simulate a partially-discharged battery, then sweep I.
    // If no I value is stable at the current P, P is stepped down and the sweep repeats.
    // Optimal F is always restored in the final output; P may be reduced.
    private static final double BATTERY_STRESS_FACTOR  = 0.85;  // ~15 % voltage drop
    private static final double CORRECTION_TARGET_SEC  = 2.0;   // I must correct stress error within this many seconds

    // ── Hardware ─────────────────────────────────────────────────────────────
    private DcMotorEx rightLauncher;
    private DcMotorEx leftLauncher;
    private double launcherRPMActual = 0;

    // ── Telemetry ─────────────────────────────────────────────────────────────
    private String phaseLabel   = "";
    private String stepLabel    = "";
    private String latestMetric = "";

    // ── Final results ─────────────────────────────────────────────────────────
    private double finalSteadyErr = 0, finalStdDev = 0, finalOvershoot = 0;

    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {
        initMotors();

        phaseLabel = "Ready — press START to auto-tune";
        stepLabel  = "";
        while (!isStarted() && !isStopRequested()) {
            updateTelemetry();
            sleep(50);
        }
        if (isStopRequested()) { shutdown(); return; }

        // ── Phase 1: Feedforward ─────────────────────────────────────────────
        phaseLabel = "Phase 1 / 4 — Feedforward (F)  P=I=D=0";
        tuneP = 0; tuneI = 0; tuneD = 0;
        phaseTuneF();
        if (!opModeIsActive()) { shutdown(); return; }

        // ── Phase 2: Proportional ────────────────────────────────────────────
        phaseLabel = "Phase 2 / 4 — Proportional (P)";
        phaseTuneP();
        if (!opModeIsActive()) { shutdown(); return; }

        // ── Phase 3: Derivative ──────────────────────────────────────────────
        phaseLabel = "Phase 3 / 4 — Derivative (D)";
        phaseTuneD();
        if (!opModeIsActive()) { shutdown(); return; }

        // ── Phase 4: Integral ────────────────────────────────────────────────
        phaseLabel = "Phase 4 / 4 — Integral (I)";
        phaseTuneI();
        if (!opModeIsActive()) { shutdown(); return; }

        // ── Final verification step-response ─────────────────────────────────
        phaseLabel  = "Final verification…";
        stepLabel   = "Step response with tuned PIDF";
        latestMetric = "";
        applyPIDF();
        spinDown(1000);
        spinUpToWindow();
        Stats fin = measureWindow(3.0);
        finalSteadyErr = fin.mean - TEST_RPM;
        finalStdDev    = fin.stdDev;
        finalOvershoot = Math.max(0, fin.max - TEST_RPM);

        // Hold results on screen until operator stops
        phaseLabel = "TUNING COMPLETE";
        stepLabel  = "";
        while (opModeIsActive()) {
            updateRPM();
            showResults();
            sleep(50);
        }

        shutdown();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Phase 1 — F
    // ═════════════════════════════════════════════════════════════════════════

    private void phaseTuneF() {
        for (int iter = 0; iter < 7 && opModeIsActive(); iter++) {
            applyPIDF();
            stepLabel    = "iteration " + (iter + 1) + " / 7   F = " + String.format("%.4f", tuneF);
            latestMetric = "";

            spinDown(700);
            if (!opModeIsActive()) return;
            spinUp(5.0);
            if (!opModeIsActive()) return;

            Stats s      = measureWindow(2.0);
            double errPct = s.mean > 0 ? (s.mean - TEST_RPM) / TEST_RPM * 100.0 : -100;
            latestMetric = String.format("mean=%.0f  err=%+.1f%%  stdDev=%.0f",
                    s.mean, errPct, s.stdDev);

            if (Math.abs(errPct) < F_ERR_DONE_PCT) {
                stepLabel = "F converged at " + String.format("%.4f", tuneF);
                break;
            }

            // Proportional correction with 70 % damping (avoids oscillation across iterations)
            if (s.mean > 50) {
                double ratio      = TEST_RPM / s.mean;
                double dampedCorr = 1.0 + (ratio - 1.0) * 0.70;
                tuneF = Math.max(0.5, Math.min(150.0, tuneF * dampedCorr));
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Phase 2 — P
    // ═════════════════════════════════════════════════════════════════════════

    private void phaseTuneP() {
        double bestP     = 0;
        double bestScore = Double.MAX_VALUE;

        for (double candP : P_SWEEP) {
            if (!opModeIsActive()) return;

            tuneP     = candP;
            stepLabel = "testing P = " + String.format("%.4f", candP);
            applyPIDF();

            spinDown(400);
            if (!opModeIsActive()) return;
            spinUpToWindow();
            if (!opModeIsActive()) return;

            Stats s        = measureWindow(2.0);
            double overshoot = Math.max(0, s.max - TEST_RPM);
            double overPct   = overshoot / TEST_RPM * 100.0;
            boolean osc      = isOscillating(s);
            double score     = s.stdDev + overshoot * 0.3;

            latestMetric = String.format(
                    "P=%.0f  err=%+.0f  stdDev=%.0f  over=%.0f  bandX=%d",
                    candP, s.mean - TEST_RPM, s.stdDev, overshoot, s.bandCrossings);

            if (osc || overPct > OVERSHOOT_LIMIT_PCT) {
                break;  // crossed the line — use last good value
            }
            if (score < bestScore) {
                bestScore = score;
                bestP     = candP;
            }
        }

        tuneP     = Math.max(1.0, bestP);
        stepLabel = "P settled at " + String.format("%.4f", tuneP);
        applyPIDF();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Phase 3 — D
    // ═════════════════════════════════════════════════════════════════════════

    private void phaseTuneD() {
        // Baseline: measure overshoot/stdDev with current P, D=0
        tuneD = 0;
        applyPIDF();
        spinDown(400);
        if (!opModeIsActive()) return;
        spinUpToWindow();
        if (!opModeIsActive()) return;
        Stats baseline = measureWindow(2.0);
        double baseOver = Math.max(0, baseline.max - TEST_RPM);

        if (baseOver < D_NEEDED_OVER_RPM && baseline.stdDev < D_NEEDED_STDDEV) {
            stepLabel = String.format("D not needed  (over=%.0f RPM  stdDev=%.0f)", baseOver, baseline.stdDev);
            latestMetric = "";
            return;
        }

        double bestD     = 0;
        double bestScore = baseOver + baseline.stdDev * 0.5;

        for (double candD : D_SWEEP) {
            if (!opModeIsActive()) return;

            tuneD     = candD;
            stepLabel = "testing D = " + String.format("%.4f", candD);
            applyPIDF();

            spinDown(400);
            if (!opModeIsActive()) return;
            spinUpToWindow();
            if (!opModeIsActive()) return;

            Stats s        = measureWindow(2.0);
            double overshoot = Math.max(0, s.max - TEST_RPM);
            boolean osc      = isOscillating(s);
            double score     = overshoot + s.stdDev * 0.5;

            latestMetric = String.format(
                    "D=%.3f  err=%+.0f  stdDev=%.0f  over=%.0f  bandX=%d",
                    candD, s.mean - TEST_RPM, s.stdDev, overshoot, s.bandCrossings);

            if (osc) break;
            if (score < bestScore) {
                bestScore = score;
                bestD     = candD;
            } else if (score > bestScore * 1.3) {
                break;  // clearly getting worse
            }
        }

        tuneD     = bestD;
        stepLabel = "D settled at " + String.format("%.4f", tuneD);
        applyPIDF();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Phase 4 — I
    // ═════════════════════════════════════════════════════════════════════════

    private void phaseTuneI() {
        // Sequence:
        //   1. Apply stressed F (85 % of optimal) with I=0 and measure the resulting
        //      steady-state error.  Use that measurement to derive a dynamic minimum I —
        //      the I needed to correct the stress-induced offset within CORRECTION_TARGET_SEC.
        //   2. Sweep I from dynamicMinI upward with the current P.
        //      Take the best (lowest error) stable I found.
        //   3. If every I tried oscillated, step P down to the next lower P_SWEEP
        //      candidate and repeat the I sweep.
        //   4. Restore optimal F.  The found P (possibly reduced) and I are kept.

        double normalF   = tuneF;
        double stressedF = tuneF * BATTERY_STRESS_FACTOR;
        double originalP = tuneP;

        // ── Step 1: baseline measurement at stressed F, I=0 ─────────────────────
        tuneI = 0;
        tuneF = stressedF;
        applyPIDF();
        stepLabel    = String.format("baseline: F×%.0f%%  I=0  measuring stress error…", BATTERY_STRESS_FACTOR * 100);
        latestMetric = "";
        spinDown(500);
        if (!opModeIsActive()) { tuneF = normalF; return; }
        spinUpToWindow();
        if (!opModeIsActive()) { tuneF = normalF; return; }
        Stats baseline = measureWindow(2.0);

        // Dynamic minimum I derivation:
        //   powerDeficit  = (F_optimal − F_stressed) × velocity_ticks
        //                 = F × (1 − BATTERY_STRESS_FACTOR) × (TEST_RPM × TICKS/60)
        //   stressErrTicks = |measured_RPM_error| × TICKS/60
        //   dynamicMinI   = 0.9 × powerDeficit / (stressErrTicks × CORRECTION_TARGET_SEC)
        // The 0.9 factor targets 90 % correction so we stop just short of overshoot.
        // If the measured error is negligibly small (motor hardware surprises us), fall back to 0.05.
        double tpsTarget   = TEST_RPM * TICKS_PER_REV / 60.0;
        double powerDeficit = normalF * (1.0 - BATTERY_STRESS_FACTOR) * tpsTarget;
        double stressErrTicks = Math.abs(baseline.mean - TEST_RPM) * TICKS_PER_REV / 60.0;
        double dynamicMinI = (stressErrTicks > 1.0)
                ? 0.9 * powerDeficit / (stressErrTicks * CORRECTION_TARGET_SEC)
                : 0.05;  // fallback: error too small to derive meaningful I

        latestMetric = String.format("stressErr=%.0f RPM  powerDef=%.1f  minI=%.5f",
                baseline.mean - TEST_RPM, powerDeficit, dynamicMinI);

        // ── Step 2+3: sweep I ────────────────────────────────────────────────────

        // Build P candidates: current tuneP first, then descending through P_SWEEP
        double[] pCandidates = pCandidatesAtOrBelow(tuneP);

        double foundI = dynamicMinI;
        boolean stableFound = false;

        outer:
        for (double candP : pCandidates) {
            if (!opModeIsActive()) break;

            tuneP = candP;
            tuneF = stressedF;

            double bestI   = 0;
            double bestErr = Double.MAX_VALUE;
            boolean anyStable = false;

            for (double candI : I_SWEEP) {
                if (candI < dynamicMinI) continue;   // enforce dynamic floor
                if (!opModeIsActive()) break outer;

                tuneI = candI;
                applyPIDF();
                stepLabel = String.format("P=%.0f  I=%.5f  (F×%.0f%%)",
                        candP, candI, BATTERY_STRESS_FACTOR * 100);

                spinDown(500);
                if (!opModeIsActive()) break outer;
                spinUpToWindow();
                if (!opModeIsActive()) break outer;
                Stats s = measureWindow(3.0);

                double err  = Math.abs(s.mean - TEST_RPM);
                boolean osc = isOscillating(s);
                latestMetric = String.format("P=%.0f  I=%.4f  err=%+.0f  stdDev=%.0f  bandX=%d",
                        candP, candI, s.mean - TEST_RPM, s.stdDev, s.bandCrossings);

                if (!osc) {
                    anyStable = true;
                    if (err < bestErr) { bestI = candI; bestErr = err; }
                } else if (anyStable) {
                    break;   // was getting better, now oscillating — stop here
                } else {
                    break;   // oscillated immediately — try lower P
                }
            }

            if (anyStable) {
                foundI      = bestI;
                stableFound = true;
                break;
            }

            stepLabel = String.format(
                    "P=%.0f unstable for all I≥%.5f — reducing P", candP, dynamicMinI);
            latestMetric = "";
        }

        // Restore optimal F; keep found P and I (P may be lower than originalP)
        tuneF = normalF;
        tuneI = stableFound ? foundI : dynamicMinI;
        if (!stableFound) {
            // Extreme fallback: halve original P, accept dynamicMinI
            tuneP = originalP * 0.5;
            latestMetric = String.format("fallback: P halved, I=minI(%.5f)", dynamicMinI);
        }
        applyPIDF();

        stepLabel = String.format("I=%.5f  P=%.4f  (F restored)", tuneI, tuneP);
    }

    /**
     * Returns an array starting with maxP, followed by all P_SWEEP values strictly
     * below maxP in descending order — so the caller can try each in turn.
     */
    private double[] pCandidatesAtOrBelow(double maxP) {
        int count = 1;
        for (double p : P_SWEEP) if (p < maxP) count++;
        double[] out = new double[count];
        out[0] = maxP;
        int idx = 1;
        for (int i = P_SWEEP.length - 1; i >= 0; i--)
            if (P_SWEEP[i] < maxP) out[idx++] = P_SWEEP[i];
        return out;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Motor helpers
    // ═════════════════════════════════════════════════════════════════════════

    private void initMotors() {
        rightLauncher = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
        leftLauncher  = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
        rightLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorEx.Direction.FORWARD);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        applyPIDF();
    }

    private void applyPIDF() {
        rightLauncher.setVelocityPIDFCoefficients(tuneP, tuneI, tuneD, tuneF);
        leftLauncher.setVelocityPIDFCoefficients( tuneP, tuneI, tuneD, tuneF);
    }

    /** Coast to zero, wait ms, keep telemetry alive. */
    private void spinDown(long ms) {
        rightLauncher.setVelocity(0);
        leftLauncher.setVelocity(0);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) {
            updateRPM();
            updateTelemetry();
            sleep(20);
        }
    }

    /**
     * F-phase only: command TEST_RPM and wait until actual ≥ 95 % or timeout,
     * then add a 0.5 s settle margin so the mean measurement reflects true steady state.
     */
    private void spinUp(double maxWaitSec) {
        double tps = TEST_RPM * TICKS_PER_REV / 60.0;
        rightLauncher.setVelocity(tps);
        leftLauncher.setVelocity(tps);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < maxWaitSec) {
            updateRPM();
            updateTelemetry();
            if (launcherRPMActual >= TEST_RPM * 0.95) break;
            sleep(20);
        }
        ElapsedTime settle = new ElapsedTime();
        while (opModeIsActive() && settle.seconds() < 0.5) {
            updateRPM();
            updateTelemetry();
            sleep(20);
        }
    }

    /**
     * P / D / I / final phases: command TEST_RPM and return as soon as RPM enters
     * the ±OSC_BAND_RPM window around the setpoint.  Measurement starts immediately —
     * no fixed post-settle margin.  If the motor oscillates and never settles into the
     * band the timeout (5 s) prevents an infinite wait; the measurement window will
     * then capture the oscillation anyway.
     */
    private void spinUpToWindow() {
        double tps = TEST_RPM * TICKS_PER_REV / 60.0;
        rightLauncher.setVelocity(tps);
        leftLauncher.setVelocity(tps);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < 5.0) {
            updateRPM();
            updateTelemetry();
            if (Math.abs(launcherRPMActual - TEST_RPM) <= OSC_BAND_RPM){
                sleep(100);
                break;
            }
            sleep(20);
        }
    }

    /**
     * Collect RPM samples for durationSec at ~50 Hz.
     * Returns Stats over the full window.
     */
    private Stats measureWindow(double durationSec) {
        int maxN    = (int)(durationSec * 55) + 10;
        double[] buf = new double[maxN];
        int n = 0;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < durationSec && n < maxN) {
            updateRPM();
            buf[n++] = launcherRPMActual;
            updateTelemetry();
            sleep(18);
        }
        return computeStats(buf, n);
    }

    private void updateRPM() {
        double raw = rightLauncher.getVelocity() * 60.0 / TICKS_PER_REV;
        launcherRPMActual = 0.15 * raw + 0.85 * launcherRPMActual;
    }

    private void shutdown() {
        rightLauncher.setVelocity(0);
        leftLauncher.setVelocity(0);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Stats
    // ═════════════════════════════════════════════════════════════════════════

    private static class Stats {
        double mean = 0, stdDev = 0, max = Double.NEGATIVE_INFINITY, min = Double.MAX_VALUE;
        int bandCrossings = 0;  // full ±band traversals (hysteresis — immune to quantization jitter)
    }

    private Stats computeStats(double[] data, int n) {
        Stats s = new Stats();
        if (n == 0) return s;

        double sum = 0;
        for (int i = 0; i < n; i++) {
            sum   += data[i];
            s.max  = Math.max(s.max, data[i]);
            s.min  = Math.min(s.min, data[i]);
        }
        s.mean = sum / n;

        double sumSq = 0;
        for (int i = 0; i < n; i++) {
            double d = data[i] - s.mean;
            sumSq += d * d;
        }
        s.stdDev = Math.sqrt(sumSq / n);

        // Hysteresis band crossing count.
        // State: 0 = inside band, +1 = confirmed above upper edge, -1 = confirmed below lower edge.
        // A crossing is only counted when the signal exits the OPPOSITE edge from the last exit.
        // This means the RPM had to traverse the full 2×OSC_BAND_RPM width, so quantization
        // jitter (≤50 RPM) around the setpoint can never generate false crossings.
        double upper = TEST_RPM + OSC_BAND_RPM;
        double lower = TEST_RPM - OSC_BAND_RPM;
        int bandState = 0;  // start: unknown / inside
        for (int i = 0; i < n; i++) {
            double rpm = data[i];
            if (rpm > upper) {
                if (bandState == -1) s.bandCrossings++;  // came from below-lower to above-upper
                bandState = +1;
            } else if (rpm < lower) {
                if (bandState == +1) s.bandCrossings++;  // came from above-upper to below-lower
                bandState = -1;
            }
            // if inside band: state unchanged, no crossing recorded
        }
        return s;
    }

    /** True if the measurement window shows oscillation. */
    private boolean isOscillating(Stats s) {
        return s.stdDev > OSC_STDDEV || s.bandCrossings >= OSC_BAND_CROSSINGS;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Telemetry
    // ═════════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        telemetry.addLine(phaseLabel);
        if (!stepLabel.isEmpty())    telemetry.addLine("  " + stepLabel);
        if (!latestMetric.isEmpty()) telemetry.addLine("  " + latestMetric);
        telemetry.addLine();
        telemetry.addData("RPM Actual", "%.0f  /  %.0f target", launcherRPMActual, TEST_RPM);
        telemetry.addData("RPM Error",  "%+.0f", launcherRPMActual - TEST_RPM);
        telemetry.addLine();
        telemetry.addData("F", String.format("%.4f", tuneF));
        telemetry.addData("P", String.format("%.4f", tuneP));
        telemetry.addData("I", String.format("%.5f", tuneI));
        telemetry.addData("D", String.format("%.4f", tuneD));
        telemetry.update();
    }

    private void showResults() {
        telemetry.addLine("══════════ TUNING COMPLETE ══════════");
        telemetry.addLine("Copy these values into Demo_teleop:");
        telemetry.addLine();
        telemetry.addData("LAUNCHER_VEL_F", String.format("%.4f", tuneF));
        telemetry.addData("LAUNCHER_VEL_P", String.format("%.4f", tuneP));
        telemetry.addData("LAUNCHER_VEL_I", String.format("%.5f", tuneI));
        telemetry.addData("LAUNCHER_VEL_D", String.format("%.4f", tuneD));
        telemetry.addLine();
        telemetry.addLine("── Final step-response metrics ──");
        telemetry.addData("Steady-state error", "%+.0f RPM", finalSteadyErr);
        telemetry.addData("Std dev (noise)",    "%.0f RPM",  finalStdDev);
        telemetry.addData("Overshoot",          "%.0f RPM",  finalOvershoot);
        telemetry.addLine();
        telemetry.addData("Live RPM",   "%.0f", launcherRPMActual);
        telemetry.addData("Live Error", "%+.0f", launcherRPMActual - TEST_RPM);
        telemetry.update();
    }

}
