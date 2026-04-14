package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.PossumsConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Match7_pedro_auto — BLUE alliance, configurable step sequence.
 *
 * Changes from Match6:
 *  - All field coordinates come from FieldConstants (inches, audience-wall-corner origin).
 *  - All robot geometry comes from RobotConstants (inches).
 *  - Chute outlet is the robot reference point (0,0).
 *  - Camera pose seeding: averaged X/Y from limelight is compared against 4 predefined
 *    start poses; the closest pose is used to seed pinpoint (no std-dev gating needed).
 *  - Turret servo: always rotates toward PedroFieldConstants.SHOOT_TARGET_X/Y.
 *    Turret encoder resets to 0 at match start (robot pre-positioned facing forward).
 *
 * PRE-START MENU (gamepad1):
 *   DPAD UP / DOWN    — select which step row to edit
 *   DPAD RIGHT / LEFT — cycle the action for the selected step
 *   A button          — confirm the selected step
 *   START             — lock in sequence and proceed to limelight seeding
 */
//@TeleOp(name = "Match7_Pedro_Auto", group = "Match")
@Autonomous(name = "Possums Match7_Blue_Pedro_Auto", preselectTeleOp = "MainTeleOpBlueAutoPinpoint", group = "Match")
public class Possums_Match7_Blue_pedro_auto extends LinearOpMode {

    private static final double BALL_LINE_X_MARGIN_IN = 9.0;

    private static final double INTAKE_WALL_CLEARANCE_IN = 0.5;

    private static final double BALL_LINE_CLEAR_X =
              PedroFieldConstants.FLOOR_BALLS_BLUE_X
            + BALL_LINE_X_MARGIN_IN
            + PedroRobotConstants.CHUTE_TO_FRONT_IN;   // 32 + 2 + 14.01 = 48.01

    // Chute X at end of intake sweep: intake is WALL_CLEARANCE_IN from the X=0 side wall
    private static final double BALL_PICKUP_COMPLETE_X = PedroRobotConstants.CHUTE_TO_FRONT_IN + INTAKE_WALL_CLEARANCE_IN;  // 14.01 + 0.1 = 14.11

    private static final double SHOOT_TRANSFER_SEC      =    3.0;

    private static final double INTAKE_POWER            =    1.0;
    private static final double DRIVE_TIMEOUT_SEC       =    5.0;  //if pedro path not done within 5 seconds then continue anyway
    private static final double BALL_LINE_MAX_POWER     =    0.5;  //this slows the robot way down when picking balls max=1.0

    // =====================================================================
    //  STEP CONFIGURATION
    // =====================================================================
    private static final int MAX_STEPS = 10;

    private enum StepAction { LINE_1, LINE_2, LINE_3, GATE, CORNER, TWELVE_BALL, DONE }

    private static final StepAction[] STEP_OPTIONS      = StepAction.values();
    private static final String[]     STEP_OPTION_NAMES =
            { "Line 1", "Line 2", "Line 3", "Gate Lever", "Corner", "12 Ball", "DONE" };

    private int           currentMenuStepIndex = 0;
    private List<Integer> stepOptionIndex      = new ArrayList<>();



    // =====================================================================
    //  HARDWARE
    // =====================================================================
    private DcMotorEx       intake;
    private CRServo         transfer;
    private DcMotorEx       rightLauncher, leftLauncher;
    private DcMotorEx       turretMotor;
    private Limelight3A     limelight;

    // Pedro Pathing follower — owns drive motors and pinpoint
    private Follower follower;

    // =====================================================================
    //  POSE (read from Pedro)
    // =====================================================================
    private double robotX, robotY, robotHeading;
    private double initialRobotY = 0;

    // =====================================================================
    //  TURRET
    // =====================================================================
    private double turretAngleDeg = 0;

    // =====================================================================
    //  LAUNCHER
    // =====================================================================
    private double launcherVelocityCmd = 0;  // ticks/sec

    // =====================================================================
    //  TIMING / STATE
    // =====================================================================
    private long        startingTimeMsec       = 0;
    private double      startingBatteryVoltage = 0;
    private ElapsedTime delayTimer             = new ElapsedTime();
    private boolean     isDelayRunning         = false;
    private ElapsedTime rpmSettleTimer         = new ElapsedTime();
    private boolean     rpmSettleTimerWasReset = true;

    // =====================================================================
    //  SEQUENCE STATE
    // =====================================================================
    private int autoPhase        = 0;
    private int autoSubStep      = 0;
    private int shootSubStep     = 0;
    private int currentStepIndex = 0;

    // =====================================================================
    //  LIMELIGHT / POSE SEEDING
    // =====================================================================
    private double  pedroX = 0, pedroY = 0, pedroHeading = 0;
    private boolean pedroHasValidPose = false;

    // =====================================================================
    //  runOpMode
    // =====================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        stepOptionIndex.add(STEP_OPTIONS.length - 1);  // start with DONE

        initializeHardware();

        try {
            if (isStopRequested()) return;

            for (int i = 0; i < 10; i++) {
                startingBatteryVoltage += hardwareMap.voltageSensor.iterator().next().getVoltage();
            }
            startingBatteryVoltage /= 10.0;

            runStepSelectionMenu();
            if (isStopRequested()) return;

            if (!isStarted()) {
                seedPoseFromLimelight();
                if (isStopRequested()) return;
            }

            while (!isStarted() && !isStopRequested()) {
                follower.update();
                readPoseFromFollower();
                displayTelemetry();
            }
            if (isStopRequested()) return;

            waitForStart();
            turretMotor.setPower(1.0);  // enable turret only after match start
            startingTimeMsec = System.currentTimeMillis();
            initialRobotY    = robotY;
            launcherVelocityCmd = 1;  // any positive value enables the launcher; updateLauncher() computes actual velocity from distance
            limelight.stop();

            while (opModeIsActive()) {
                follower.update();
                readPoseFromFollower();
                updateTurret();
                if (autoPhase < 99) updateLauncher();
                runAutonomousSequence();
                displayTelemetry();
            }
        } finally {
            stopRobot();
        }
    }

    // =====================================================================
    //  PRE-START MENU
    // =====================================================================
    private void runStepSelectionMenu() {
        boolean upPrev = false, downPrev = false,
                rightPrev = false, leftPrev = false, startPrev = false;

        while (!isStarted() && !isStopRequested()) {
            boolean up    = gamepad1.dpad_up;
            boolean down  = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;
            boolean left  = gamepad1.dpad_left;
            boolean start = gamepad1.start;

            if (start && !startPrev) break;

            if (down && !downPrev) {
                if (currentMenuStepIndex < stepOptionIndex.size() - 1) {
                    currentMenuStepIndex++;
                } else if (stepOptionIndex.get(currentMenuStepIndex) != STEP_OPTIONS.length - 1
                        && stepOptionIndex.size() < MAX_STEPS) {
                    stepOptionIndex.add(STEP_OPTIONS.length - 1);
                    currentMenuStepIndex++;
                }
            }
            if (up && !upPrev) {
                currentMenuStepIndex = Math.max(0, currentMenuStepIndex - 1);
            }
            if (right && !rightPrev) {
                int cur = stepOptionIndex.get(currentMenuStepIndex);
                stepOptionIndex.set(currentMenuStepIndex, (cur + 1) % STEP_OPTIONS.length);
            }
            if (left && !leftPrev) {
                int cur = stepOptionIndex.get(currentMenuStepIndex);
                stepOptionIndex.set(currentMenuStepIndex,
                        (cur + STEP_OPTIONS.length - 1) % STEP_OPTIONS.length);
            }

            upPrev = up; downPrev = down; rightPrev = right; leftPrev = left;
            startPrev = start;

            telemetry.addLine("=== AUTO STEP CONFIGURATION  (BLUE alliance) ===");
            telemetry.addLine("UP/DOWN: select step | RIGHT/LEFT: change action");
            telemetry.addLine("Gamepad START: lock in sequence | DS START: begin match");
            telemetry.addLine();
            for (int i = 0; i < stepOptionIndex.size(); i++) {
                String arrow = (i == currentMenuStepIndex) ? "-->" : "   ";
                telemetry.addData(arrow + " Step " + (i + 1),
                        STEP_OPTION_NAMES[stepOptionIndex.get(i)]);
            }
            telemetry.update();
        }

        // Expand TWELVE_BALL preset into its fixed sub-sequence
        List<Integer> expanded = new ArrayList<>();
        for (int idx : stepOptionIndex) {
            if (STEP_OPTIONS[idx] == StepAction.TWELVE_BALL) {
                expanded.add(StepAction.LINE_1.ordinal());
                expanded.add(StepAction.LINE_2.ordinal());
                expanded.add(StepAction.GATE.ordinal());
                expanded.add(StepAction.LINE_3.ordinal());
            } else {
                expanded.add(idx);
            }
        }
        stepOptionIndex = expanded;
    }

    // =====================================================================
    //  AUTONOMOUS SEQUENCE DISPATCHER
    // =====================================================================
    private void runAutonomousSequence() {
        if (!opModeIsActive()) return;

        switch (autoPhase) {

            case 0:  // drive to initial shoot position (far start only; near start shoots in place)
                if (initialRobotY >= 72.0) {  //if start on audience wall then skip the initial move and just aim the turret
                    double[] shootXYH = blueShootPose();
                    driveToPose(shootXYH[0], shootXYH[1], shootXYH[2], false);
                }
                shootSubStep   = 0;
                isDelayRunning = false;
                autoPhase      = 1;
                break;

            case 1:  // shoot pre-loaded balls
                if (executeShootSequence()) {
                    currentStepIndex = 0;
                    autoSubStep      = 0;
                    isDelayRunning   = false;
                    autoPhase        = 2;
                }
                break;

            case 2:  // execute configured steps
                if (isStopRequested()) return;
                if (currentStepIndex >= stepOptionIndex.size()) { autoPhase = 99; break; }
                StepAction action = STEP_OPTIONS[stepOptionIndex.get(currentStepIndex)];
                if (action == StepAction.DONE) { autoPhase = 99; break; }
                boolean stepDone;
                if (action == StepAction.GATE) {
                    stepDone = executeGateStep();
                } else {
                    stepDone = executeBallLineAndShootStep(action);
                }
                if (stepDone) {
                    currentStepIndex++;
                    autoSubStep    = 0;
                    isDelayRunning = false;
                }
                break;

            default:
                stopRobot();
                autoPhase = 1000;
                break;
        }
    }

    // =====================================================================
    //  SHOOT SEQUENCE
    // =====================================================================
    private boolean executeShootSequence() {
        switch (shootSubStep) {
            case 0:
                // Single timer starts when robot stops. Fire as soon as RPM and turret
                // are both on-target, or shoot anyway when the timer expires.
                if ((rpmReadyToShoot() && turretAtTarget())
                        || nonBlockingDelay(PedroRobotConstants.MAX_RPM_TURRET_WAIT_SEC)) {
                    isDelayRunning = false;
                    intake.setPower(INTAKE_POWER);
                    transfer.setPower(1.0);
                    shootSubStep = 1;
                }
                return false;
            case 1:
                if (nonBlockingDelay(SHOOT_TRANSFER_SEC)) {
                    isDelayRunning = false;
                    intake.setPower(0);
                    transfer.setPower(0.0);
                    shootSubStep = 99;
                    return true;
                }
                return false;
            default:
                return true;
        }
    }

    // =====================================================================
    //  GATE STEP
    // =====================================================================
    private boolean executeGateStep() {
        double gateY = PedroFieldConstants.BLUE_GATE_Y - (PedroRobotConstants.CHUTE_TO_PUSHER_RIGHT_IN);
        switch (autoSubStep) {
            case 0:
                driveToPose(BALL_LINE_CLEAR_X, gateY, 180.0, false);
                autoSubStep = 1;
                return false;
            case 1:
                // Intake (heading 180°, facing -X) presses gate; pusher surface offset in Y via gateY
                driveToPose(
                        PedroFieldConstants.BLUE_GATE_X - 2.0 + PedroRobotConstants.CHUTE_TO_FRONT_IN,
                        gateY, 180.0, false);
                autoSubStep = 2;
                return false;
            case 2:
                if (nonBlockingDelay(2.0)) {
                    isDelayRunning = false;
                    autoSubStep = isLastActionableStep() ? 99 : 3;
                }
                return false;
            case 3:
                driveToPose(BALL_LINE_CLEAR_X, gateY, 180.0, false);
                autoSubStep = 99;
                return false;
            default:
                return true;
        }
    }

    // =====================================================================
    //  BALL LINE + SHOOT STEP
    // =====================================================================
    private boolean executeBallLineAndShootStep(StepAction line) {
        double lineY         = getBallLineY(line);
        double exitX         = BALL_PICKUP_COMPLETE_X;
        double pickupHeading = 180.0;

        switch (autoSubStep) {
            case 0:
                driveToPose(BALL_LINE_CLEAR_X, lineY, pickupHeading, false);
                autoSubStep = 1;
                return false;

            case 1:
                intake.setPower(INTAKE_POWER);
                driveToPoseSlow(exitX, lineY, pickupHeading, false);
                intake.setPower(0);
                autoSubStep = 2;
                return false;

            case 2:
                driveToPose(BALL_LINE_CLEAR_X, lineY, pickupHeading, false);
                autoSubStep = 3;
                return false;

            case 3:
                double[] shoot = blueShootPose();
                driveToPose(shoot[0], shoot[1], shoot[2], false);
                shootSubStep   = 0;
                isDelayRunning = false;
                autoSubStep    = 4;
                return false;

            case 4:
                if (executeShootSequence()) {
                    autoSubStep = 0;
                    return true;
                }
                return false;

            default:
                return true;
        }
    }

    // =====================================================================
    //  NAVIGATION — NORMAL SPEED
    // =====================================================================
    private void driveToPose(double toX, double toY, double headingDeg, boolean holdEnd) {
        followPathBlocking(buildPath(toX, toY, headingDeg), holdEnd, 1.0);
    }

    // =====================================================================
    //  NAVIGATION — SLOW SPEED  (for ball line intake pass)
    // =====================================================================
    private void driveToPoseSlow(double toX, double toY, double headingDeg, boolean holdEnd) {
        followPathBlocking(buildPath(toX, toY, headingDeg), holdEnd, BALL_LINE_MAX_POWER);
    }

    // =====================================================================
    //  PATH BUILDER
    // =====================================================================
    private Object buildPath(double toX, double toY, double headingDeg) {
        Pose   currentPose = follower.getPose();
        double fromX       = currentPose.getX();
        double fromY       = currentPose.getY();
        double fromH       = currentPose.getHeading();
        double toH         = Math.toRadians(headingDeg);

        Pose start = new Pose(fromX, fromY, fromH);
        Pose end   = new Pose(toX,   toY,   toH);
        Pose via   = null;

        if (via != null) {
            PathChain chain = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(start, via)))
                    .addPath(new Path(new BezierLine(via, end)))
                    .setGlobalLinearHeadingInterpolation(fromH, toH,0.6)  //0.6 means end the heading portion by 60% of the distance
                    .build();
            return chain;
        } else {
            Path path = new Path(new BezierLine(start, end));
            path.setLinearHeadingInterpolation(fromH, toH, .6);     //0.6 means end the heading portion by 60% of the distance
            return path;
        }
    }

    // =====================================================================
    //  BLOCKING FOLLOW — handles both Path and PathChain, with maxPower
    // =====================================================================
    private void followPathBlocking(Object pathOrChain, boolean holdEnd, double maxPower) {
        if (pathOrChain instanceof PathChain) {
            follower.followPath((PathChain) pathOrChain, maxPower, holdEnd);
        } else {
            follower.setMaxPower(maxPower);
            follower.followPath((Path) pathOrChain, holdEnd);
        }

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timeout.seconds() < DRIVE_TIMEOUT_SEC) {
            follower.update();
            readPoseFromFollower();
            updateTurret();
            updateLauncher();
            displayTelemetry();
        }

        if (maxPower < 1.0) follower.setMaxPower(1.0);
    }


    // =====================================================================
    //  TURRET CONTROL
    //  Computes the angle from the chute (robot position) toward the fixed
    //  shooting target, expressed in the robot frame (0 = front), and
    //  commands the turret motor (RUN_TO_POSITION) accordingly.
    //  Encoder 0 = forward (reset at match start).
    // =====================================================================
    private void updateTurret() {
        double dx = PedroFieldConstants.BLUE_SHOOT_TARGET_X - robotX;
        double dy = PedroFieldConstants.SHOOT_TARGET_Y - robotY;
        // Bearing to target in field frame (Pedro: 0 = right/+X, angles CCW-positive)
        double fieldBearingRad = Math.atan2(dy, dx);
        // Convert robot heading (degrees) to radians
        double headingRad = Math.toRadians(robotHeading);
        // Angle in robot frame: 0 = toward robot front
        double turretRad = fieldBearingRad - headingRad;
        // Normalize to [-PI, PI]
        turretRad      = Math.atan2(Math.sin(turretRad), Math.cos(turretRad));
        turretAngleDeg = Math.toDegrees(turretRad);
        // Convert to encoder ticks, clamp to physical limits, and command motor
        int targetTicks = (int) Math.min(Math.max(
                turretAngleDeg * PedroRobotConstants.TURRET_TICKS_PER_DEG, -1400), 1400);
        turretMotor.setTargetPosition(targetTicks);
    }

    // =====================================================================
    //  HELPERS
    // =====================================================================
    private boolean isLastActionableStep() {
        int next = currentStepIndex + 1;
        if (next >= stepOptionIndex.size()) return true;
        return STEP_OPTIONS[stepOptionIndex.get(next)] == StepAction.DONE;
    }

    private double getBallLineY(StepAction action) {
        switch (action) {
            case LINE_1: return PedroFieldConstants.BALL_LINE_1_Y;
            case LINE_2: return PedroFieldConstants.BALL_LINE_2_Y;
            case LINE_3: return PedroFieldConstants.BALL_LINE_3_Y;
            case CORNER: return PedroFieldConstants.CORNER_Y;
            default:     return PedroFieldConstants.BALL_LINE_2_Y;
        }
    }

    /** Returns {x, y, headingDeg} for the BLUE shoot position based on starting side. */
    private double[] blueShootPose() {
        if (initialRobotY < 72.0) {
            return PedroFieldConstants.BLUE_NEAR_SHOOT_POSE;
        } else {
            return PedroFieldConstants.BLUE_FAR_SHOOT_POSE;
        }
    }

    private void readPoseFromFollower() {
        Pose p   = follower.getPose();
        robotX   = p.getX();
        robotY   = p.getY();
        robotHeading = Math.toDegrees(p.getHeading());
    }

    // =====================================================================
    //  LIMELIGHT POSE SEEDING — snap to closest predefined start pose
    //
    //  1. Collect averaged X/Y samples from the camera (no std-dev gate).
    //  2. Convert to Pedro inches.
    //  3. Find the predefined start pose with the smallest XY distance.
    //  4. Seed Pedro with that pose.
    // =====================================================================
    private void seedPoseFromLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PedroRobotConstants.GOALS_PIPELINE);
        sleep(500);
        limelight.start();
        sleep(500);

        telemetry.addData("Camera", "Collecting pose samples...");
        telemetry.update();

        pollLimelight();
        //pedroHasValidPose=true;
        //pedroX=PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1[0];
        //pedroY=PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1[1];
        //pedroHeading=PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1[2];
        if (pedroHasValidPose) {
            follower.setPose(new Pose(pedroX, pedroY, Math.toRadians(pedroHeading)));
            readPoseFromFollower();
            telemetry.addData("Pose seeded",
                    "X=%.1f in  Y=%.1f in  H=%.1f deg", pedroX, pedroY, pedroHeading);
        } else {
            telemetry.addData("Camera", "No valid pose — Pedro starts at its current origin");
        }
        telemetry.update();
        sleep(500);
    }

    /**
     * Collects camera samples, averages X/Y, converts to Pedro inches,
     * then selects the closest predefined start pose to seed pinpoint.
     */
    private void pollLimelight() {
        final int TARGET_SAMPLES = 20;
        final int MAX_FAIL_COUNT = 10;
        final int POLL_SLEEP_MS  = 33;

        List<Double> xSamples = new ArrayList<>();
        List<Double> ySamples = new ArrayList<>();
        int    failCount      = 0;
        double lastTimestamp  = -1;

        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()
                    && result.getBotpose() != null
                    && result.getBotposeTagCount() > 0) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    if (result.getTimestamp() != lastTimestamp) {
                        lastTimestamp = result.getTimestamp();
                        Pose3D botpose = result.getBotpose();
                        xSamples.add(botpose.getPosition().x);
                        ySamples.add(botpose.getPosition().y);
                        failCount = 0;
                    }
                } else {
                    failCount++;
                    telemetry.addData("Poll fail", "(%d) No fiducials", failCount);
                    telemetry.update();
                }
            } else {
                failCount++;
                String reason = result == null             ? "null result"
                        : !result.isValid()               ? "isValid=false"
                        : result.getBotpose() == null      ? "botpose=null"
                        : "tagCount=" + result.getBotposeTagCount();
                telemetry.addData("Poll fail", "(%d) %s", failCount, reason);
                telemetry.update();
            }

            if (xSamples.size() >= TARGET_SAMPLES || failCount > MAX_FAIL_COUNT) break;
            sleep(POLL_SLEEP_MS);
        }

        telemetry.addData("Poll done", "samples=%d  fails=%d", xSamples.size(), failCount);
        telemetry.update();
        sleep(500);  // hold so result is readable

        if (xSamples.isEmpty()) {
            telemetry.addData("Limelight", "No samples — pose not set");
            telemetry.update();
            sleep(500);  // hold so failure reason is readable
            return;
        }

        // Drop outliers: remove any sample whose X or Y deviates more than
        // OUTLIER_THRESHOLD from the median before averaging.
        final double OUTLIER_THRESHOLD = 0.07; // meters (~6 in); tune if needed
        xSamples = filterOutliers(xSamples, OUTLIER_THRESHOLD);
        ySamples = filterOutliers(ySamples, OUTLIER_THRESHOLD);
        if (xSamples.isEmpty()) return;

        // Average filtered samples
        double avgX = 0, avgY = 0;
        for (double v : xSamples) avgX += v;
        for (double v : ySamples) avgY += v;
        avgX /= xSamples.size();
        avgY /= ySamples.size();

        // Convert limelight WCS (meters) to Pedro inches
        //   Pedro X = limelight_Y * 39.37 + 72
        //   Pedro Y = -limelight_X * 39.37 + 72
        double camPedroX = avgY * 39.37 + 72.0;
        double camPedroY = -avgX * 39.37 + 72.0;
        camPedroX=camPedroX+4;   //added this becuase camera was off a lot in X on OSM field
        // Find closest predefined start pose
        double[][] poses = {
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1,
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_2,
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_3,
            PedroFieldConstants.BLUE_BACK_WALL_START_POSE
        };

        double minDist   = Double.MAX_VALUE;
        double[] bestPose = null;
        //loop to fiind closest start point to camera reading
        for (double[] pose : poses) {
            double dist = Math.hypot(camPedroX - pose[0], camPedroY - pose[1]);
            if (dist < minDist) {
                minDist   = dist;
                bestPose  = pose;
            }
        }

        if (bestPose != null) {
            pedroX            = bestPose[0];
            pedroY            = bestPose[1];
            pedroHeading      = bestPose[2];
            pedroHasValidPose = true;
            telemetry.addData("Camera snap",
                    "cam=(%.1f, %.1f)  ->  pose=(%.1f, %.1f, %.1f deg)  dist=%.1f in",
                    camPedroX, camPedroY, pedroX, pedroY, pedroHeading, minDist);
            telemetry.addData("Samples used", xSamples.size());
            telemetry.update();
            sleep(500);
        }
    }

    /**
     * Returns a copy of {@code samples} with any value that deviates more than
     * {@code threshold} from the median removed.  If all values would be removed
     * the original list is returned unchanged so the caller always has data.
     */
    private List<Double> filterOutliers(List<Double> samples, double threshold) {
        if (samples.size() < 3) return samples;   // not enough data to judge

        List<Double> sorted = new ArrayList<>(samples);
        java.util.Collections.sort(sorted);
        double median = sorted.get(sorted.size() / 2);

        List<Double> filtered = new ArrayList<>();
        for (double v : samples) {
            if (Math.abs(v - median) <= threshold) filtered.add(v);
        }
        return filtered.isEmpty() ? samples : filtered;
    }

    // =====================================================================
    //  HARDWARE INIT
    // =====================================================================
    private void initializeHardware() {
        intake        = hardwareMap.get(DcMotorEx.class,  PedroRobotConstants.INTAKE_CONFIG_NAME);
        transfer      = hardwareMap.get(CRServo.class,    PedroRobotConstants.TRANSFER_SERVO_CONFIG_NAME);
        rightLauncher = hardwareMap.get(DcMotorEx.class,  PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
        leftLauncher  = hardwareMap.get(DcMotorEx.class,  PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
        turretMotor   = hardwareMap.get(DcMotorEx.class,  PedroRobotConstants.TURRET_MOTOR_CONFIG_NAME);

        transfer.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setVelocityPIDFCoefficients(150, 0, 0, 14);
        leftLauncher.setVelocityPIDFCoefficients(150, 0, 0, 14);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Turret motor — reset encoder (robot pre-positioned facing forward at match start)
        turretMotor.setPositionPIDFCoefficients(PedroRobotConstants.TURRET_PIDF);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setTargetPositionTolerance((int) PedroRobotConstants.TURRET_TICKS_PER_DEG); // ±1 deg
        // NOTE: setPower(1.0) intentionally deferred until after waitForStart()

        follower = PossumsConstants.createFollower(hardwareMap);
    }

    // =====================================================================
    //  LAUNCHER
    // =====================================================================
    private void updateLauncher() {
        if (launcherVelocityCmd <= 0) {
            rightLauncher.setVelocity(0);
            leftLauncher.setVelocity(0);
            return;
        }
        double distToGoal = Math.hypot(
                robotX - PedroFieldConstants.BLUE_SHOOT_TARGET_X,
                robotY - PedroFieldConstants.SHOOT_TARGET_Y);  // inches
        launcherVelocityCmd = 1160 + distToGoal * 4.5;  // ticks/sec, same formula as TeleOp
        rightLauncher.setVelocity(launcherVelocityCmd);
        leftLauncher.setVelocity(launcherVelocityCmd);
    }

    private void stopRobot() {
        follower.breakFollowing();
        launcherVelocityCmd = 0;
        rightLauncher.setVelocity(0);
        leftLauncher.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0.0);
        readPoseFromFollower();
        QuickOdometryStorage.x             = robotX;
        QuickOdometryStorage.y             = robotY;
        QuickOdometryStorage.heading       = robotHeading;
        QuickOdometryStorage.turretDegrees = turretAngleDeg;
        QuickOdometryStorage.alliance      = "BLUE";
        QuickOdometryStorage.valid         = true;
    }

    private boolean nonBlockingDelay(double sec) {
        if (!isDelayRunning) { delayTimer.reset(); isDelayRunning = true; return false; }
        if (delayTimer.seconds() < sec) return false;
        isDelayRunning = false;
        return true;
    }

    private boolean rpmReadyToShoot() {
        // 933 ticks/sec ≈ 2000 RPM; 47 ticks/sec ≈ 200 RPM tolerance
        return (launcherVelocityCmd > 933  && Math.abs(launcherVelocityCmd - rightLauncher.getVelocity()) <= 94);
    }

    private boolean turretAtTarget() {
        double actualDeg = turretMotor.getCurrentPosition() / PedroRobotConstants.TURRET_TICKS_PER_DEG;
        return Math.abs(actualDeg - turretAngleDeg) <= PedroRobotConstants.TURRET_ACCURACY_DEG;
    }

    private boolean rpmAccurate() {
        // 9 ticks/sec ≈ 20 RPM tolerance; 933 ticks/sec ≈ 2000 RPM minimum
        double err = Math.abs(launcherVelocityCmd - rightLauncher.getVelocity());
        if (err < 9 && launcherVelocityCmd > 933) {
            if (rpmSettleTimerWasReset) { rpmSettleTimer.reset(); rpmSettleTimerWasReset = false; }
            return rpmSettleTimer.seconds() >= PedroRobotConstants.RPM_SETTLE_TIME_SECONDS;
        }
        rpmSettleTimerWasReset = true;
        return false;
    }

    // =====================================================================
    //  TELEMETRY
    // =====================================================================
    private void displayTelemetry() {
        long elapsed = (System.currentTimeMillis() - startingTimeMsec) / 1000;
        telemetry.addData("Elapsed Sec", elapsed);
        telemetry.addData("Phase / SubStep / ShootSub",
                "%d / %d / %d", autoPhase, autoSubStep, shootSubStep);
        telemetry.addData("Config Step", currentStepIndex + 1);
        telemetry.addData("Robot X/Y/H", "%.1f in  %.1f in  %.1f deg",
                robotX, robotY, robotHeading);
        telemetry.addData("Turret Angle", "%.1f deg", turretAngleDeg);
        telemetry.addData("Pedro Busy", follower.isBusy());
        telemetry.addData("Launcher Tic/second cmd/act", "%.0f / %.0f", launcherVelocityCmd, rightLauncher.getVelocity());
        telemetry.addData("RPM Accurate", rpmAccurate());
        for (int i = 0; i < stepOptionIndex.size(); i++) {
            String marker = (i == currentStepIndex && autoPhase == 2) ? ">" : " ";
            telemetry.addData(marker + " Step " + (i + 1),
                    STEP_OPTION_NAMES[stepOptionIndex.get(i)]);
        }
        telemetry.update();
    }
}
