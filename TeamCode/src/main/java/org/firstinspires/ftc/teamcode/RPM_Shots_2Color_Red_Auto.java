package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.PossumsConstants;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Alliance-aware autonomous — configurable step sequence.
 *
 * TO CREATE RED VERSION: copy this file, then make exactly these 3 changes:
 *   1. Rename file and class to Worlds_2Color_Red_Auto
 *   2. Change @Autonomous: name="Worlds_2Color_Red_Auto", preselectTeleOp="Worlds_2Color_Red_TeleOp"
 *   3. Change:  private static final boolean BLUE_ALLIANCE = false;
 * No other code changes are needed.
 *
 * PRE-START MENU (gamepad1):
 *   DPAD UP / DOWN    — select which step row to edit
 *   DPAD RIGHT / LEFT — cycle the action for the selected step
 *   START             — lock in sequence and proceed to limelight seeding
 */
@Autonomous(name = "RPM_Shots_2Color_Red_Auto", preselectTeleOp = "RPM_Shots_2Color_Red_TeleOp", group = "Match")
public class RPM_Shots_2Color_Red_Auto extends LinearOpMode {

    // *** ONLY LINE TO CHANGE FOR RED ALLIANCE ***
    private static final boolean BLUE_ALLIANCE = false;
    private static final double CAMERA_X_FUDGE = -3.0;
    private static final double CAMERA_Y_FUDGE = + 3.0;

    // =====================================================================
    //  LAUNCHER LOGGING — set to true to enable, false to disable
    //
    //  DASHBOARD (live graph):
    //    1. Sync Gradle, deploy to robot
    //    2. Connect laptop to Control Hub WiFi
    //    3. Open browser to 192.168.43.1:8080/dash
    //    4. Run auto — graphs for cmd/right/left ticks/sec appear live
    //    NOTE: data is lost when OpMode stops; use CSV for post-run analysis
    //
    //  CSV FILE (post-run analysis):
    //    1. Run the auto
    //    2. Connect laptop to Control Hub via USB
    //    3. In Android Studio terminal run:
    //         adb pull /sdcard/FIRST/launcher_log.csv
    //    4. Open launcher_log.csv in Excel or Google Sheets and chart columns
    //    NOTE: each run overwrites the previous file
    //
    //  TO DISABLE FOR COMPETITION: set LAUNCHER_LOGGING_ENABLED = false
    // =====================================================================
    private static final boolean LAUNCHER_LOGGING_ENABLED = false;

    // Prism PWM constants
    private static final double NORMAL_RAINBOW = 0.1988;
    private static final double FAST_RAINBOW   = 0.223;
    private static final double COLOR_GREEN    = 0.388;
    private static final double COLOR_BLUE     = 0.611;

    private static final double INTAKE_POWER             = 1.0;
    private static final double TRANSFER_POWER           = 1.0;
    private static final double DRIVE_TIMEOUT_SEC        = 5.0;
    private static final double BALL_LINE_MAX_POWER      = 0.5;
    private static final double BALL_LINE_X_MARGIN_IN    = 9.0;
    private static final double INTAKE_WALL_CLEARANCE_IN = 1.5;
    private static final double SHOOTER_READY_MAX_WAIT_SEC       = 2.0;  // max wait per ball for RPM+direction
    private static final double SHOT_DETECT_TIMEOUT_SEC  = 1.5;  // max feed time before assuming ball shot
    private static final double SHOT_RPM_DROP_FRACTION   = 0.80; // RPM below this fraction = shot detected

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
    //  ALLIANCE-SPECIFIC VALUES — set once in initAllianceConstants()
    //  All field coordinates and headings that differ between Blue and Red
    //  live here; the rest of the code references these fields only.
    // =====================================================================
    private double     shootTargetX;
    private double[]   nearShootPose;
    private double[]   farShootPose;
    private double     gateX;
    private double     gateY;
    private double     gatePushX;           // chute-tracking X to press the gate lever
    private double     ballLineClearX;      // safe X to approach a ball line from
    private double     ballPickupCompleteX; // X after sweeping through all balls
    private double     gateHeading;         // robot heading while pushing gate (deg)
    private double     pickupHeading;       // robot heading during ball-line intake (deg)
    private double[][] startPoses;          // 4 predefined start poses for Limelight snap
    private String     allianceName;

    // =====================================================================
    //  HARDWARE
    // =====================================================================
    private DcMotorEx   intake;
    private CRServo     transfer;
    private DcMotorEx   rightLauncher, leftLauncher;
    private DcMotorEx   turretMotor;
    private Limelight3A limelight;
    private Follower    follower;
    private Servo prism;

    // =====================================================================
    //  POS
    // =====================================================================
    private double  robotX, robotY, robotHeading;
    private double  initialRobotY       = 0;
    private double  initialRobotHeading = 0;
    private double  endNearX, endNearY, endFarX, endFarY;

    // =====================================================================
    //  TURRET
    // =====================================================================
    private double turretAngleDeg = 0;

    // =====================================================================
    //  LAUNCHER
    // =====================================================================
    private double      launcherVelocityCmd = 0;
    private PrintWriter launcherLog         = null;
    private ElapsedTime logTimer            = new ElapsedTime();

    // =====================================================================
    //  TIMING / STATE
    // =====================================================================
    private long        startingTimeMsec       = 0;
    private double      startingBatteryVoltage = 0;
    private ElapsedTime delayTimer             = new ElapsedTime();
    private boolean     isDelayRunning         = false;
    private ElapsedTime rpmSettleTimer         = new ElapsedTime();
    private boolean     rpmSettleTimerWasReset = true;
    private ElapsedTime loopTimer              = new ElapsedTime();

    // =====================================================================
    //  SEQUENCE STATE
    // =====================================================================
    private int     autoPhase        = 0;
    private int     autoSubStep      = 0;
    private int     shootSubStep     = 0;
    private boolean endGameStarted   = false;
    private int     ballsShot        = 0;
    private int     currentStepIndex = 0;
    private boolean line2Picked      = false;
    private boolean line3Picked      = false;

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

        initAllianceConstants();
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
            turretMotor.setPower(1.0);
            startingTimeMsec    = System.currentTimeMillis();
            initialRobotY       = robotY;
            initialRobotHeading = robotHeading;
            launcherVelocityCmd = 1;
            limelight.stop();

            while (opModeIsActive()) {
                double elapsedSec = (System.currentTimeMillis() - startingTimeMsec) / 1000.0;

                if (elapsedSec >= 29.5) break;  //this will run stopRobot()

                if (elapsedSec >= 27.0 && !endGameStarted) {
                    endGameStarted  = true;
                    follower.breakFollowing();
                    intake.setPower(0);
                    transfer.setPower(0);
                    launcherVelocityCmd = 0;
                    autoPhase       = 98;
                    autoSubStep     = 0;
                    isDelayRunning  = false;
                }

                follower.update();
                readPoseFromFollower();
                updateTurret();
                if (autoPhase < 99) updateLauncher();
                runAutonomousSequence();
                updatePrism();
                updateLogging();
                displayTelemetry();
            }
        } finally {
            stopRobot();
        }
    }

    // =====================================================================
    //  ALLIANCE CONSTANTS INIT — the only if/else in the whole file
    // =====================================================================
    private void initAllianceConstants() {
        if (BLUE_ALLIANCE) {
            shootTargetX        = PedroFieldConstants.BLUE_SHOOT_TARGET_X;
            nearShootPose       = PedroFieldConstants.BLUE_NEAR_SHOOT_POSE;
            farShootPose        = PedroFieldConstants.BLUE_FAR_SHOOT_POSE;
            gateX               = PedroFieldConstants.BLUE_GATE_X;
            gateY               = PedroFieldConstants.BLUE_GATE_Y;
            gatePushX           = gateX - 2.0 + PedroRobotConstants.CHUTE_TO_FRONT_IN;
            ballLineClearX      = PedroFieldConstants.FLOOR_BALLS_BLUE_X
                                  + BALL_LINE_X_MARGIN_IN
                                  + PedroRobotConstants.CHUTE_TO_FRONT_IN;
            ballPickupCompleteX = PedroRobotConstants.CHUTE_TO_FRONT_IN + INTAKE_WALL_CLEARANCE_IN;
            gateHeading         = 180.0;
            pickupHeading       = 180.0;
            startPoses          = new double[][] {
                PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1,
                PedroFieldConstants.BLUE_AUDIENCE_START_POSE_2,
                PedroFieldConstants.BLUE_AUDIENCE_START_POSE_3,
                PedroFieldConstants.BLUE_BACK_WALL_START_POSE
            };
            endNearX     = PedroFieldConstants.BLUE_END_NEAR_X;
            endNearY     = PedroFieldConstants.BLUE_END_NEAR_Y;
            endFarX      = PedroFieldConstants.BLUE_END_FAR_X;
            endFarY      = PedroFieldConstants.BLUE_END_FAR_Y;
            allianceName = "BLUE";
        } else {
            shootTargetX        = PedroFieldConstants.RED_SHOOT_TARGET_X;
            nearShootPose       = PedroFieldConstants.RED_NEAR_SHOOT_POSE;
            farShootPose        = PedroFieldConstants.RED_FAR_SHOOT_POSE;
            gateX               = PedroFieldConstants.RED_GATE_X;
            gateY               = PedroFieldConstants.RED_GATE_Y;
            gatePushX           = gateX + 2.0 - PedroRobotConstants.CHUTE_TO_FRONT_IN;
            ballLineClearX      = PedroFieldConstants.FLOOR_BALLS_RED_X
                                  - BALL_LINE_X_MARGIN_IN
                                  - PedroRobotConstants.CHUTE_TO_FRONT_IN;
            ballPickupCompleteX = PedroFieldConstants.FIELD_WIDTH_IN
                                  - PedroRobotConstants.CHUTE_TO_FRONT_IN
                                  - INTAKE_WALL_CLEARANCE_IN;
            gateHeading         = 0.0;
            pickupHeading       = 0.0;
            startPoses          = new double[][] {
                PedroFieldConstants.RED_AUDIENCE_START_POSE_1,
                PedroFieldConstants.RED_AUDIENCE_START_POSE_2,
                PedroFieldConstants.RED_AUDIENCE_START_POSE_3,
                PedroFieldConstants.RED_BACK_WALL_START_POSE
            };
            endNearX     = PedroFieldConstants.RED_END_NEAR_X;
            endNearY     = PedroFieldConstants.RED_END_NEAR_Y;
            endFarX      = PedroFieldConstants.RED_END_FAR_X;
            endFarY      = PedroFieldConstants.RED_END_FAR_Y;
            allianceName = "RED";
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

            telemetry.addLine("=== AUTO STEP CONFIGURATION  (" + allianceName + " alliance) ===");
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

        List<Integer> expanded = new ArrayList<>();
        for (int idx : stepOptionIndex) {
            if (STEP_OPTIONS[idx] == StepAction.TWELVE_BALL) {
                expanded.add(StepAction.LINE_2.ordinal());
                expanded.add(StepAction.GATE.ordinal());
                expanded.add(StepAction.LINE_1.ordinal());
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

            case 0:
                intake.setPower(0);
                transfer.setPower(0);
                if (initialRobotY >= 60.0) {
                    double[] pose = shootPose();
                    driveToPose(pose[0], pose[1], pose[2], false);
                }
                shootSubStep   = 0;
                ballsShot      = 0;
                isDelayRunning = false;
                autoPhase      = 1;
                break;

            case 1:
                if (executeShootSequence()) {
                    currentStepIndex = 0;
                    autoSubStep      = 0;
                    isDelayRunning   = false;
                    autoPhase        = 2;
                }
                break;

            case 2:
                if (isStopRequested()) return;
                if (currentStepIndex >= stepOptionIndex.size()) { autoPhase = 49; break; }
                StepAction action = STEP_OPTIONS[stepOptionIndex.get(currentStepIndex)];
                if (action == StepAction.DONE) { autoPhase = 49; break; }
                boolean stepDone;
                if (action == StepAction.GATE) {
                    stepDone = executeGateStep();
                } else {
                    stepDone = executeBallLineAndShootStep(action);
                }
                if (stepDone) {
                    if (action == StepAction.LINE_2) line2Picked = true;
                    if (action == StepAction.LINE_3) line3Picked = true;
                    currentStepIndex++;
                    autoSubStep    = 0;
                    isDelayRunning = false;
                }
                break;

            case 98:
                double endX;
                double endY;
                if (robotY < 60.0) {
                    endX = endNearX;
                    endY = endNearY;
                } else {
                    endX = endFarX;
                    endY = endFarY;
                }
                driveToPose(endX, endY, 90.0, false);
                autoPhase = 49;
                break;

            case 49:
                break;

            default:
                stopRobot();
                autoPhase = 49;
                break;
        }
    }

    // =====================================================================
    //  SHOOT SEQUENCE — RPM-gated, 3 balls
    //
    //  Case 0: Wait up to SHOOTER_READY_MAX_WAIT_SEC for RPM + direction ready,
    //          then start feeding (case 1).
    //  Case 1: Run intake + transfer at full power. Stop when a shot is
    //          detected (20% RPM drop) or SHOT_DETECT_TIMEOUT_SEC elapses.
    //          Increment ballsShot; if 3 done → return true, else → case 0.
    // =====================================================================
    private boolean executeShootSequence() {  //1st call to this after driving the transfer and intake are at 0 power
        switch (shootSubStep) {
            case 0:
                // Short shots (near position): skip full RPM+direction check for balls 2 & 3,
                // but still wait for RPM to recover above the shot-detection threshold first
                // to prevent shotDetected() from triggering immediately on re-entry.
                if (robotY >= 60.0 && ballsShot > 0) {
                    if (!shotDetected() && nonBlockingDelay(0.1)) {  //forces short shots to wait for  above rpm droop threshold
                        isDelayRunning = false;
                        shootSubStep   = 1;
                    }
                } else if ((rpmReadyToShoot() && turretAtTarget())
                        || nonBlockingDelay(SHOOTER_READY_MAX_WAIT_SEC)) {
                    isDelayRunning = false;
                    shootSubStep   = 1;
                }
                return false;

            case 1:
                intake.setPower(INTAKE_POWER);
                transfer.setPower(TRANSFER_POWER);
                if (shotDetected()) {
                    // RPM dropped — real shot detected; stop feeding and wait for recovery
                    isDelayRunning = false;
                    intake.setPower(0);
                    transfer.setPower(0);
                    ballsShot++;
                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        shootSubStep = 99;
                        return true;
                    }
                    shootSubStep = 0;  // wait for RPM recovery before next ball
                } else if (nonBlockingDelay(SHOT_DETECT_TIMEOUT_SEC)) {
                    // No RPM drop detected — assume ball shot; keep feeding, just restart timer
                    isDelayRunning = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        intake.setPower(0);
                        transfer.setPower(0);
                        ballsShot = 0; shootSubStep = 99; return true;
                    }
                    // Stay in case 1 — isDelayRunning=false restarts the timer next iteration
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
        double gateYBias = 0.0;
        if (line2Picked && !line3Picked) gateYBias = -3.0;
        if (line3Picked && !line2Picked) gateYBias = +3.0;
        double targetGateY = gateY - PedroRobotConstants.CHUTE_TO_PUSHER_RIGHT_IN + gateYBias;

        switch (autoSubStep) {
            case 0:
                driveToPose(ballLineClearX, targetGateY, gateHeading, false);
                autoSubStep = 1;
                return false;
            case 1:
                driveToPose(gatePushX, targetGateY, gateHeading, false);
                autoSubStep = 2;
                return false;
            case 2:
                if (nonBlockingDelay(0.5)) {
                    isDelayRunning = false;
                    if (isLastActionableStep()) {
                        autoSubStep = 99;
                    } else {
                        autoSubStep = 3;
                    }
                }
                return false;
            case 3:
                driveToPose(ballLineClearX, targetGateY, gateHeading, false);
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
        double lineY = getBallLineY(line);

        switch (autoSubStep) {
            case 0:
                driveToPose(ballLineClearX, lineY, pickupHeading, false);
                autoSubStep = 1;
                return false;

            case 1:
                intake.setPower(INTAKE_POWER);
                driveToPoseSlow(ballPickupCompleteX, lineY, pickupHeading, false);
                intake.setPower(0);
                autoSubStep = 2;
                return false;

            case 2:
                driveToPose(ballLineClearX, lineY, pickupHeading, false);
                autoSubStep = 3;
                return false;

            case 3:
                intake.setPower(0);
                transfer.setPower(0);
                double[] pose = shootPose();
                driveToPose(pose[0], pose[1], pose[2], false);
                shootSubStep   = 0;
                ballsShot      = 0;
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
    //  NAVIGATION
    // =====================================================================
    private void driveToPose(double toX, double toY, double headingDeg, boolean holdEnd) {
        followPathBlocking(buildPath(toX, toY, headingDeg), holdEnd, 1.0);
    }

    private void driveToPoseSlow(double toX, double toY, double headingDeg, boolean holdEnd) {
        followPathBlocking(buildPath(toX, toY, headingDeg), holdEnd, BALL_LINE_MAX_POWER);
    }

    private Object buildPath(double toX, double toY, double headingDeg) {
        Pose   currentPose = follower.getPose();
        double fromH       = currentPose.getHeading();
        double toH         = Math.toRadians(headingDeg);
        Pose   start       = new Pose(currentPose.getX(), currentPose.getY(), fromH);
        Pose   end         = new Pose(toX, toY, toH);
        Path   path        = new Path(new BezierLine(start, end));
        path.setLinearHeadingInterpolation(fromH, toH, 0.6);
        return path;
    }

    private void followPathBlocking(Object pathOrChain, boolean holdEnd, double maxPower) {
        if (pathOrChain instanceof PathChain) {
            follower.followPath((PathChain) pathOrChain, maxPower, holdEnd);
        } else {
            follower.setMaxPower(maxPower);
            follower.followPath((Path) pathOrChain, holdEnd);
        }

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timeout.seconds() < DRIVE_TIMEOUT_SEC
                && (System.currentTimeMillis() - startingTimeMsec) < 29500) {
            follower.update();
            readPoseFromFollower();
            updateTurret();
            updateLauncher();
            updatePrism();
            updateLogging();
            displayTelemetry();
        }

        if (maxPower < 1.0) follower.setMaxPower(1.0);
    }

    // =====================================================================
    //  TURRET CONTROL
    // =====================================================================
    private void updateTurret() {
        double dx              = shootTargetX - robotX;
        double dy              = PedroFieldConstants.SHOOT_TARGET_Y - robotY;
        double fieldBearingRad = Math.atan2(dy, dx);
        double turretRad       = fieldBearingRad - Math.toRadians(robotHeading);
        turretRad      = Math.atan2(Math.sin(turretRad), Math.cos(turretRad));
        turretAngleDeg = Math.toDegrees(turretRad);
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

    private double[] shootPose() {
        return (initialRobotY < 60.0) ? nearShootPose : farShootPose;
    }

    private void readPoseFromFollower() {
        Pose p       = follower.getPose();
        robotX       = p.getX();
        robotY       = p.getY();
        robotHeading = Math.toDegrees(p.getHeading());
    }

    // =====================================================================
    //  LIMELIGHT POSE SEEDING
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
                String reason = result == null            ? "null result"
                        : !result.isValid()               ? "isValid=false"
                        : result.getBotpose() == null     ? "botpose=null"
                        : "tagCount=" + result.getBotposeTagCount();
                telemetry.addData("Poll fail", "(%d) %s", failCount, reason);
                telemetry.update();
            }

            if (xSamples.size() >= TARGET_SAMPLES || failCount > MAX_FAIL_COUNT) break;
            sleep(POLL_SLEEP_MS);
        }

        telemetry.addData("Poll done", "samples=%d  fails=%d", xSamples.size(), failCount);
        telemetry.update();
        sleep(500);

        if (xSamples.isEmpty()) {
            telemetry.addData("Limelight", "No samples — pose not set");
            telemetry.update();
            sleep(500);
            return;
        }

        final double OUTLIER_THRESHOLD = 0.07;
        xSamples = filterOutliers(xSamples, OUTLIER_THRESHOLD);
        ySamples = filterOutliers(ySamples, OUTLIER_THRESHOLD);
        if (xSamples.isEmpty()) return;

        double avgX = 0, avgY = 0;
        for (double v : xSamples) avgX += v;
        for (double v : ySamples) avgY += v;
        avgX /= xSamples.size();
        avgY /= ySamples.size();

        // Limelight WCS (meters, field-center origin) → Pedro inches (audience-left corner)
        double camPedroX = avgY * 39.37 + 72.0 + CAMERA_X_FUDGE;  // +4 in empirical correction
        double camPedroY = -avgX * 39.37 + 72.0 + CAMERA_Y_FUDGE;

        double   minDist  = Double.MAX_VALUE;
        double[] bestPose = null;
        for (double[] pose : startPoses) {
            double dist = Math.hypot(camPedroX - pose[0], camPedroY - pose[1]);
            if (dist < minDist) {
                minDist  = dist;
                bestPose = pose;
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

    private List<Double> filterOutliers(List<Double> samples, double threshold) {
        if (samples.size() < 3) return samples;
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
        intake        = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.INTAKE_CONFIG_NAME);
        transfer      = hardwareMap.get(CRServo.class,   PedroRobotConstants.TRANSFER_SERVO_CONFIG_NAME);
        rightLauncher = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
        leftLauncher  = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
        turretMotor   = hardwareMap.get(DcMotorEx.class, PedroRobotConstants.TURRET_MOTOR_CONFIG_NAME);

        transfer.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setVelocityPIDFCoefficients(230, 0, 0, 13);
        leftLauncher.setVelocityPIDFCoefficients(230, 0, 0, 13);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turretMotor.setPositionPIDFCoefficients(PedroRobotConstants.TURRET_PIDF);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setTargetPositionTolerance((int) PedroRobotConstants.TURRET_TICKS_PER_DEG);

        follower = PossumsConstants.createFollower(hardwareMap);
        prism    = hardwareMap.get(Servo.class, "prism");

        if (LAUNCHER_LOGGING_ENABLED) {
            try {
                launcherLog = new PrintWriter("/sdcard/FIRST/launcher_log.csv");
                launcherLog.println("time_sec,cmd,right,left,transfer_pwr,intake_pwr,intake_vel");
                logTimer.reset();
            } catch (Exception e) {
                launcherLog = null;
            }
        }
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
                robotX - shootTargetX,
                robotY - PedroFieldConstants.SHOOT_TARGET_Y);
        launcherVelocityCmd = 1160 + distToGoal * 3.5;
        rightLauncher.setVelocity(launcherVelocityCmd);
        leftLauncher.setVelocity(launcherVelocityCmd);
    }

    // =====================================================================
    //  LOGGING — called every loop iteration regardless of auto phase
    // =====================================================================
    private void updateLogging() {
        if (!LAUNCHER_LOGGING_ENABLED) return;

        double right       = rightLauncher.getVelocity();
        double left        = leftLauncher.getVelocity();
        double transferPwr = transfer.getPower();
        double intakePwr   = intake.getPower();
        double intakeVel   = intake.getVelocity();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("cmd (ticks/s)",   launcherVelocityCmd);
        packet.put("right (ticks/s)", right);
        packet.put("left (ticks/s)",  left);
        packet.put("transfer pwr",    transferPwr);
        packet.put("intake pwr",      intakePwr);
        packet.put("intake vel",      intakeVel);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (launcherLog != null)
            launcherLog.printf("%.3f,%.0f,%.0f,%.0f,%.2f,%.2f,%.0f%n",
                    logTimer.seconds(), launcherVelocityCmd, right, left,
                    transferPwr, intakePwr, intakeVel);
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
        QuickOdometryStorage.alliance      = allianceName;
        QuickOdometryStorage.valid         = true;

        if (launcherLog != null) { launcherLog.flush(); launcherLog.close(); }
    }

    private void updatePrism() {
        //- Fast rainbow — follower driving (highest priority)
        //- Blue — waiting for RPM + turret ready (shootSubStep == 0 during shoot phase)
        //- Green — feeding, waiting for shot detect (shootSubStep == 1)
        //- OFF — idle/done

        boolean inShootPhase = (autoPhase == 1)
                || (autoPhase == 2 && autoSubStep == 4);
        if (follower.isBusy()) {
            prism.setPosition(FAST_RAINBOW);
        } else if (inShootPhase && shootSubStep == 1) {
            prism.setPosition(COLOR_GREEN);
        } else if (inShootPhase && shootSubStep == 0) {
            prism.setPosition(COLOR_BLUE);
        } else {
            prism.setPosition(0.0);
        }
    }

    private boolean nonBlockingDelay(double sec) {
        if (!isDelayRunning) { delayTimer.reset(); isDelayRunning = true; return false; }
        if (delayTimer.seconds() < sec) return false;
        isDelayRunning = false;
        return true;
    }

    private boolean rpmReadyToShoot() {
        return (launcherVelocityCmd > 933
                && Math.abs(launcherVelocityCmd - rightLauncher.getVelocity()) <= 70);
    }

    private boolean turretAtTarget() {
        double actualDeg  = turretMotor.getCurrentPosition() / PedroRobotConstants.TURRET_TICKS_PER_DEG;
        // Near/short shots (large Y) need less direction precision — allow 2x tolerance
        double tolerance;
        if (robotY >= 60.0) {
            tolerance = PedroRobotConstants.TURRET_ACCURACY_DEG * 2.0;  // near shot — looser
        } else {
            tolerance = PedroRobotConstants.TURRET_ACCURACY_DEG;         // far shot — tight
        }
        return Math.abs(actualDeg - turretAngleDeg) <= tolerance;
    }

    private boolean shotDetected() {
        return rightLauncher.getVelocity() < launcherVelocityCmd * SHOT_RPM_DROP_FRACTION;
    }

    private boolean rpmAccurate() {
        double err = Math.abs(launcherVelocityCmd - rightLauncher.getVelocity());
        if (err < 20 && launcherVelocityCmd > 933) {
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
        telemetry.addData("Loop ms",      "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addData("Alliance", allianceName);
        telemetry.addData("Phase / SubStep / ShootSub / Balls",
                "%d / %d / %d / %d", autoPhase, autoSubStep, shootSubStep, ballsShot);
        telemetry.addData("Robot X/Y/H", "%.1f in  %.1f in  %.1f deg",
                robotX, robotY, robotHeading);
        telemetry.addData("Turret Angle", "%.1f deg", turretAngleDeg);
        telemetry.addData("Pedro Busy", follower.isBusy());
        telemetry.addData("Launcher Tic/second cmd/act",
                "%.0f / %.0f", launcherVelocityCmd, rightLauncher.getVelocity());
        telemetry.addData("RPM Ready",    rpmReadyToShoot());
        telemetry.addData("Turret Ready", turretAtTarget());
        telemetry.addData("Shot Detect",  shotDetected());
        telemetry.addData("Balls Shot",   ballsShot);
        for (int i = 0; i < stepOptionIndex.size(); i++) {
            String marker = (i == currentStepIndex && autoPhase == 2) ? ">" : " ";
            telemetry.addData(marker + " Step " + (i + 1),
                    STEP_OPTION_NAMES[stepOptionIndex.get(i)]);
        }
        telemetry.update();
    }
}
