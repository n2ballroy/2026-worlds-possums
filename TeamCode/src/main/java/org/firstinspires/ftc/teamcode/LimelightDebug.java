package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "LimelightDebug")
public class LimelightDebug extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    private boolean pedroHasValidPose = false;
    private double  pedroX = 0, pedroY = 0, pedroHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        // Init pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.resetPosAndIMU();

        // Init limelight — same startup sequence as auto
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PedroRobotConstants.GOALS_PIPELINE);
        sleep(500);
        limelight.start();
        sleep(500);

        telemetry.addData("Status", "Ready. Press START, then A to run pose seed.");
        telemetry.update();

        waitForStart();

        boolean prevA = false;

        while (opModeIsActive()) {
            pinpoint.update();

            // Press A to (re-)run the pose seeding poll
            if (gamepad1.a && !prevA) {
                pedroHasValidPose = false;
                pollLimelight();
            }
            prevA = gamepad1.a;

            // ── Live limelight status ──────────────────────────────────────
            LLResult result = limelight.getLatestResult();
            telemetry.addData("=== LIMELIGHT ===", "");
            if (result == null) {
                telemetry.addData("Result", "NULL");
            } else {
                telemetry.addData("Result valid",      result.isValid());
                telemetry.addData("BotposeTagCount",   result.getBotposeTagCount());
                telemetry.addData("Botpose null",      result.getBotpose() == null);
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                telemetry.addData("Fiducials",         fids == null ? "null" : fids.size());
                if (result.getBotpose() != null) {
                    Pose3D bp = result.getBotpose();
                    telemetry.addData("Raw X/Y (m)",   "%.3f / %.3f",
                            bp.getPosition().x, bp.getPosition().y);
                    double camPedroX =  bp.getPosition().y * 39.37 + 72.0;
                    double camPedroY = -bp.getPosition().x * 39.37 + 72.0;
                    telemetry.addData("Pedro X/Y (in)", "%.1f / %.1f", camPedroX, camPedroY);
                }
            }

            // ── Pose seed result ──────────────────────────────────────────
            telemetry.addData("=== POSE SEED ===", "");
            telemetry.addData("pedroHasValidPose", pedroHasValidPose);
            if (pedroHasValidPose) {
                telemetry.addData("Seeded pose",
                        "X=%.1f  Y=%.1f  H=%.1f deg", pedroX, pedroY, pedroHeading);
            }

            // ── Pinpoint ──────────────────────────────────────────────────
            telemetry.addData("=== PINPOINT ===", "");
            telemetry.addData("X / Y (in)",   "%.2f / %.2f",
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f",
                    pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.addData("--- Controls ---", "A = run pose seed");
            telemetry.update();
        }

        limelight.stop();
    }

    // =========================================================================
    //  POLL — exact same logic as auto, with verbose failure reasons
    // =========================================================================
    private void pollLimelight() {
        final int TARGET_SAMPLES = 20;
        final int MAX_FAIL_COUNT = 10;
        final int POLL_SLEEP_MS  = 33;

        List<Double> xSamples  = new ArrayList<>();
        List<Double> ySamples  = new ArrayList<>();
        int    failCount       = 0;
        double lastTimestamp   = -1;

        telemetry.addData("Poll", "Running...");
        telemetry.update();

        while (opModeIsActive()) {
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
                    telemetry.addData("Poll fail", "No fiducials (%d)", failCount);
                    telemetry.update();
                }
            } else {
                failCount++;
                // Diagnose exactly which condition failed
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

        if (xSamples.isEmpty()) {
            telemetry.addData("Outcome", "NO SAMPLES — pose not set");
            telemetry.update();
            return;
        }

        final double OUTLIER_THRESHOLD = 0.07;
        List<Double> filtX = filterOutliers(xSamples, OUTLIER_THRESHOLD);
        List<Double> filtY = filterOutliers(ySamples, OUTLIER_THRESHOLD);
        telemetry.addData("After filter", "x=%d  y=%d", filtX.size(), filtY.size());

        if (filtX.isEmpty()) {
            telemetry.addData("Outcome", "All samples filtered as outliers");
            telemetry.update();
            return;
        }

        double avgX = 0, avgY = 0;
        for (double v : filtX) avgX += v;
        for (double v : filtY) avgY += v;
        avgX /= filtX.size();
        avgY /= filtY.size();

        double camPedroX =  avgY * 39.37 + 72.0;
        double camPedroY = -avgX * 39.37 + 72.0;
        telemetry.addData("Avg cam Pedro X/Y", "%.1f / %.1f", camPedroX, camPedroY);

        double[][] poses = {
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1,
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_2,
            PedroFieldConstants.BLUE_AUDIENCE_START_POSE_3,
            PedroFieldConstants.BLUE_BACK_WALL_START_POSE
        };

        double minDist    = Double.MAX_VALUE;
        double[] bestPose = null;
        for (double[] pose : poses) {
            double dist = Math.hypot(camPedroX - pose[0], camPedroY - pose[1]);
            telemetry.addData(String.format("  pose(%.0f,%.0f)", pose[0], pose[1]),
                    "dist=%.1f in", dist);
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
            telemetry.addData("Outcome", "Snapped to (%.1f, %.1f, %.1f deg)  dist=%.1f in",
                    pedroX, pedroY, pedroHeading, minDist);
        }
        telemetry.update();
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
}