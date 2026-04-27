package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Pre-match camera calibration tool.
 *
 * Shows all 10 known field positions at once.  Place the robot at a position,
 * press A to measure it, then move to the next.  Unmeasured positions show
 * dashes.  Overall mean error across all measured positions is shown at the
 * bottom and is the best estimate of the camera memory correction needed.
 *
 * Camera conversion uses exactly the same formula as production teleop:
 *   Pedro X = llY * 39.37 + 72.0
 *   Pedro Y = -llX * 39.37 + 72.0
 *   Heading = llYaw - 90.0
 * No software fudge factor is applied.
 *
 * CONTROLS
 *   DPAD UP / DOWN : cycle through the 10 known poses
 *   A              : take a measurement at the selected pose (~3 s)
 *   X              : clear all measurements
 */
@TeleOp(name = "Camera_Calibration", group = "Calibration")
public class CameraCalibration_TeleOp extends LinearOpMode {

    private static final int  SAMPLES_PER_READING = 20;
    private static final long SAMPLE_WINDOW_MS    = 3000;

    // -------------------------------------------------------------------------
    //  Known ground-truth poses  { X_in, Y_in, headingDeg }
    // -------------------------------------------------------------------------
    private static final String[] POSE_NAMES = {
        "Blue 1", "Blue 2", "Blue 3", "Blue 4",    "Blue Center",
        "Red 1",  "Red 2",  "Red 3",  "Red 4",     "Red Center"
    };


    private static final double[][] KNOWN_POSES = {
        PedroFieldConstants.BLUE_AUDIENCE_START_POSE_1,
        PedroFieldConstants.BLUE_AUDIENCE_START_POSE_2,
        PedroFieldConstants.BLUE_AUDIENCE_START_POSE_3,
        PedroFieldConstants.BLUE_BACK_WALL_START_POSE,
        { 72.0, 72.0, 135.0 },
        PedroFieldConstants.RED_AUDIENCE_START_POSE_1,
        PedroFieldConstants.RED_AUDIENCE_START_POSE_2,
        PedroFieldConstants.RED_AUDIENCE_START_POSE_3,
        PedroFieldConstants.RED_BACK_WALL_START_POSE,
        { 72.0, 72.0,  45.0 }
    };

    // -------------------------------------------------------------------------
    //  Measurement — one slot per pose, null until measured
    // -------------------------------------------------------------------------
    private static class Measurement {
        double camX, camY, camH;
        double dX,   dY,   dH;
        int    samples;
    }

    // -------------------------------------------------------------------------
    //  Hardware / state
    // -------------------------------------------------------------------------
    private Limelight3A       limelight;
    private final Measurement[] results      = new Measurement[KNOWN_POSES.length];
    private int                 selectedPose = 0;

    // -------------------------------------------------------------------------
    //  runOpMode
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PedroRobotConstants.GOALS_PIPELINE);
        limelight.start();

        boolean prevA = false, prevX = false, prevUp = false, prevDown = false;

        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Camera Calibration ready");
        telemetry.addLine("DPAD UP/DOWN: select pose | A: measure | X: clear");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean a    = gamepad1.a;
            boolean x    = gamepad1.x;
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up   && !prevUp)   selectedPose = (selectedPose + KNOWN_POSES.length - 1) % KNOWN_POSES.length;
            if (down && !prevDown) selectedPose = (selectedPose + 1) % KNOWN_POSES.length;

            if (x && !prevX) {
                results[selectedPose] = null;
            }

            if (a && !prevA) {
                Measurement m = takeMeasurement(selectedPose);
                if (m != null) {
                    results[selectedPose] = m;
                } else {
                    telemetry.addData("Measurement FAILED", "no valid camera frames in 3s");
                    telemetry.update();
                }
            }

            prevA = a; prevX = x; prevUp = up; prevDown = down;

            displayTelemetry();
        }

        limelight.stop();
    }

    // -------------------------------------------------------------------------
    //  Collect samples — same logic as production reseedFromLimelight()
    // -------------------------------------------------------------------------
    private Measurement takeMeasurement(int poseIndex) {
        List<Double> xSamples = new ArrayList<>();
        List<Double> ySamples = new ArrayList<>();
        List<Double> hSamples = new ArrayList<>();
        double       lastTs   = -1;
        long         deadline = System.currentTimeMillis() + SAMPLE_WINDOW_MS;

        while (opModeIsActive()
                && System.currentTimeMillis() < deadline
                && xSamples.size() < SAMPLES_PER_READING) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && result.getBotpose() != null
                    && result.getBotposeTagCount() > 0
                    && result.getTimestamp() != lastTs) {
                lastTs = result.getTimestamp();
                Pose3D bp = result.getBotpose();
                xSamples.add(bp.getPosition().x);
                ySamples.add(bp.getPosition().y);
                hSamples.add(bp.getOrientation().getYaw(AngleUnit.DEGREES));
            }
            sleep(33);
        }

        if (xSamples.isEmpty()) return null;

        // Simple mean for X/Y; circular mean for heading
        double avgLLX = 0, avgLLY = 0, sinSum = 0, cosSum = 0;
        for (int i = 0; i < xSamples.size(); i++) {
            avgLLX += xSamples.get(i);
            avgLLY += ySamples.get(i);
            sinSum += Math.sin(Math.toRadians(hSamples.get(i)));
            cosSum += Math.cos(Math.toRadians(hSamples.get(i)));
        }
        avgLLX /= xSamples.size();
        avgLLY /= xSamples.size();
        double avgYaw = Math.toDegrees(Math.atan2(sinSum, cosSum));

        // Same conversion as production teleop — no fudge factor
        double camX  = avgLLY * 39.37 + 72.0;
        double camY  = -avgLLX * 39.37 + 72.0;
        double camH  = avgYaw - 90.0;
        double rawDH = camH - KNOWN_POSES[poseIndex][2];

        Measurement m = new Measurement();
        m.camX    = camX;
        m.camY    = camY;
        m.camH    = camH;
        m.dX      = camX - KNOWN_POSES[poseIndex][0];
        m.dY      = camY - KNOWN_POSES[poseIndex][1];
        m.dH      = Math.toDegrees(Math.atan2(
                        Math.sin(Math.toRadians(rawDH)),
                        Math.cos(Math.toRadians(rawDH))));
        m.samples = xSamples.size();
        return m;
    }

    // -------------------------------------------------------------------------
    //  Telemetry — all 10 rows always visible, mean at bottom
    // -------------------------------------------------------------------------
    private void displayTelemetry() {
        telemetry.addLine("=== CAMERA CALIBRATION ===");
        telemetry.addLine("DPAD UP/DOWN: select  |  A: measure  |  X: clear");
        telemetry.addData("Selected", POSE_NAMES[selectedPose]);

        double sumDX = 0, sumDY = 0, sinDH = 0, cosDH = 0;
        int    count = 0;

        for (int i = 0; i < KNOWN_POSES.length; i++) {
            String prefix = (i == selectedPose) ? "-->" : "   ";
            Measurement m = results[i];
            if (m == null) {
                telemetry.addData(prefix + " " + POSE_NAMES[i], "---");
            } else {
                telemetry.addData(prefix + " " + POSE_NAMES[i],
                        "dX=%+.2f\" dY=%+.2f\" dH=%+.1f deg  (%d)",
                        m.dX, m.dY, m.dH, m.samples);
                sumDX  += m.dX;
                sumDY  += m.dY;
                sinDH  += Math.sin(Math.toRadians(m.dH));
                cosDH  += Math.cos(Math.toRadians(m.dH));
                count++;
            }
        }

        if (count > 0) {
            double meanDX = sumDX / count;
            double meanDY = sumDY / count;
            double meanDH = Math.toDegrees(Math.atan2(sinDH, cosDH));
            telemetry.addLine("--- Mean error (" + count + " points) ---");
            telemetry.addData("  Mean",
                    "dX=%+.2f\" dY=%+.2f\" dH=%+.1f deg", meanDX, meanDY, meanDH);
            telemetry.addData("  Cam adj",
                    "dX=%+.2f\" dY=%+.2f\"", -meanDX, -meanDY);
        }

        telemetry.update();
    }
}