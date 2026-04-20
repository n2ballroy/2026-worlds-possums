package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.PedroFieldConstants;
import org.firstinspires.ftc.teamcode.PedroRobotConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Same as MainTeleOpBlueAutoPinpoint but START button reseeds pinpoint from
 * a single valid Limelight frame rather than resetting to the corner.
 *
 * GAMEPAD1 START — point camera at a visible AprilTag and press START.
 *   The robot waits up to 2 seconds for a valid tagged frame, then reseeds
 *   X, Y, and heading from the Limelight botpose.  Telemetry confirms the
 *   new position or reports the failure reason.
 *
 * Coordinate conversion (same as auto):
 *   Pinpoint X (in) = limelight_Y (m) * 39.37 + 72 + 4   (4" empirical correction)
 *   Pinpoint Y (in) = -limelight_X (m) * 39.37 + 72
 *   Heading (deg)   = 90 - limelight_yaw
 *   NOTE: if heading is consistently off, adjust the formula above.
 */
@TeleOp(name = "MainTeleOpBlueLLReset", group = "TeleOp")
public class MainTeleOpBlueLLReset extends LinearOpMode {

    private DcMotorEx Rightshooter;
    private DcMotorEx Leftshooter;
    private CRServo   transfer;
    private DcMotorEx turret;
    private DcMotor   frontLeft;
    private DcMotor   backLeft;
    private DcMotor   backRight;
    private DcMotor   frontright;
    private DcMotorEx intake;
    private GoBildaPinpointDriver pinpointOdometry;
    private Limelight3A           limelight;

    private Servo prism;
    private ElapsedTime runtime = new ElapsedTime();

    // PWM Constants
    private static final double NORMAL_RAINBOW = 0.1988;
    private static final double FAST_RAINBOW   = 0.223;
    private static final double FTC_TIMER      = 0.0069;
    private static final double COLOR_GREEN    = 0.388;
    private static final double COLOR_BLUE     = 0.611;
    private static final double COLOR_OFF      = 0.0;

    @Override
    public void runOpMode() {
        int    Shooter_Speed;
        double Goal_X;
        double Goal_Y;
        double Delta__X_;
        double Delta__Y_;
        int    Aim;
        int    Offset;
        float  Y;
        float  X;
        float  RX;
        double X_Pinpoint;
        double Y_Pinpoint;
        double Yaw_Pinpoint;
        double Target_Heading;
        double Distance;
        double Target_Ticks;
        int    Shoot    = 1;
        boolean prevA     = false;
        boolean prevStart = false;
        ElapsedTime dpadTimer = new ElapsedTime();

        Rightshooter     = hardwareMap.get(DcMotorEx.class,           PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
        Leftshooter      = hardwareMap.get(DcMotorEx.class,           PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
        turret           = hardwareMap.get(DcMotorEx.class,           PedroRobotConstants.TURRET_MOTOR_CONFIG_NAME);
        intake           = hardwareMap.get(DcMotorEx.class,           PedroRobotConstants.INTAKE_CONFIG_NAME);
        transfer         = hardwareMap.get(CRServo.class,             PedroRobotConstants.TRANSFER_SERVO_CONFIG_NAME);
        prism            = hardwareMap.get(Servo.class,               "prism");
        pinpointOdometry = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight        = hardwareMap.get(Limelight3A.class,         "limelight");

        frontLeft  = hardwareMap.get(DcMotor.class, "right_rear_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRight  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontright = hardwareMap.get(DcMotor.class, "left_rear_drive");

        pinpointOdometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointOdometry.resetPosAndIMU();
        pinpointOdometry.recalibrateIMU();
        pinpointOdometry.setOffsets(-5.46, -2.7, DistanceUnit.INCH);

        Rightshooter.setVelocityPIDFCoefficients(150, 0, 0, 14);
        Leftshooter.setVelocityPIDFCoefficients(150, 0, 0, 14);
        transfer.setDirection(CRServo.Direction.REVERSE);
        Leftshooter.setDirection(DcMotor.Direction.FORWARD);
        Rightshooter.setDirection(DcMotor.Direction.REVERSE);
        Leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooter_Speed = 1600;
        Goal_X  = PedroFieldConstants.BLUE_SHOOT_TARGET_X;
        Goal_Y  = PedroFieldConstants.SHOOT_TARGET_Y;
        Delta__X_ = 0;
        Delta__Y_ = 0;
        Aim    = 0;
        Offset = 0;

        turret.setPositionPIDFCoefficients(12.5);
        transfer.setDirection(CRServo.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        telemetry.setMsTransmissionInterval(11);
        turret.setVelocity(1800);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setTargetPosition(0);
        turret.setTargetPositionTolerance(0);

        limelight.pipelineSwitch(PedroRobotConstants.GOALS_PIPELINE);
        limelight.start();

        if (QuickOdometryStorage.valid) {
            QuickOdometryStorage.valid = false;
            pinpointOdometry.setPosX(QuickOdometryStorage.x, DistanceUnit.INCH);
            pinpointOdometry.setPosY(QuickOdometryStorage.y, DistanceUnit.INCH);
            pinpointOdometry.setHeading(QuickOdometryStorage.heading, AngleUnit.DEGREES);
            turret.setTargetPosition((int) (QuickOdometryStorage.turretDegrees * PedroRobotConstants.TURRET_TICKS_PER_DEG));
            sleep(500);
        } else {
            pinpointOdometry.setPosX(0.0, DistanceUnit.INCH);
            pinpointOdometry.setPosY(0.0, DistanceUnit.INCH);
            pinpointOdometry.setHeading(90.0, AngleUnit.DEGREES);
            turret.setTargetPosition(0);
        }

        prism.setPosition(COLOR_BLUE);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            prismTimer();
            turret.setVelocity(1800);
            Y  = gamepad1.left_stick_y;
            X  = gamepad1.left_stick_x;
            RX = gamepad1.right_stick_x;

            X_Pinpoint   = pinpointOdometry.getPosX(DistanceUnit.INCH);
            Y_Pinpoint   = pinpointOdometry.getPosY(DistanceUnit.INCH);
            Yaw_Pinpoint = pinpointOdometry.getHeading(AngleUnit.DEGREES);
            Delta__X_    = Goal_X - X_Pinpoint;
            Delta__Y_    = Goal_Y - Y_Pinpoint;
            Target_Heading = Math.atan2(Delta__Y_, Delta__X_) / Math.PI * 180;
            Distance       = Math.sqrt(Math.pow(Delta__X_, 2) + Math.pow(Delta__Y_, 2));
            Shooter_Speed  = (int) (1160 + Distance * 3.75);

            if ((gamepad1.dpad_left || gamepad1.dpad_right) && dpadTimer.milliseconds() > 150) {
                if (gamepad1.dpad_left) Offset++;
                else                   Offset--;
                dpadTimer.reset();
            }

            // START — reseed pinpoint from Limelight (robot must see an AprilTag)
            if (gamepad1.start && !prevStart) {
                reseedFromLimelight();
            }

            double rawError    = Math.toRadians(Target_Heading - Yaw_Pinpoint + Offset);
            double headingError = Math.toDegrees(Math.atan2(Math.sin(rawError), Math.cos(rawError)));
            Target_Ticks = PedroRobotConstants.TURRET_TICKS_PER_DEG * headingError;

            if (gamepad1.right_bumper) {
                if (Y_Pinpoint < 50.0) {
                    intake.setPower(0.5);
                    transfer.setPower(0.3);
                } else {
                    intake.setPower(0.8);
                    transfer.setPower(1.0);
                }
            } else {
                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                transfer.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            if (gamepad1.a && !prevA) {
                Aim = (Aim == 0) ? 1 : 0;
            }
            if (Aim == 0) {
                turret.setTargetPosition(0);
            } else {
                turret.setTargetPosition((int) Math.min(Math.max(Target_Ticks, -1400), 1400));
            }

            if (gamepad1.b)      Shoot = 1;
            else if (gamepad1.x) Shoot = 0;

            if (Shoot == 1) {
                Leftshooter.setVelocity(Shooter_Speed);
                Rightshooter.setVelocity(Shooter_Speed);
            } else {
                Rightshooter.setVelocity(0);
                Leftshooter.setVelocity(0);
            }

            frontLeft.setPower((Y - X) - RX);
            backLeft.setPower((Y + X) - RX);
            frontright.setPower(Y + X + RX);
            backRight.setPower((Y - X) + RX);

            telemetry.addData("Target Heading", Double.parseDouble(JavaUtil.formatNumber(Target_Ticks, 3)));
            telemetry.addData("Shooter",   Shooter_Speed);
            telemetry.addData("Aim",       Aim);
            telemetry.addData("Delta X",   Delta__X_);
            telemetry.addData("Delta Y",   Delta__Y_);
            telemetry.addData("Distance",  Distance);
            telemetry.addData("X",         X_Pinpoint);
            telemetry.addData("Y",         Y_Pinpoint);
            telemetry.addData("Left velo", Leftshooter.getVelocity());
            telemetry.update();

            prevA     = gamepad1.a;
            prevStart = gamepad1.start;
            pinpointOdometry.update();
        }
    }

    /**
     * Waits up to 2 seconds for a valid Limelight frame with at least one AprilTag.
     * On success, reseeds pinpoint X, Y, and heading from the botpose.
     * On timeout, leaves pinpoint unchanged and reports the failure reason.
     */
    private void reseedFromLimelight() {
        long deadline = System.currentTimeMillis() + 2000;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && result.getBotpose() != null
                    && result.getBotposeTagCount() > 0) {

                Pose3D botpose = result.getBotpose();
                double llX = botpose.getPosition().x;   // meters, WCS
                double llY = botpose.getPosition().y;   // meters, WCS

                // Convert Limelight WCS (meters) to Pinpoint inches
                // Same formula used in auto pose seeding
                double pinX = llY * 39.37 + 72.0;
                double pinY = -llX * 39.37 + 72.0;

                // Heading: Limelight yaw is CCW-positive in WCS.
                // Pinpoint heading at the audience-wall corner start = 90 deg,
                // so we rotate: heading = 90 - limelight_yaw.
                // NOTE: if heading is consistently off, adjust this formula.
                double llYaw  = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                double heading = llYaw - 90.0;

                pinpointOdometry.setPosX(pinX, DistanceUnit.INCH);
                pinpointOdometry.setPosY(pinY, DistanceUnit.INCH);
                pinpointOdometry.setHeading(heading, AngleUnit.DEGREES);

                telemetry.addData("LL Reset OK", "X=%.1f  Y=%.1f  H=%.1f deg", pinX, pinY, heading);
                telemetry.addData("Tags seen", result.getBotposeTagCount());
                telemetry.update();
                return;
            }
            sleep(33);
        }
        // Timed out — report why the last result was rejected
        LLResult last = limelight.getLatestResult();
        String reason = last == null                    ? "null result"
                      : !last.isValid()                ? "isValid=false"
                      : last.getBotpose() == null      ? "botpose=null"
                      : "tagCount=" + last.getBotposeTagCount();
        telemetry.addData("LL Reset FAILED", reason);
        telemetry.update();
    }

    private void prismTimer() {
        double seconds = runtime.seconds();
        if      (seconds < 10) prism.setPosition(NORMAL_RAINBOW);
        else if (seconds < 20) prism.setPosition(COLOR_GREEN);
        else if (seconds < 25) prism.setPosition(FAST_RAINBOW);
        else if (seconds < 30) prism.setPosition(COLOR_BLUE);
        else                   prism.setPosition(COLOR_OFF);
    }
}
