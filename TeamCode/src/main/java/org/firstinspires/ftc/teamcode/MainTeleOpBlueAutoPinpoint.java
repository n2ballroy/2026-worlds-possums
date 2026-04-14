package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

@TeleOp(name = "MainTeleOpBlueAutoPinpoint")
public class MainTeleOpBlueAutoPinpoint extends LinearOpMode {

    private DcMotor Rightshooter;
    private DcMotor Leftshooter;
    private CRServo transfer;
    private DcMotor turret;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontright;
    private DcMotor intake;
    private CRServo middle;
    private GoBildaPinpointDriver pinpointOdometry;

    private Servo prism;
    private ElapsedTime runtime = new ElapsedTime();

    // PWM Constants
    //.1988 is the normal rainbow, .223 is fast rainbow, .0069 is ftc timer
    private static final double NORMAL_RAINBOW = 0.1988;
    private static final double FAST_RAINBOW = 0.223;
    private static final double FTC_TIMER = 0.0069;
    private static final double COLOR_GREEN = 0.388;
    private static final double COLOR_BLUE = 0.611;
    private static final double COLOR_OFF = 0.0;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        int Shooter_Speed;
        double Goal_X;
        double Goal_Y;
        double Delta__X_;
        double Delta__Y_;
        int Aim;
        int Offset;
        float Y;
        float X;
        float RX;
        double X_Pinpoint;
        double Y_Pinpoint;
        double Yaw_Pinpoint;
        double Target_Heading;
        double Distance;
        double Target_Ticks;
        int Shoot=1;
        boolean prevA = false;
        boolean prevStart = false;
        ElapsedTime dpadTimer = new ElapsedTime();

        Rightshooter = hardwareMap.get(DcMotor.class, PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
        Leftshooter = hardwareMap.get(DcMotor.class, PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
        turret = hardwareMap.get(DcMotor.class, PedroRobotConstants.TURRET_MOTOR_CONFIG_NAME);
        intake = hardwareMap.get(DcMotor.class, PedroRobotConstants.INTAKE_CONFIG_NAME);
        transfer = hardwareMap.get(CRServo.class, PedroRobotConstants.TRANSFER_SERVO_CONFIG_NAME);
        prism = hardwareMap.get(Servo.class, "prism");
        pinpointOdometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
/*possums
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        frontright = hardwareMap.get(DcMotor.class, "fr");
*/
        frontLeft = hardwareMap.get(DcMotor.class, "right_rear_drive");
        backLeft = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRight = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontright = hardwareMap.get(DcMotor.class, "left_rear_drive");

        // Setting the Boolean to true reverses the encoder, false leaves it normal
        //pinpointOdometry.reverseEncoders(false, false);
        pinpointOdometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // Resets the current position to 0,0,0 and recalibrates the Odometry Computer's
        // internal IMU. Robot MUST Be stationary. Device takes a large number of samples,
        // and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
        pinpointOdometry.resetPosAndIMU();
        // Recalibrates the Odometry Computer's internal IMU. Robot MUST Be
        // stationary. Device takes a large number of samples, and uses those
        // as the gyroscope zero-offset. This takes approximately 0.25 seconds.
        pinpointOdometry.recalibrateIMU();
        ((DcMotorEx) Rightshooter).setVelocityPIDFCoefficients(150, 0, 0, 14);
        ((DcMotorEx) Leftshooter).setVelocityPIDFCoefficients(150, 0, 0, 14);
        transfer.setDirection(CRServo.Direction.REVERSE);
        Leftshooter.setDirection(DcMotor.Direction.FORWARD);
        Rightshooter.setDirection(DcMotor.Direction.REVERSE);
        Leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Sets the odometry pod positions relative to the point that the odometry computer
        // tracks around.The X pod offset refers to how far sideways from the tracking point
        // the X (forward) odometry pod is. left is positivethe Y Pod offset refers to how far
        // forward from the tracking point the Y (strafe) odometry pod is. Forward increases
        //pinpointOdometry.offsets(DistanceUnit.INCH, 1.875, 7.5);
        //pinpointOdometry.setOffsets(RobotConstants.Y_OFFSET_TO_X_DEADWHEEL_MM, RobotConstants.X_OFFSET_TO_Y_DEADWHEEL_MM, ODOMETRY_DISTANCE_UNIT);  //see gobilda pinpoint manual
        pinpointOdometry.setOffsets(1.0,7.5,DistanceUnit.INCH);  //see gobilda pinpoint manual
        Shooter_Speed = 1600;
        Goal_X = PedroFieldConstants.BLUE_SHOOT_TARGET_X;
        Goal_Y = PedroFieldConstants.SHOOT_TARGET_Y;
        Delta__X_ = 0;
        Delta__Y_ = 0;
        Aim = 0;
        ((DcMotorEx) turret).setPositionPIDFCoefficients(12.5);
        transfer.setDirection(CRServo.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        telemetry.setMsTransmissionInterval(11);
        ((DcMotorEx) turret).setVelocity(1800);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setTargetPosition(0);
        ((DcMotorEx) turret).setTargetPositionTolerance(0);
        Offset = 0;

        if (QuickOdometryStorage.valid) {
            //set location stored from last autonomous
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
            // Call the automated timer method
            prismTimer();
            ((DcMotorEx) turret).setVelocity(1800);
            Y = gamepad1.left_stick_y;
            X = gamepad1.left_stick_x;
            RX = gamepad1.right_stick_x;
            // Returns x position the unit of your choice
            X_Pinpoint = pinpointOdometry.getPosX(DistanceUnit.INCH);
            // Returns y position the unit of your choice
            Y_Pinpoint = pinpointOdometry.getPosY(DistanceUnit.INCH);
            // Returns the direction your robot is facing the unit of your choice
            Yaw_Pinpoint = pinpointOdometry.getHeading(AngleUnit.DEGREES);
            // Returns x position the unit of your choice
            Delta__X_ = Goal_X - pinpointOdometry.getPosX(DistanceUnit.INCH);
            // Returns y position the unit of your choice
            Delta__Y_ = Goal_Y - pinpointOdometry.getPosY(DistanceUnit.INCH);
            Target_Heading = Math.atan2(Delta__Y_, Delta__X_) / Math.PI * 180;
            Distance = Math.sqrt(Math.pow(Delta__X_, 2) + Math.pow(Delta__Y_, 2));
            Shooter_Speed = (int) (1160 + Distance * 3.5);
            if ((gamepad1.dpad_left || gamepad1.dpad_right) && dpadTimer.milliseconds() > 150) {
                if (gamepad1.dpad_left) {
                    Offset++;
                } else {
                    Offset--;
                }
                dpadTimer.reset();
            }
            if (gamepad1.start && !prevStart) {
                // Send a position that the Pinpoint should use to track your robot relative
                // to. You can use this to update the estimated position of your robot
                // with new external sensor data, or to run a robot in field coordinates.
                //pinpointOdometry.position(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);
                pinpointOdometry.setPosX(0.0, DistanceUnit.INCH);
                pinpointOdometry.setPosY(0.0, DistanceUnit.INCH);
                pinpointOdometry.setHeading(90.0, AngleUnit.DEGREES);
                sleep(500);
            }
            double rawError = Math.toRadians(Target_Heading - Yaw_Pinpoint + Offset);
            double headingError = Math.toDegrees(Math.atan2(Math.sin(rawError), Math.cos(rawError)));
            Target_Ticks = PedroRobotConstants.TURRET_TICKS_PER_DEG * headingError;
            if (gamepad1.right_bumper) {
                intake.setPower(0.8);
                middle.setPower(1);
                transfer.setPower(1);
            } else {
                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                middle.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                transfer.setPower(0);
            }
            if (gamepad1.a && !prevA) {
                if (Aim == 0) {
                    Aim = 1;  // enable auto-aim
                } else {
                    Aim = 0;  // disable auto-aim
                }
            }
            if (Aim == 0) {
                turret.setTargetPosition(0);
            } else {
                turret.setTargetPosition((int) Math.min(Math.max(Target_Ticks, -1400), 1400));
            }
            if (gamepad1.b) {
                Shoot = 1;
            } else if (gamepad1.x) {
                Shoot = 0;
            }
            if (Shoot == 1) {
                ((DcMotorEx) Leftshooter).setVelocity(Shooter_Speed);
                ((DcMotorEx) Rightshooter).setVelocity(Shooter_Speed);
            } else {
                ((DcMotorEx) Rightshooter).setVelocity(0);
                ((DcMotorEx) Leftshooter).setVelocity(0);
            }
            frontLeft.setPower((Y - X) - RX);
            backLeft.setPower((Y + X) - RX);
            frontright.setPower(Y + X + RX);
            backRight.setPower((Y - X) + RX);
            telemetry.addData("Target Heading", Double.parseDouble(JavaUtil.formatNumber(Target_Ticks, 3)));
            telemetry.addData("Shooter", Shooter_Speed);
            telemetry.addData("`Aim", Aim);
            telemetry.addData("Delta X", Delta__X_);
            telemetry.addData("Delta Y", Delta__Y_);
            telemetry.addData("Distance", Distance);
            // Returns x position the unit of your choice
            telemetry.addData("X", pinpointOdometry.getPosX(DistanceUnit.INCH));
            // Returns y position the unit of your choice
            telemetry.addData("Y", pinpointOdometry.getPosY(DistanceUnit.INCH));
            telemetry.addData("Left velo", ((DcMotorEx) Leftshooter).getVelocity());
            telemetry.update();
            prevA = gamepad1.a;
            prevStart = gamepad1.start;
            // Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
            pinpointOdometry.update();
        }
    }
    private void prismTimer() {
        double seconds = runtime.seconds();

        // Predefined time mapping
        if (seconds < 10) {
            // First 10 seconds: Rainbow
            prism.setPosition(NORMAL_RAINBOW);
        } else if (seconds < 20) {
            // 10 to 20 seconds: Green
            prism.setPosition(COLOR_GREEN);
        } else if (seconds < 25) {
            // 20 to 25 seconds: Red (Warning!)
            prism.setPosition(FAST_RAINBOW);
        } else if (seconds < 30) {
            // 25 to 30 seconds: Blue
            prism.setPosition(COLOR_BLUE);
        } else {
            // After 30 seconds: Off
            prism.setPosition(COLOR_OFF);
        }
    }
}
