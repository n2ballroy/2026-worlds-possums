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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

 /**
  * Alliance-aware TeleOp with Limelight pinpoint reseeding on START.
  *
  * TO CREATE RED VERSION: copy this file, then make exactly these 3 changes:
  *   1. Rename file and class to Worlds_2Color_Red_TeleOp
  *   2. Change @TeleOp: name="Worlds_2Color_Red_TeleOp"
  *   3. Change:  private static final boolean BLUE_ALLIANCE = false;
  * No other code changes are needed.
  *
  * GAMEPAD1 START — point camera at a visible AprilTag and press START.
  *   The robot waits up to 2 seconds for a valid tagged frame, then reseeds
  *   X, Y, and heading from the Limelight botpose.
  *
  * Coordinate conversion:
  *   Pinpoint X (in) = limelight_Y (m) * 39.37 + 72
  *   Pinpoint Y (in) = -limelight_X (m) * 39.37 + 72
  *   Heading (deg)   = limelight_yaw - 90
  *   Both systems CCW-positive. Limelight 0° = audience wall = Pedro +Y = Pedro 90°.
  *   Pedro 0° = blue wall = Limelight +Y = Limelight 90°. Offset = -90°.
  *   This conversion is alliance-independent (both use absolute field coordinates).
  */
 // *** RED VERSION: change name="RPM_Shots_2Color_Red_TeleOp" ***
 @TeleOp(name = "RPM_Shots_2Color_Red_TeleOp", group = "TeleOp")
 public class RPM_Shots_2Color_Red_TeleOp extends LinearOpMode {

     // *** ONLY LINE TO CHANGE FOR RED ALLIANCE ***
     private static final boolean BLUE_ALLIANCE = false;

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

     // Shooting constants
     private static final double INTAKE_POWER               = 1.0;
     private static final double TRANSFER_POWER             = 0.8;
     private static final double SHOOTER_READY_MAX_WAIT_SEC = 2.0;
     private static final double SHOT_DETECT_TIMEOUT_SEC    = 1.5;
     private static final double SHOT_RPM_DROP_FRACTION     = 0.80;

     public DcMotorEx getIntake() {
         return intake;
     }

     // PWM Constants
     private static final double NORMAL_RAINBOW = 0.1988;
     private static final double FAST_RAINBOW   = 0.223;
     private static final double FTC_TIMER      = 0.0069;
     private static final double COLOR_GREEN    = 0.388;
     private static final double COLOR_BLUE     = 0.611;
     private static final double COLOR_OFF      = 0.0;

     // Shooting state — RPM-gated long-shot state machine
     private int         shootSubStep   = 0;
     private boolean     isDelayRunning = false;
     private ElapsedTime delayTimer     = new ElapsedTime();
     private int         Shooter_Speed  = 0;
     private double      Target_Ticks   = 0;

     @Override
     public void runOpMode() {
         // Alliance-specific shoot target X — only conditional in this file
         double Goal_X = BLUE_ALLIANCE
                 ? PedroFieldConstants.BLUE_SHOOT_TARGET_X
                 : PedroFieldConstants.RED_SHOOT_TARGET_X;

         double Goal_Y     = PedroFieldConstants.SHOOT_TARGET_Y;
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
         int    Shoot    = 1;
         boolean prevA     = false;
         boolean prevStart = false;
         ElapsedTime dpadTimer = new ElapsedTime();
         double turretInitOffsetTicks =0;
         Rightshooter     = hardwareMap.get(DcMotorEx.class,            PedroRobotConstants.RIGHT_LAUNCHER_CONFIG_NAME);
         Leftshooter      = hardwareMap.get(DcMotorEx.class,            PedroRobotConstants.LEFT_LAUNCHER_CONFIG_NAME);
         turret           = hardwareMap.get(DcMotorEx.class,            PedroRobotConstants.TURRET_MOTOR_CONFIG_NAME);
         intake           = hardwareMap.get(DcMotorEx.class,            PedroRobotConstants.INTAKE_CONFIG_NAME);
         transfer         = hardwareMap.get(CRServo.class,              PedroRobotConstants.TRANSFER_SERVO_CONFIG_NAME);
         prism            = hardwareMap.get(Servo.class,                "prism");
         pinpointOdometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
         limelight        = hardwareMap.get(Limelight3A.class,          "limelight");
 //

         frontLeft  = hardwareMap.get(DcMotor.class, "lf");
         backLeft   = hardwareMap.get(DcMotor.class, "rf");
         backRight  = hardwareMap.get(DcMotor.class, "rb");
         frontright = hardwareMap.get(DcMotor.class, "LB");

         pinpointOdometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
         pinpointOdometry.resetPosAndIMU();
         pinpointOdometry.recalibrateIMU();
         pinpointOdometry.setOffsets(-5.46, -2.7, DistanceUnit.INCH);

         Rightshooter.setVelocityPIDFCoefficients(230, 0, 0, 13);
         Leftshooter.setVelocityPIDFCoefficients(230, 0, 0, 13);
         transfer.setDirection(CRServo.Direction.REVERSE);
         Leftshooter.setDirection(DcMotor.Direction.FORWARD);
         Rightshooter.setDirection(DcMotor.Direction.REVERSE);
         Leftshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         Rightshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         Leftshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         Rightshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         Shooter_Speed = 1600;
         Delta__X_ = 0;
         Delta__Y_ = 0;
         Aim    = 0;
         Offset = 0;

         turret.setPositionPIDFCoefficients(12.5);
         transfer.setDirection(CRServo.Direction.REVERSE);
         turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontLeft.setDirection(DcMotor.Direction.REVERSE);
         backLeft.setDirection(DcMotor.Direction.REVERSE);
         backRight.setDirection(DcMotor.Direction.FORWARD);
         frontright.setDirection(DcMotor.Direction.FORWARD);
         telemetry.setMsTransmissionInterval(11);
         turret.setVelocity(1800);
         turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         turret.setTargetPosition(0);
         turret.setTargetPositionTolerance(0);

         limelight.pipelineSwitch(PedroRobotConstants.GOALS_PIPELINE);
         limelight.start();
         telemetry.addData("quick valid",QuickOdometryStorage.valid);
         telemetry.update();
         sleep(5000);
         if (QuickOdometryStorage.valid) {
             QuickOdometryStorage.valid = false;
             pinpointOdometry.setPosX(QuickOdometryStorage.x, DistanceUnit.INCH);
             pinpointOdometry.setPosY(QuickOdometryStorage.y, DistanceUnit.INCH);
             pinpointOdometry.setHeading(QuickOdometryStorage.heading, AngleUnit.DEGREES);
             turretInitOffsetTicks=QuickOdometryStorage.turretDegrees * PedroRobotConstants.TURRET_TICKS_PER_DEG;
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
             Y  = gamepad1.left_stick_x;
             X  = gamepad1.left_stick_y;
             RX = -gamepad1.right_stick_x;

             X_Pinpoint   = pinpointOdometry.getPosX(DistanceUnit.INCH);
             Y_Pinpoint   = pinpointOdometry.getPosY(DistanceUnit.INCH);
             Yaw_Pinpoint = pinpointOdometry.getHeading(AngleUnit.DEGREES);
             Delta__X_    = Goal_X - X_Pinpoint;
             Delta__Y_    = Goal_Y - Y_Pinpoint;
             Target_Heading = Math.toDegrees(Math.atan2(Delta__Y_, Delta__X_));
             Distance       = Math.sqrt(Math.pow(Delta__X_, 2) + Math.pow(Delta__Y_, 2));
             Shooter_Speed  = (int) (1160.0 + Distance * 3.75);

             if ((gamepad1.dpad_left || gamepad1.dpad_right) && dpadTimer.milliseconds() > 150) {  //about 6 deg/sec when held down
                 if (gamepad1.dpad_left) Offset++;
                 else                   Offset--;
                 dpadTimer.reset();
             }

             // START — reseed pinpoint from Limelight (robot must see an AprilTag)
             if (gamepad1.start && !prevStart) {
                 reseedFromLimelight();
             }
             prevStart = gamepad1.start;

             double rawError    = Math.toRadians(Target_Heading - Yaw_Pinpoint + Offset);
             double headingError = Math.toDegrees(Math.atan2(Math.sin(rawError), Math.cos(rawError)));
             Target_Ticks = PedroRobotConstants.TURRET_TICKS_PER_DEG * headingError - turretInitOffsetTicks;

             if (gamepad1.right_bumper) {
                 if (Y_Pinpoint >= 60.0) {
                     intake.setPower(-INTAKE_POWER);
                     transfer.setPower(TRANSFER_POWER);
                 } else {
                     executeLongShot();
                 }
             } else {
                 shootSubStep   = 0;
                 isDelayRunning = false;
                 intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                 transfer.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
             }

             if (gamepad1.a && !prevA) {
                 if(Aim == 0){
                     Aim = 1;
                 }
                 else{
                     Aim = 0;
                 }
             }
             prevA     = gamepad1.a;

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
             telemetry.addData("Heading deg",Yaw_Pinpoint);
             telemetry.addData("Turret deg", (turret.getCurrentPosition()+turretInitOffsetTicks)/PedroRobotConstants.TURRET_TICKS_PER_DEG);
             telemetry.addData("Left velo", Leftshooter.getVelocity());
             telemetry.update();

             pinpointOdometry.update();
         }
     }

     /**
      * Waits up to 2 seconds for a valid Limelight frame with at least one AprilTag.
      * On success, reseeds pinpoint X, Y, and heading from the botpose.
      * Limelight WCS → Pedro conversion is alliance-independent (both are absolute field coords).
      */
     private void reseedFromLimelight() {
         long deadline = System.currentTimeMillis() + 2000;
         while (opModeIsActive() && System.currentTimeMillis() < deadline) {
             LLResult result = limelight.getLatestResult();
             if (result != null && result.isValid()
                     && result.getBotpose() != null
                     && result.getBotposeTagCount() > 0) {

                 Pose3D botpose = result.getBotpose();
                 double llX = botpose.getPosition().x;
                 double llY = botpose.getPosition().y;

                 double pinX   = llY * 39.37 + 72.0;
                 double pinY   = -llX * 39.37 + 72.0;
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
         LLResult last = limelight.getLatestResult();
         String reason = last == null                    ? "null result"
                       : !last.isValid()                ? "isValid=false"
                       : last.getBotpose() == null      ? "botpose=null"
                       : "tagCount=" + last.getBotposeTagCount();
         telemetry.addData("LL Reset FAILED", reason);
         telemetry.update();
     }

     private void executeLongShot() {
         switch (shootSubStep) {
             case 0:
                 intake.setPower(0);
                 transfer.setPower(0);
                 if ((rpmReadyToShoot() && turretAtTarget())
                         || nonBlockingDelay(SHOOTER_READY_MAX_WAIT_SEC)) {
                     isDelayRunning = false;
                     shootSubStep   = 1;
                 }
                 break;
             case 1:
                 intake.setPower(-INTAKE_POWER);
                 transfer.setPower(TRANSFER_POWER);
                 if (shotDetected()) {
                     isDelayRunning = false;
                     intake.setPower(0);
                     transfer.setPower(0);
                     shootSubStep = 0;
                 } else if (nonBlockingDelay(SHOT_DETECT_TIMEOUT_SEC)) {
                     isDelayRunning = false;
                     shootSubStep   = 0;
                 }
                 break;
         }
     }

     private boolean rpmReadyToShoot() {
         return Shooter_Speed > 933
                 && Math.abs(Shooter_Speed - Rightshooter.getVelocity()) <= 94;
     }

     private boolean turretAtTarget() {
         double toleranceTicks = PedroRobotConstants.TURRET_ACCURACY_DEG
                                 * PedroRobotConstants.TURRET_TICKS_PER_DEG;

         return Math.abs(turret.getCurrentPosition() - Target_Ticks) <= toleranceTicks;
     }

     private boolean shotDetected() {
         return Rightshooter.getVelocity() < Shooter_Speed * SHOT_RPM_DROP_FRACTION;
     }

     private boolean nonBlockingDelay(double sec) {
         if (!isDelayRunning) { delayTimer.reset(); isDelayRunning = true; return false; }
         if (delayTimer.seconds() < sec) return false;
         isDelayRunning = false;
         return true;
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
