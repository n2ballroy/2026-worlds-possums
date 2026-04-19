package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class PossumsConstants {

   /****example mecanum PIDs
    * .useSecondaryTranslationalPIDF(false)
    *             .useSecondaryHeadingPIDF(false)
    *             .useSecondaryDrivePIDF(false)
    *             .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.03))
    *             .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.02, 0.03))
    *             .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0, 0.001))
    */
   public static FollowerConstants followerConstants = new FollowerConstants()
           .forwardZeroPowerAcceleration(-28.0)
           .lateralZeroPowerAcceleration(-54.0)
//     (new PredictiveBrakingCoefficients(0.2, 0.08, 0.0016)) // (kP, kLinear, kQuadratic)
           .translationalPIDFCoefficients(new PIDFCoefficients(.03,0,0,0.04))
           .headingPIDFCoefficients(new PIDFCoefficients(1,0,0,.03))
           .drivePIDFCoefficients(new FilteredPIDFCoefficients(.01,0,.0001,.6,.03))  //consider  P due to strafe arrivals
           .mass(10.6);

   //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
   /**
    These are the PathConstraints in order:
    tValueConstraint=above this portion of path length ends the path follow ,
    timeoutConstraint=msec to correct after tValueConstraint is met,  basically this is msec to correct target errors
    velocityConstraint=below this in/sec required to stop the path follow,
    translationalConstraint=below ths inch error required to stop the path follow,
    headingConstraint=below this radians error required to stop the path follow,
    ****the 3 above constraints must all be met before the path follow is stopped*****
    * brakingStrength= a lower value will brake harder to compensate for strafe arrivals being weaker and might not stop quick enough
    * lower braking strength to stop overshoot
    BEZIER_CURVE_SEARCH_LIMIT=leave at 10,
    brakingStart= recommended 1 ...greater than 1 makes braking start earlier
    The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
    */
   public static PathConstraints pathConstraints = new PathConstraints(
           0.995,
           0.2,
           0.25,
           0.02,
           200,
           1.25,
           10,
           1.0
   );
   public static MecanumConstants driveConstants = new MecanumConstants()
           .xVelocity(79.24)
           .yVelocity(59.0)


           .maxPower(1)
           .rightFrontMotorName("rf")
           .rightRearMotorName("rb")
           .leftRearMotorName("LB")
           .leftFrontMotorName("lf")
           .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
           .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
           .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
           .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
   public static PinpointConstants localizerConstants = new PinpointConstants()
           .forwardPodY(-5.46)
           .strafePodX(-2.7)
           .distanceUnit(DistanceUnit.INCH)
           .hardwareMapName("pinpoint")
           .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
           .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
           .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
   public static Follower createFollower(HardwareMap hardwareMap) {
       return new FollowerBuilder(followerConstants, hardwareMap)
               .pathConstraints(pathConstraints)
               .pinpointLocalizer(localizerConstants)
               .mecanumDrivetrain(driveConstants)
               .build();
   }
}
