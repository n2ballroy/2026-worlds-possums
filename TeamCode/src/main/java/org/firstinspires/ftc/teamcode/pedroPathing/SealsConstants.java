package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SealsConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0)) // tuned constants
           // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016)) // (kP, kLinear, kQuadratic)
            .centripetalScaling(0)
            .forwardZeroPowerAcceleration(-33.63)
            .lateralZeroPowerAcceleration(-52.74);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("right_rear_drive")
            .leftRearMotorName("right_front_drive")
            .rightFrontMotorName("left_rear_drive")
            .rightRearMotorName("left_front_drive")
            .xVelocity(66)
            .yVelocity(57);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(2)       // TODO: measure forward pod offset from robot center (inches)
            .strafePodX(6)     // TODO: measure strafe pod offset from robot center (inches)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            500,
            0.75,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
