package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class autoPathsOnly {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public void Paths(Follower follower) {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.250, 9.250),
                                new Pose(63.000, 21.250)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(63.000, 21.250),
                                new Pose(44.500, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(44.500, 36.000),
                                new Pose(10.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 36.000),
                                new Pose(44.500, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(44.500, 60.000),
                                new Pose(29.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(29.000, 84.000),
                                new Pose(38.250, 109.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(44))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(38.250, 109.500),
                                new Pose(11.500, 137.250)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(44))
                .build();
    }
}
