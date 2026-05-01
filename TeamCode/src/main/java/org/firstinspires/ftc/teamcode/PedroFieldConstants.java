package org.firstinspires.ftc.teamcode;

/**
 * All field coordinates in inches using the Pedro Pathing coordinate system:
 *   Origin  : left audience wall corner (as viewed from audience)
 *   +X axis : toward the right side wall
 *   +Y axis : away from the audience (toward the back wall)
 *   Heading : 0 deg = facing +X (right), 90 deg = facing +Y (into field)
 *
 * Conversion from the old system (field center origin, mm, +X toward audience, +Y toward right):
 *   new_X = old_Y_mm / 25.4 + 72
 *   new_Y = -old_X_mm / 25.4 + 72
 */
public class PedroFieldConstants {

    // -------------------------------------------------------------------------
    // Field dimensions
    // -------------------------------------------------------------------------
    public static final double FIELD_WIDTH_IN = 144.0;
    public static final double FIELD_DEPTH_IN = 144.0;

    // -------------------------------------------------------------------------
    // Shooting target — fixed field location the turret always faces
    // -------------------------------------------------------------------------
    public static final double BLUE_SHOOT_TARGET_X = 13.75;
    public static final double RED_SHOOT_TARGET_X = 144.0-9.75; //1st qual was 7.75
    public static final double SHOOT_TARGET_Y = 137.5;

    // -------------------------------------------------------------------------
    // Predefined start poses  { X_in, Y_in, headingDeg }
    // Camera X/Y is snapped to the nearest of these four to seed pinpoint.
    // -------------------------------------------------------------------------
    // Y = robot back-to-chute distance + clearance = (ROBOT_LENGTH_IN - CHUTE_TO_FRONT_IN) + 0.75
    private static final double AUDIENCE_WALL_START_Y =
            PedroRobotConstants.ROBOT_LENGTH_IN - PedroRobotConstants.CHUTE_TO_FRONT_IN + 0.75;

    // Back wall Y = field depth - robot half-width - clearance
    private static final double BACK_WALL_START_Y =
            FIELD_DEPTH_IN - PedroRobotConstants.ROBOT_HALF_WIDTH_IN - 3;

    /** Audience-wall start, right side  (heading 90 deg = facing into field) */
    public static final double[] BLUE_AUDIENCE_START_POSE_1 = { FIELD_WIDTH_IN / 2.0 - PedroRobotConstants.ROBOT_HALF_WIDTH_IN - 0.5,   AUDIENCE_WALL_START_Y, 90.0 };

    /** Audience-wall start, tile boundry on left side */
    public static final double[] BLUE_AUDIENCE_START_POSE_2 = {FIELD_WIDTH_IN / 2.0 - 24.0 + PedroRobotConstants.ROBOT_HALF_WIDTH_IN + 0.5,   AUDIENCE_WALL_START_Y, 90.0 };

    /** Audience-wall start, tile boundry on right side */
    public static final double[] BLUE_AUDIENCE_START_POSE_3 = { FIELD_WIDTH_IN / 2.0 - 24.0 - PedroRobotConstants.ROBOT_HALF_WIDTH_IN + 0.5,   AUDIENCE_WALL_START_Y, 90.0 };

    /** Back-wall start  (heading 0 deg = facing right toward blue wall) */
    public static final double[] BLUE_BACK_WALL_START_POSE = { 29.0, BACK_WALL_START_Y, 0.0 };

    // Red start poses — mirrored: Red X = 144 - Blue X, heading = (180 - Blue heading + 360) % 360
    /** Audience-wall start, left side  (heading 90 deg = facing into field) */
    public static final double[] RED_AUDIENCE_START_POSE_1 = { FIELD_WIDTH_IN / 2.0 + PedroRobotConstants.ROBOT_HALF_WIDTH_IN + 0.5,   AUDIENCE_WALL_START_Y, 90.0 };
    /** Audience-wall start, tile boundary on right side */
    public static final double[] RED_AUDIENCE_START_POSE_2 = { FIELD_WIDTH_IN / 2.0 + 24.0 - PedroRobotConstants.ROBOT_HALF_WIDTH_IN - 0.5,   AUDIENCE_WALL_START_Y, 90.0 };
    /** Audience-wall start, tile boundary on left side */
    public static final double[] RED_AUDIENCE_START_POSE_3 = { FIELD_WIDTH_IN / 2.0 + 24.0 + PedroRobotConstants.ROBOT_HALF_WIDTH_IN - 0.5, AUDIENCE_WALL_START_Y, 90.0 };
    /** Back-wall start  (heading 180 deg = facing left toward red wall) */
    public static final double[] RED_BACK_WALL_START_POSE  = { 115.0, BACK_WALL_START_Y, 180.0 };

    // -------------------------------------------------------------------------
    // Autonomous shoot positions  { X_in, Y_in, headingDeg }
    // Red X = 144 - Blue X, heading = (180 - Blue heading + 360) % 360
    // -------------------------------------------------------------------------
    public static final double[] BLUE_NEAR_SHOOT_POSE = { 54.0,  10.0, 105.0 };
    public static final double[] BLUE_FAR_SHOOT_POSE  = { 53.0,  90.0, 180.0 };
    public static final double[] RED_NEAR_SHOOT_POSE  = { 90.0,  10.0,  75.0 };
    public static final double[] RED_FAR_SHOOT_POSE   = { 91.0,  90.0,   0.0 };

// -------------------------------------------------------------------------
    // Gate (lever) positions
    //   BLUE: old X=2*25.4 mm, old Y=(-72+10)*25.4 mm
    //   RED:  old X=0 mm,      old Y=1564 mm
    // -------------------------------------------------------------------------
    public static final double BLUE_GATE_X = 12.0;                     // = (-72+10)+72 in
    public static final double BLUE_GATE_Y = 70.0;                     // = -2+72 in
    public static final double RED_GATE_X  = 144-10;    // ≈ 133.6 in
    public static final double RED_GATE_Y  = 71.0;

    // -------------------------------------------------------------------------
    // Ball field wall X (the X-axis wall the floor balls are lined against)
    //   BLUE: old Y=(32-72)*25.4 mm  →  Pedro X = (32-72)+72 = 32 in
    //   RED:  old Y=(72-32)*25.4 mm  →  Pedro X = (72-32)+72 = 112 in
    // -------------------------------------------------------------------------
    public static final double FLOOR_BALLS_BLUE_X = 32.0;
    public static final double FLOOR_BALLS_RED_X  = 144.0-32.0;


    /** Corner sweep Y — robot half-width + 2 inches clearance from audience wall (Y=0), at heading 180 deg */
    public static final double CORNER_Y = PedroRobotConstants.ROBOT_HALF_WIDTH_IN + 2.0;

    public static final double BALL_LINE_1_Y = 36.0;  // ≈ 36.7 in
    public static final double BALL_LINE_2_Y = 60.0;  // ≈ 60.4 in
    public static final double BALL_LINE_3_Y =  84.0;  // ≈ 84.0 in

    // -------------------------------------------------------------------------
    // AprilTag IDs
    // -------------------------------------------------------------------------
    public static final int BALL_LINE_1_TAG_NUM = 21;
    public static final int BALL_LINE_2_TAG_NUM = 22;
    public static final int BALL_LINE_3_TAG_NUM = 23;

    // -------------------------------------------------------------------------
    // End-of-match parking positions — heading is preserved from match start.
    // "Near" = robot was at Y >= 60 (close to back wall / goal side).
    // "Far"  = robot was at Y <  60 (close to audience wall).
    // TODO: set these coordinates before competition
    // -------------------------------------------------------------------------
    public static final double BLUE_END_NEAR_X = 37.0;
    public static final double BLUE_END_NEAR_Y = 8.0;
    public static final double BLUE_END_FAR_X  = 40.0;
    public static final double BLUE_END_FAR_Y  = 120.0;
    public static final double RED_END_NEAR_X  = 144.0-BLUE_END_NEAR_X;
    public static final double RED_END_NEAR_Y  = BLUE_END_NEAR_Y;
    public static final double RED_END_FAR_X   = 144.0-BLUE_END_FAR_X;
    public static final double RED_END_FAR_Y   = BLUE_END_FAR_Y;
}
