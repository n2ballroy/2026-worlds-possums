package org.firstinspires.ftc.teamcode;

/**
 * All robot constants needed by Match7 and Pedro-based code.
 * All distances in inches. Chute outlet is the robot reference point (0,0).
 */
public class PedroRobotConstants {

    // -------------------------------------------------------------------------
    // Hardware config names
    // -------------------------------------------------------------------------
    public static final String INTAKE_CONFIG_NAME             = "intake";
    public static final String TRANSFER_SERVO_CONFIG_NAME     = "transfer";
    public static final String RIGHT_LAUNCHER_CONFIG_NAME     = "right shooter";
    public static final String LEFT_LAUNCHER_CONFIG_NAME      = "leftshooter";
    public static final String TURRET_MOTOR_CONFIG_NAME       = "turret";

    // -------------------------------------------------------------------------
    // Limelight pipelines
    // -------------------------------------------------------------------------
    public static final int GOALS_PIPELINE       = 0;
    public static final int BRIGHT_PIPELINE_ADDER = 5;

    // -------------------------------------------------------------------------
    // Robot geometry (inches) — chute outlet is robot reference (0,0)
    // Chute is centered side-to-side and CHUTE_TO_FRONT_IN behind the robot front
    // -------------------------------------------------------------------------
    public static final double CHUTE_TO_FRONT_IN             = 14.01;
    public static final double ROBOT_HALF_WIDTH_IN           = 8.0;
    public static final double ROBOT_LENGTH_IN               = 18.0;

    // -------------------------------------------------------------------------
    // Intake and chute geometry (inches)
    // -------------------------------------------------------------------------
    public static final double CHUTE_TO_PUSHER_RIGHT_IN      = 0.0;  // ≈  4.92 in
    public static final double CHUTE_TO_PUSHER_LEFT_IN       = 0.0;  // ≈  5.31 in

    // -------------------------------------------------------------------------
    // Turret — DC motor with encoder, 0 deg = firing toward robot front
    // RUN_TO_POSITION mode; encoder resets to 0 at match start (facing forward)
    // -------------------------------------------------------------------------
    public static final double TURRET_TICKS_PER_REV          = 2680.0;
    public static final double TURRET_TICKS_PER_DEG          = TURRET_TICKS_PER_REV / 360.0;
    public static final double TURRET_PIDF                   = 12.5;


    // -------------------------------------------------------------------------
    // Timing
    // -------------------------------------------------------------------------
    public static final double RPM_SETTLE_TIME_SECONDS       = 0.125;
    public static final double MAX_RPM_TURRET_WAIT_SEC       = 1.5;   // max wait after stop for RPM+turret
    public static final double TURRET_ACCURACY_DEG           = 2.0;   // turret considered on-target within this angle


}
