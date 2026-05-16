package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * TEACHING NOTE:
 * This OpMode demonstrates "Edge Detection" (Debouncing) and State Management.
 * 
 * KEY CONCEPTS:
 * 1. Debouncing: Preventing a single press from being counted multiple times.
 * 2. Toggles: Using one button to switch between two states.
 * 3. State Machine: Using a counter and a Switch-Case to perform different actions.
 * 4. Logic Priority: Ensuring that code only does ONE thing at a time.
 *
 * Key Structurea
 * 1. while loop
 * 2. global variables
 * 3. local variables
 *
 */

@TeleOp(name = "Student Training OpMode", group = "Training")
public class StudentTrainingOpMode extends LinearOpMode {

    /* -------------------------------------------------------------------------
     * SETTINGS (Constants)
     * ------------------------------------------------------------------------- */
    public static final double SERVO_MIN_POS = 0.0;
    public static final double SERVO_MAX_POS = 1.0;
    public static final double SERVO_MID_POS = 0.5;
    
    public static final String SERVO_NAME = "testServo";
    public static final String TOUCH_NAME = "testTouch";

    /* -------------------------------------------------------------------------
     * HARDWARE & STATE VARIABLES
     * ------------------------------------------------------------------------- */
    private Servo trainingServo;
    private TouchSensor trainingTouch;
    
    // Variables for Debouncing (Edge Detection)
    // We store the "previous" state to see if the button JUST changed.
    private boolean prevGamepadA = false;
    private boolean prevTouchState = false;
    private boolean prevGamepadB = false;

    // Variables for Logic State
    private boolean isServoAtMax = false; // Used for the Toggle
    private int touchCount = 0;           // Counts how many times touch was pressed

    @Override
    public void runOpMode() {

        /* ---------------------------------------------------------------------
         * INITIALIZATION
         * --------------------------------------------------------------------- */
        trainingServo = hardwareMap.get(Servo.class, SERVO_NAME);
        trainingServo.setDirection(Servo.Direction.FORWARD);
        // TEACHING NOTE: Advanced Hardware Setting
        // You can reverse the direction of a servo if it's mounted "backwards" on the robot.
        // trainingServo.setDirection(Servo.Direction.REVERSE);

        trainingTouch = hardwareMap.get(TouchSensor.class, TOUCH_NAME);
        trainingServo.setDirection(Servo.Direction.FORWARD);


        telemetry.addData("Status", "Initialized. Press PLAY.");
        telemetry.update();

        waitForStart();

        /* ---------------------------------------------------------------------
         * MAIN LOOP
         * --------------------------------------------------------------------- */
        while (opModeIsActive()) {

            // --- 1. DEBOUNCED TOGGLE (Gamepad A) ---
            // TEACHING NOTE: This logic ensures the servo only toggles once per click.
            if (gamepad1.a && !prevGamepadA) {
                if (isServoAtMax) {
                    moveToPosition(SERVO_MIN_POS);
                    isServoAtMax = false;
                } else {
                    moveToPosition(SERVO_MAX_POS);
                    isServoAtMax = true;
                }
            }
            prevGamepadA = gamepad1.a; // Update history


            // --- 2. DEBOUNCED COUNTER (Touch Sensor) ---
            // TEACHING NOTE: The touch sensor now increments a counter instead of moving the servo directly.
            boolean currentTouch = isTouchPressed();
            if (currentTouch && !prevTouchState) {
                touchCount++; // Increment count on every click
            }
            prevTouchState = currentTouch;


            // --- 3. SWITCH-CASE ACTION (Gamepad B) ---
            // TEACHING NOTE: When B is pressed, we perform an action based on the touch count.
            // This resolves hardware ambiguity by making Gamepad B the "trigger" for the touch logic.
            if (gamepad1.b && !prevGamepadB) {
                
                switch (touchCount) {
                    case 1:
                        moveToPosition(SERVO_MID_POS);
                        isServoAtMax = false;
                        break;

                    case 2:
                        performSweep(SERVO_MIN_POS, SERVO_MAX_POS, 50);
                        isServoAtMax = true; 
                        break;

                    case 3:
                        performSweep(SERVO_MAX_POS, SERVO_MIN_POS, 50);
                        isServoAtMax = false;
                        break;

                    default:
                        // Reset if count is 0 or > 3
                        moveToPosition(SERVO_MIN_POS);
                        isServoAtMax = false;
                        break;
                }

                // Reset the counter after the action is performed
                touchCount = 0;
            }
            prevGamepadB = gamepad1.b;

            /* -----------------------------------------------------------------
             * TELEMETRY
             * ----------------------------------------------------------------- */
            if (isServoAtMax) {
                telemetry.addData("Servo State", "MAX");
            } else {
                telemetry.addData("Servo State", "MIN/MID");
            }
            
            telemetry.addData("Touch Count", touchCount);
            telemetry.addLine("-------------------");
            telemetry.addLine("A: Toggle Servo");
            telemetry.addLine("Touch: Increment Counter");
            telemetry.addLine("B: Run Count Action + Reset Counter");
            telemetry.addLine("Count=1 Move to Mid");
            telemetry.addLine("Count=2 sweep Min to Max");
            telemetry.addLine("Count=3 sweep Max to Min");
            telemetry.addLine("Count=0 or >3 Move to Min");

            telemetry.update();
        }
    }

    /* -------------------------------------------------------------------------
     * METHODS
     * ------------------------------------------------------------------------- */

    private boolean isTouchPressed()
    {
        return trainingTouch.isPressed();
    }

    private void moveToPosition(double pos)
    {
        trainingServo.setPosition(pos);
    }

    private void performSweep(double start, double end, int steps) {
        double currentPos = start;
        double stepSize = (end - start) / steps;

        for (int i = 0; i <= steps; i++) {
            if (!opModeIsActive()) break;

            trainingServo.setPosition(currentPos);
            currentPos += stepSize;
            sleep(20); 
            
            telemetry.addData("Sweep Status", "Moving to %.2f", currentPos);
            telemetry.update();
        }
    }
}
