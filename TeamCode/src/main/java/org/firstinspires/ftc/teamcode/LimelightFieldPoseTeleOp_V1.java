package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight Field Pose V1", group = "Sensor")
public class LimelightFieldPoseTeleOp_V1 extends LinearOpMode {

    private Limelight3A limelight;

    // Pipeline 1 for red AprilTags, 2 for blue AprilTags
    private final int GOALS_PIPELINE = 0;
    private int activePipeline = GOALS_PIPELINE;
    private double lastTimeStampStep=0;
    private double lastTimeControlHubStep=0;
    private double lastDistanceToGoal=0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(activePipeline);
        sleep(500);
        limelight.start();
        sleep(500);


        waitForStart();


        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result.isValid() &&
                    result.getBotpose() != null &&
                    result.getBotposeTagCount() > 0){ // &&
                    //(result.getTimestamp()-lastTimeStampStep)>20.0) {
                Pose3D botpose = result.getBotpose();



                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                double tagDistanceRelativeToCamera =-1;
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult firstTag = fiducials.get(0); // Get data for the first detected tag

                    tagDistanceRelativeToCamera = Math.sqrt(Math.pow(firstTag.getRobotPoseTargetSpace().getPosition().z*1000.0,2)+
                            Math.pow(firstTag.getRobotPoseTargetSpace().getPosition().x*1000.0,2));

                    if(Math.abs(tagDistanceRelativeToCamera-lastDistanceToGoal)!=0){
                        telemetry.addData("Botpose", "X: %.0f mm, Y: %.0f mm, orient: %.0f deg",
                                botpose.getPosition().x*1000.0, botpose.getPosition().y*1000.0, botpose.getOrientation().getYaw());

                        // Total latency is the pipeline latency plus the timestamp age
                        telemetry.addData("Latency", "%.2f ms", result.getCaptureLatency());
                        telemetry.addData("control hub - camera delta", "%.2f ms", result.getControlHubTimeStamp()-lastTimeControlHubStep);
                        telemetry.addData("Time Stamp Delta", "%.2f ms", result.getTimestamp()-lastTimeStampStep);
                        telemetry.addData("Staleness", "%d ms", result.getStaleness());
                        telemetry.addData("angle to goal(+deg ccw)", "%.2f ms", -result.getTx());
                        telemetry.addData("distance to goal", "%.2f in", tagDistanceRelativeToCamera/25.4);
                        telemetry.addData("X/Y Pedro coords","%.0f / %.0f in",botpose.getPosition().y*39.37+72,-botpose.getPosition().x*39.37+72);
                        lastTimeControlHubStep=result.getControlHubTimeStamp();

                    }else{
                        telemetry.addData("Limelight", "No new distance  data");
                    }


                }



            } else {
                telemetry.addData("Limelight", "No valid data");
            }
            lastTimeStampStep=result.getTimestamp();
            telemetry.update();
        }
        limelight.stop();
    }
}
