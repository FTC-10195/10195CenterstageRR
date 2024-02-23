package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Ateg")
public class AprilTags extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "0");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
        visionPortal.resumeStreaming();
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        double absy = 0;

        for (AprilTagDetection detection : currentDetections) {
            idsFound.append("ID: " + detection.id + ";\n");

            idsFound.append(detection.ftcPose.x);
            idsFound.append(System.getProperty("line.separator"));

            idsFound.append(detection.ftcPose.y);
            idsFound.append(System.getProperty("line.separator"));

            idsFound.append(detection.ftcPose.z);
            idsFound.append(System.getProperty("line.separator"));
            /*idsFound.append(detection.ftcPose.pitch);
            idsFound.append(detection.ftcPose.roll);
            idsFound.append(detection.ftcPose.yaw); */

            idsFound.append(detection.ftcPose.range);
            idsFound.append(System.getProperty("line.separator"));
            //idsFound.append(detection.ftcPose.bearing);
            //idsFound.append(detection.ftcPose.elevation);

            if (detection.id == 1) {
                absy = 41.41;
            } else if (detection.id == 2) {
                absy = 35.41;
            } else if (detection.id == 3) {
                absy = 29.41;
            } else if (detection.id == 4) {
                absy = -29.41;
            } else if (detection.id == 5) {
                absy = -35.41;
            } else if (detection.id == 6) {
                absy = -41.41;
            }

            double robotPositionX = 60.25 - detection.ftcPose.x;
            double robotPositionY = absy - detection.ftcPose.y;
            double robotPositionZ = 4 - detection.ftcPose.z;

            idsFound.append("X: " + robotPositionX);
            idsFound.append(System.getProperty("line.separator"));

            idsFound.append("Y: " + robotPositionY);
            idsFound.append(System.getProperty("line.separator"));

            idsFound.append("Z: " + robotPositionZ);
            idsFound.append(System.getProperty("line.separator"));

            idsFound.append("X: " + detection.ftcPose.x + ";\n");
            idsFound.append("Y: " + detection.ftcPose.y + ";\n");
            idsFound.append("Z: " + detection.ftcPose.z + ";\n");

            idsFound.append("Pitch: " + detection.ftcPose.pitch + ";\n");
            idsFound.append("Roll: " + detection.ftcPose.roll + ";\n");
            idsFound.append("Yaw: " + detection.ftcPose.yaw + ";\n");

            idsFound.append("Range: " + detection.ftcPose.range + ";\n");
            idsFound.append("Bearing: " + detection.ftcPose.bearing + ";\n");
            idsFound.append("Elevation: " + detection.ftcPose.elevation + ";\n");


            idsFound.append(' ');

        }

        telemetry.addData("April Tags", idsFound);
    }

    @Override
    public void start() {
        //visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}
