package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class DistanceFromAprilTags extends LinearOpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        int distanceFromAprilTag = detection.rawPose.y.distance;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            WebcamName webcamName = hardwareMap.get(WebcamName.class, "0");
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
            visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
            visionPortal.resumeStreaming();

            telemetry.addData("Distance from AprilTag", distanceFromAprilTag);

            }
    }
}
