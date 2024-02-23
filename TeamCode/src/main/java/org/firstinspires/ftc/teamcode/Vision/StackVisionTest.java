package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class StackVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        StackTest stack = new StackTest();

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), stack);

        telemetry.clear();

        while (opModeInInit()) {
            telemetry.addData("# in stack",stack.getSelection());
            telemetry.update();

        }
    }

}
