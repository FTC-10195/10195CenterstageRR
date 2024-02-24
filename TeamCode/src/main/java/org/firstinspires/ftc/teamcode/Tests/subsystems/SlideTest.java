package org.firstinspires.ftc.teamcode.Tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;

@TeleOp
@Config
public class SlideTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Slides slide = new Slides(hardwareMap, telemetry);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            slide.manualControl(gamepad1.left_trigger > .05, gamepad1.right_trigger > .05);

            if(gamepad1.dpad_down) {
           //     slide.setTarget(Slides.SlideStates.MINIMUM);
            }
            telemetry.update();
        }
    }
}

