package org.firstinspires.ftc.teamcode.Tests.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "thingy")
public class AxonSet extends LinearOpMode {
    Servo left;
    Servo right;

    public static double le = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(Servo.class, "ld");
        right = hardwareMap.get(Servo.class, "rd");
        left.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            left.setPosition(le);
            right.setPosition(le);

        }
    }
}
