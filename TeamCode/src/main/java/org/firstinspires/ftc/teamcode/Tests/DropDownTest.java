package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DropDownTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor;
        motor = hardwareMap.get(DcMotorEx.class, "fl");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            motor.setPower(1);

        }
    }
}
