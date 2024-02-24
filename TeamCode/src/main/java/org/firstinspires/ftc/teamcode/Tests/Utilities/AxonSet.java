package org.firstinspires.ftc.teamcode.Tests.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "thingy")
public class AxonSet extends LinearOpMode {
    Servo drop;
  //  Servo right;

    DcMotorEx intake;
    public static double le = 0;

    public double intakePos = .5;
    public double intakePower = -1;
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drop = hardwareMap.get(Servo.class, "drop");
        intake = hardwareMap.get(DcMotorEx.class, "spin");
       // drop.setDirection(Servo.Direction.REVERSE);
       // right = hardwareMap.get(Servo.class, "rd");
      //  left.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            intake.setPower(power);
            drop.setPosition(le);
          //  right.setPosition(le);

        }
    }
}
