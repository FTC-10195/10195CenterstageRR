package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;
import org.firstinspires.ftc.teamcode.SubSys.DropDown;
import org.firstinspires.ftc.teamcode.SubSys.PaperAirplane;
import org.firstinspires.ftc.teamcode.SubSys.SimpleBucket;

@TeleOp
@Config

public class uncg extends LinearOpMode {

    public static double armPos = 0;
    public static double dropPos = 0;
    public double planePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
      //  frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      //  backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "ls");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rs");


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        DropDown intake = new DropDown(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        SimpleBucket bucket = new SimpleBucket(hardwareMap);
        PaperAirplane plane = new PaperAirplane(hardwareMap);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);
            double frontLeftPower = (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator;
            double backLeftPower = (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator;
            double frontRightPower = (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator;
            double backRightPower = (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator;

            //set motor power values
            frontLeftMotor.setPower(frontLeftPower); // This is oriented by looking straight on at the robot, so this would be the wheel closest to the slides on the control hub side
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.left_bumper && dropPos != .5) {
                dropPos += .1;
                intake.manualMove(dropPos);
            }
            else if (gamepad1.right_bumper && dropPos != .1) {
                dropPos -= .1;
                intake.manualMove(dropPos);
            }
            intake.spin(gamepad1.triangle, gamepad1.circle, gamepad1.square);

            if (gamepad1.dpad_up) {
                //   if (rightSlide.getCurrentPosition() < 2600) {
                rightSlide.setPower(1);
                //    } else if (rightSlide.getCurrentPosition() > 2600) {
                leftSlide.setPower(1);
                //   }
                //    if (leftSlide.getCurrentPosition() < 2150) {
                //  leftSlide.setPower(1);
            }
            //  else if (leftSlide.getCurrentPosition() > 2150) {

            else if (gamepad1.dpad_down) {
                rightSlide.setPower(-1);
                leftSlide.setPower(-1);

            } else {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }

            if(gamepad1.dpad_left) {
                armPos = .6;
            } else if (gamepad1.dpad_right) {
                armPos = 0;
            }

            arm.rotate(armPos);

            if(gamepad1.right_stick_button) {
                bucket.intakeMode();
            }
            else if(gamepad1.left_stick_button) {
                bucket.outtakeLower();
                bucket.outtakeUpper();
            }

            if(gamepad1.cross) {
                planePos += .1;

                if(planePos == 1) {
                    planePos = 0;
                }
                plane.setPosition(planePos);
            }

          //  bucket.manualMove(armPos);
            telemetry.addData("Left slide", leftSlide.getCurrentPosition());
            telemetry.addData("Right slide", rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }

}
