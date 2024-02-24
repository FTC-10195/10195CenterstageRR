package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;
import org.firstinspires.ftc.teamcode.SubSys.DropDown;
import org.firstinspires.ftc.teamcode.SubSys.PaperAirplane;
import org.firstinspires.ftc.teamcode.SubSys.SimpleBucket;

import java.nio.charset.CoderMalfunctionError;

@TeleOp
@Config
public class uncg extends LinearOpMode {

    public static double armPos = 0;
    public static double dropPos = 0;
    public double planePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "ls");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "rs");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        DropDown intake = new DropDown(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        SimpleBucket bucket = new SimpleBucket(hardwareMap);
        PaperAirplane plane = new PaperAirplane(hardwareMap);
        GamepadEx controller1 = new GamepadEx(gamepad1);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        drive.setDefaultCommand( new ManualDriveCommand(drive,
                controller1::getLeftY,
                controller1::getRightX,
                controller1::getLeftX,
                () -> controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ));
        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            //    CommandScheduler.getInstance().run();

            drive.robotDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, gamepad1.touchpad);
            if (gamepad1.dpad_up) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            } else if (gamepad1.dpad_down) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
            if (gamepad1.dpad_left) {
                armPos = .275;
            } else if (gamepad1.dpad_right) {
                armPos = .9;
            }

            arm.rotate(armPos);
            if (gamepad1.left_bumper) {
                bucket.intakeMode();
            } else if (gamepad1.right_bumper) {
                bucket.outtakeLower();
            } else if (gamepad1.share) {
                bucket.outtakeUpper();
            }

            intake.spin(gamepad2.a, gamepad2.b, gamepad2.x);
            if (gamepad2.dpad_up) {
                intake.intake();
            } else if (gamepad2.dpad_down) {
                intake.idle();
            }
        }






           }
      //  CommandScheduler.getInstance().reset();
         /*
            else if (gamepad1.right_bumper && dropPos != .1) {
                dropPos -= .1;
                intake.manualMove(dropPos);
            }
            intake.spin(gamepad1.triangle, gamepad1.circle, gamepad1.square);

            if (gamepad1.dpad_up) {
                //   if (rightSlide.getCurrentPosition() < 2600) {
                rightSlide.setPower(.25);
                //    } else if (rightSlide.getCurrentPosition() > 2600) {
                leftSlide.setPower(.25);
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

          */

    }



