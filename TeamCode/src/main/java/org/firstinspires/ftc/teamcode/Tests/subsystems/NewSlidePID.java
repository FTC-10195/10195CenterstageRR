package org.firstinspires.ftc.teamcode.Tests.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSys.Arm;

import java.util.logging.Handler;

@TeleOp
@Config

public class NewSlidePID extends LinearOpMode {

    private PIDController controller;

    public static double p  =0.05 , i =0, d = 0.0005;

    public static double f = 0;

    public static int target = 0;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

private ElapsedTime timer;

private int numLoops = 0;
private double sum;
private double average;
    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        controller= new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlide = hardwareMap.get(DcMotorEx.class, "ls");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rs");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Arm arm = new Arm(hardwareMap);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            timer.reset();
            numLoops++;

            arm.rotate(0);
            controller.setPID(p,i,d);
            int pos = leftSlide.getCurrentPosition();

            double pid = controller.calculate(pos, target);

            double power = pid;

            sum += 1000/timer.milliseconds();

            average = sum/numLoops;

            leftSlide.setPower(power);
            rightSlide.setPower(power);
            telemetry.addData("Loop time", 1000/timer.milliseconds());
            telemetry.addData("Pos", pos);
            telemetry.addData("Target", target);
            telemetry.addData("Avg loop (Hz)", average);
            telemetry.addData("Error", target-pos);
            telemetry.update();



        }
    }
}
