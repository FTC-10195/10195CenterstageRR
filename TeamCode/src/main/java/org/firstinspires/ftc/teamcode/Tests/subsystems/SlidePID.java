package org.firstinspires.ftc.teamcode.Tests.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSys.Arm;

import java.util.ArrayList;
import java.util.List;



@TeleOp
@Config

public class SlidePID extends LinearOpMode {

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    public static double Kp = .04;
    public static double Ki = 0.005;
    public static double Kd= 0.001;
    public static int target = 0;
    public  static double Kf = 0;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    PIDFController controller = new PIDFController(Kp, Ki, Kd, Kf);

//    PIDFController controllerRight = new PIDFController(Kp, Ki, Kd, Kf);

   //   Arm arm = new Arm(hardwareMap);
    FtcDashboard dashboard=  FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
Arm arm = new Arm(hardwareMap);
        arm.rotate(0);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        controller.setPIDF(Kp, Ki, Kd, Kf);
     //   controllerRight.setPIDF(Kp, Ki, Kd, Kf);
        timer.reset();
       // Arm arm = new Arm(hardwareMap);
        TelemetryPacket pack = new TelemetryPacket();

        controller.setTolerance(50);
      //  controllerRight.setTolerance(50);
        leftSlide = hardwareMap.get(DcMotorEx.class, "ls");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rs");
double leftPower = 0;
double rightPower =0;
double previousLeftPower;
double previousRightPower;
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {

            timer.reset();
            previousRightPower = rightPower;
            previousLeftPower = leftPower;

            for (LynxModule hub: allHubs) {
               hub.clearBulkCache();
            }


            int leftPos = leftSlide.getCurrentPosition();
            int rightPos = rightSlide.getCurrentPosition();



             leftPower = controller.calculate(leftPos, target);
           //  rightPower = controllerRight.calculate(rightPos, target);


            if(Math.abs(leftPower-previousLeftPower) > .05) {leftSlide.setPower(leftPower); rightSlide.setPower(leftPower);}
           // if(Math.abs(rightPower-previousRightPower) > .05) {rightSlide.setPower(rightPower);}

            telemetry.addData("Calculate left", leftPower);
            telemetry.addData("Calculate right", rightPower);
            telemetry.addData("Previous Left", previousLeftPower);
            telemetry.addData("Previous right", previousRightPower);

            telemetry.addData("Left", leftPos);
            telemetry.addData("Right", rightPos);
            telemetry.addData("Target", target);

            pack.put("Position", leftPos);
            pack.put("Target", target);
            pack.put("Error", target-leftPos);
            pack.put("POwer diff", leftPower-previousLeftPower);
            dashboard.sendTelemetryPacket(pack);
            telemetry.addData("Loop time", 1000/timer.milliseconds());
            telemetry.update();

        }
    }
}
