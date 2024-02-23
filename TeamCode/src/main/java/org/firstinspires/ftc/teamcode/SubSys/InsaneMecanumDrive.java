package org.firstinspires.ftc.teamcode.SubSys;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InsaneMecanumDrive implements Subsystem {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Telemetry telemetry;


    public InsaneMecanumDrive(HardwareMap hwmap, Telemetry telemetry) {
        frontLeftMotor = hwmap.dcMotor.get("fl");
        frontRightMotor = hwmap.dcMotor.get("fr");
        backLeftMotor = hwmap.dcMotor.get("bl");
        backRightMotor = hwmap.dcMotor.get("br");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

    }

    public void robotDrive(double left_stick_y, double left_stick_x, double right_stick_x) {

        //denominator is largest motor power to ensure power ratio between motors is mantained
        double denominator = Math.max(Math.abs(left_stick_y) + Math.abs(left_stick_x) + Math.abs(right_stick_x), 1);
        double frontLeftPower = (left_stick_y + left_stick_x + right_stick_x) / denominator;
        double backLeftPower = (left_stick_y - left_stick_x + right_stick_x) / denominator;
        double frontRightPower = (left_stick_y - left_stick_x - right_stick_x) / denominator;
        double backRightPower = (left_stick_y + left_stick_x - right_stick_x) / denominator;

        //set motor power values
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


        telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
        telemetry.addData("BL", backLeftMotor.getCurrentPosition());
        telemetry.addData("BR", backRightMotor.getCurrentPosition());
        telemetry.addData("FR", frontRightMotor.getCurrentPosition());
        telemetry.update();
    }

    public void forward(int distance) {
//reset encoder, go forward x amount of ticks at .1 speed
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor.setTargetPosition(distance);
        backLeftMotor.setTargetPosition(distance);
        frontRightMotor.setTargetPosition(distance);
        frontLeftMotor.setTargetPosition(distance);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(.1);
        backLeftMotor.setPower(.1);
        frontRightMotor.setPower(.1);
        frontLeftMotor.setPower(.1);

        //while loop exists so that the program does not stop the motors while the motors are still running, blocking code
        while (backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
        }
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);


    }

    //literally same as before, just add some negatives bc of mecanum equations
    public void strafe(int rightDistance) {
        //Set the motors to original directions for this code only
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setTargetPosition(-rightDistance);
        backLeftMotor.setTargetPosition(rightDistance);
        frontRightMotor.setTargetPosition(rightDistance);
        frontLeftMotor.setTargetPosition(-rightDistance);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        All of these values are set to .1, but all of these values in strafeLeft() are set to 1
        backRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        frontLeftMotor.setPower(1);
        while (backLeftMotor.isBusy() && backRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy()) {
        }
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


}




