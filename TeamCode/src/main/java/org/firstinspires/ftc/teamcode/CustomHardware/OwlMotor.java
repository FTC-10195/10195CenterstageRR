package org.firstinspires.ftc.teamcode.CustomHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OwlMotor  {

    DcMotorEx motor;

    double threshold = 0;

    Telemetry telemetry;

    public OwlMotor(HardwareMap hardwareMap, Telemetry telemetry, String config, String displayName, DcMotor.RunMode mode, DcMotorSimple.Direction direction) {
        this.telemetry = telemetry;
        motor= hardwareMap.get(DcMotorEx.class, config);
        motor.setMode(mode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }


    public DcMotorEx returnDevice() {
        return motor;
    }
    public boolean currentOverThreshold() {
        return returnDevice().getCurrent(CurrentUnit.AMPS) > threshold;
    }


    public void telemetry() {
        telemetry.addData("Current current", returnDevice().getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current power", returnDevice().getPower());
        telemetry.addData("Over threshold", currentOverThreshold());


    }
}
