package org.firstinspires.ftc.teamcode.SubSys;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DropDown implements Subsystem {
    DcMotorEx brrrr;
    Servo drop;
    public DropDown(HardwareMap hwmap) {
        drop = hwmap.get(Servo.class, "drop");
        brrrr = hwmap.get(DcMotorEx.class, "spin");
        brrrr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brrrr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void intake() {
        drop.setPosition(.5);
    }
    public void idle() {
        drop.setPosition(.75);
    }

    public void spin(boolean on, boolean off,boolean reverse) {
        if(on) {
            brrrr.setPower(1);
        }
            else if(reverse) {
                brrrr.setPower(-1);
        }

            else if(off) {
                    brrrr.setPower(0);
        }
    }

//    public boolean isJammed() {
     //   return drop.getCurrent(CurrentUnit.AMPS) > 10;
   // }


}




