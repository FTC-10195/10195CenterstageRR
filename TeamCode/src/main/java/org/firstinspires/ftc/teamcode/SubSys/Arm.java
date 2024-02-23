package org.firstinspires.ftc.teamcode.SubSys;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm implements Subsystem {

    Servo left;
    Servo right;


    public static double out = 0;

    public Arm(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "la");
        right = hardwareMap.get(Servo.class, "ra");
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void rotate(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    public void intake() {
     //   left.setPosition();
    }

}
