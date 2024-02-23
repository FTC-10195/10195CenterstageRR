package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PaperAirplane {

    Servo plane;

    double shootPos = 0;

    public PaperAirplane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(Servo.class, "pA");
    }

    public void setPostion(double position) {
        plane.setPosition(position);
    }


}
