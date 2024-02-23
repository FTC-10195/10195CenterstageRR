package org.firstinspires.ftc.teamcode.SubSys;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SimpleBucket implements Subsystem {

    Servo lowerServo;
    Servo upperServo;


    //lower out makes it so that the thing is outside of the hole, lower in makes it inside of the hole

    double lowerOut = 0;
    double lowerIn = .7;

    double upperOut = 0;
    double upperIn = .7;



    public SimpleBucket(HardwareMap hardwareMap) {
        lowerServo = hardwareMap.get(Servo.class, "lserv");
        upperServo = hardwareMap.get(Servo.class, "userv");
        upperServo.setDirection(Servo.Direction.REVERSE);

    }




    public void intakeMode() {
       lowerServo.setPosition(lowerOut);
        upperServo.setPosition(upperOut);
    }




    public void manualMove(double pos) {
        lowerServo.setPosition(pos);
        upperServo.setPosition(pos);

    }

    public void outtakeUpper() {
        upperServo.setPosition(upperIn);
    }

    public void outtakeLower() {
        lowerServo.setPosition(lowerIn);
    }


}
