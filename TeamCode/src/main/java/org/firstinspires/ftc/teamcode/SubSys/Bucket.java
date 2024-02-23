package org.firstinspires.ftc.teamcode.SubSys;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Bucket implements Subsystem {
    public static int flashLength = 500;

    Servo lowerServo;
    Servo upperServo;
    RevColorSensorV3 lowerSensor;
    RevColorSensorV3 upperSensor;
    ServoEx armServo;
    int switchPosition;
    double bucketDepth;
    Blinkin lights;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    enum COLOR {
        NOTHING,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW
    }

    public Bucket(HardwareMap hardwareMap) {
      //  lights = new Blinkin(hardwareMap);
        lowerSensor = hardwareMap.get(RevColorSensorV3.class, "lsens");
        upperSensor = hardwareMap.get(RevColorSensorV3.class, "usens");

        lowerServo = hardwareMap.get(Servo.class, "lserv");
        upperServo = hardwareMap.get(Servo.class, "userv");
        //  armServo = hardwareMap.get(ServoEx.class, "aserv");
    }

    COLOR top = COLOR.NOTHING;
    COLOR bottom = COLOR.NOTHING;


  /*  public void controlLights() {
        RevBlinkinLedDriver.BlinkinPattern upperPixel = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
        RevBlinkinLedDriver.BlinkinPattern lowerPixel = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;


        if (quality is in range) {
            upperPixel = BlinkinPattern.WHITE;
        }

        else if (quality is in range2) {
            upperPixel = BlinkinPattern.GOLD;
        }

        else if (quality is in range3) {
            upperPixel = BlinkinPattern.VIOLET;
        }

        else if (quality is in range4) {
            upperPixel = BlinkinPattern.LIME;
        }

        else {
            upperPixel = BlinkinPattern.BLACK;
        }

        if (quality1 is in range) {
            lowerPixel = BlinkinPattern.WHITE;
        }

        else if (quality2 is in range2) {
            lowerPixel = BlinkinPattern.GOLD;
        }

        else if (quality3 is in range3) {
            lowerPixel = BlinkinPattern.VIOLET;
        }

        else if (quality4 is in range4) {
            lowerPixel = BlinkinPattern.LIME;
        }

        else {
            lowerPixel = BlinkinPattern.BLACK;
        }

        if ((timer.time() / flashLength) % 4 == 0) {
            lights.changeColor(upperPixel);
        } else if ((timer.time() / flashLength) % 4 == 2) {
            lights.changeColor(lowerPixel);
        } else {
            lights.changeColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }

   */


    public boolean detectSens(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.MM) < bucketDepth;
    }

    public void rotateHook(ServoEx servo) {
        servo.rotateBy(switchPosition);
    }

    public void rotateArm(double angle) {
        armServo.turnToAngle(angle);
    }
}
