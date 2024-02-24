package org.firstinspires.ftc.teamcode.SubSys.CommandSubsys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSys.Blinkin;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake extends SubsystemBase {

    Servo left;
    Servo right;

    Servo lowerServo;
    Servo upperServo;
    RevColorSensorV3 lowerSensor;
    RevColorSensorV3 upperSensor;

    enum LOWERCOLOR {
        NOTHING,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW
    }

    enum UPPERCOLOR {
        NOTHING,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW
    }

    LOWERCOLOR lowercolor = LOWERCOLOR.NOTHING;

    UPPERCOLOR uppercolor = UPPERCOLOR.NOTHING;

    Blinkin lights;

    private double OUTTAKE;
    private double INTAKE;

    //lower out makes it so that the thing is outside of the hole, lower in makes it inside of the hole

    double lowerOut = 0;
    double lowerIn = .7;

    double upperOut = 0;
    double upperIn = .7;

    RevBlinkinLedDriver.BlinkinPattern upperPixel;
    RevBlinkinLedDriver.BlinkinPattern lowerPixel;
    public static int flashLength = 500;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public Outtake(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "la");
        right = hardwareMap.get(Servo.class, "ra");

        left.setDirection(Servo.Direction.REVERSE);

        lowerSensor = hardwareMap.get(RevColorSensorV3.class, "lsens");
        upperSensor = hardwareMap.get(RevColorSensorV3.class, "usens");

        lowerServo = hardwareMap.get(Servo.class, "lserv");
        upperServo = hardwareMap.get(Servo.class, "userv");

        lights = hardwareMap.get(Blinkin.class, "led"
        );

        upperServo.setDirection(Servo.Direction.REVERSE);
    }


    public void outtake() {
        left.setPosition(OUTTAKE);
        right.setPosition(OUTTAKE);
    }

    public void intake() {
        left.setPosition(INTAKE);
        right.setPosition(INTAKE);
    }

    public void upperSensor() {
        double red = upperSensor.red();
        double blue = upperSensor.blue();
        double green = upperSensor.green();
        double  alpha = upperSensor.alpha();
        double  distance = upperSensor.getDistance(DistanceUnit.MM);

        if (distance < 40) {
            uppercolor = UPPERCOLOR.NOTHING;
        }
        else if (alpha > 2000 && blue > 2000 && red > 2000 && green > 2000 && alpha > 2000) {
            uppercolor = UPPERCOLOR.WHITE;
        } else if (blue > 1000 && green < 3000) {
            uppercolor = UPPERCOLOR.PURPLE;
        } else if (blue < 1000 && red < 1000 && green > 1000) {
            uppercolor = UPPERCOLOR.GREEN;
        } else if (blue < 1000 && red > 1000 && green > 1000 && alpha < 2900) {
            uppercolor = UPPERCOLOR.YELLOW;
        }}

    public void lowerSensor() {
        double red = lowerSensor.red();
        double blue = lowerSensor.blue();
        double green = lowerSensor.green();
        double  alpha = lowerSensor.alpha();
        double  distance = lowerSensor.getDistance(DistanceUnit.MM);

        if (distance < 40) {
            lowercolor = LOWERCOLOR.NOTHING;
        }
        else if (alpha > 2000 && blue > 2000 && red > 2000 && green > 2000 && alpha > 2000) {
            lowercolor = LOWERCOLOR.WHITE;
        } else if (blue > 1000 && green < 3000) {
            lowercolor = LOWERCOLOR.PURPLE;
        } else if (blue < 1000 && red < 1000 && green > 1000) {
            lowercolor = LOWERCOLOR.GREEN;
        } else if (blue < 1000 && red > 1000 && green > 1000 && alpha < 2900) {
            lowercolor = LOWERCOLOR.YELLOW;
        }}


    public void setLowerOut() {
        lowerServo.setPosition(lowerOut);
    }

    public void setUpperOut() {
        upperServo.setPosition(upperOut);
    }

    public void setLowerIn() {
        lowerServo.setPosition(lowerIn);
    }

    public void setUpperIn() {
        upperServo.setPosition(upperIn);
    }

    public void moveIn() {
        setLowerIn();
        setUpperIn();
    }

    public void moveOut() {
        setUpperOut();
        setLowerOut();
    }

    @Override
    public void periodic() {
        switch(uppercolor) {
            case NOTHING:
                upperPixel = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                break;
            case WHITE:
                upperPixel = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            case PURPLE:
                upperPixel = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case GREEN:
                upperPixel = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case YELLOW:
                upperPixel = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
        }

        switch(lowercolor) {
            case NOTHING:
                lowerPixel = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                break;
            case WHITE:
                lowerPixel = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            case PURPLE:
                lowerPixel = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case GREEN:
                lowerPixel = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case YELLOW:
                lowerPixel = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
        }

        if ((timer.time() / flashLength) % 4 == 0) {
            lights.changeColor(upperPixel);
        }
        else if ((timer.time() / flashLength) % 4 == 2) {
            lights.changeColor(lowerPixel);
        }
        else {
            lights.changeColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }
}
