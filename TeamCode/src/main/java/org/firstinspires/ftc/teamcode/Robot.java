package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSys.Bucket;
import org.firstinspires.ftc.teamcode.SubSys.DropDown;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;


public class Robot {

    private final Bucket bucket;
    private final Chamber chamber;
    private final DropDown dropDown;
    private MecanumDrive mecanumDrive;
    private final Slides slides;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        bucket = new Bucket(hardwareMap);
        chamber = new Chamber(hardwareMap);
        dropDown = new DropDown(hardwareMap);
        slides = new Slides(hardwareMap, telemetry);

    }

    public enum Color {
        BLUE,
        RED
    }

    Color color = Color.BLUE;

    public void setColor(int col) {
        switch (col) {
            case 0:
                color = Color.BLUE;
                break;
            case 1:
                color = Color.RED;
                break;
        }
    }

    public String getColor() {
        return color.toString();
    }


}
