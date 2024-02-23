package org.firstinspires.ftc.teamcode.SubSys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Blinkin extends SubsystemBase {
    public RevBlinkinLedDriver ledDriver;

    public Blinkin(HardwareMap hwmap) {

        ledDriver = hwmap.get(RevBlinkinLedDriver.class, "led");
    }

    //Note, current implementation is rough and is meant for testing manipulating the Blinkin. TODO refactor to switch statement once algo is built
    public void changeColor(boolean down, boolean up, boolean left, boolean right) {
        if (down) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (up) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else if (left) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (right) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }

    }

    public void changeColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        ledDriver.setPattern(pattern);
    }

}
