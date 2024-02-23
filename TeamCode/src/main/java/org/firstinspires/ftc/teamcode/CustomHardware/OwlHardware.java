package org.firstinspires.ftc.teamcode.CustomHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Superclass for all OwlHardware
 */
public class OwlHardware {

    private Object device;

    private  Class  hardwareType;
    Telemetry telemetry;

    String displayName;

    String config;

    public OwlHardware(HardwareMap hardwareMap, Telemetry telemetry, String config, String displayName, Class hardwareType) {
       // this.hardwareType = hardwareType;
      //  this.telemetry = telemetry;
      //  this.config = config;
     //   this.displayName = displayName;

    }

    public Object returnDevice(){
        return null;
    }

    public void telemetry() {


    }

}
