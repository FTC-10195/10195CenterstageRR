package org.firstinspires.ftc.teamcode.CustomHardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Extension of the RevColorSensorV3 class with detection logic and telemetry niceness
 */
public class OwlColorSensor extends OwlHardware  {
     RevColorSensorV3 sensor;


    /**
     *
     * @param hardwareMap
     * @param telemetry
     * @param config
     * @param displayName -- name to be printed on telemetry, is not the same as config
     */
    public OwlColorSensor(HardwareMap hardwareMap, Telemetry telemetry, String config, String displayName) {
        super(hardwareMap, telemetry, config, displayName, RevColorSensorV3.class);
    }

    enum SensedColor {
        NOTHING,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW
    }

    int red;
    int blue;
    int green;
    int alpha;

    double distance;

    SensedColor color = SensedColor.NOTHING;


    /**
     * Sets the inner enum to currently sensed color
     */
    public void senseColor() {
        red = sensor.red();
        blue = sensor.blue();
        green = sensor.green();
        alpha = sensor.alpha();
        distance = sensor.getDistance(DistanceUnit.MM);

        if (distance < 40) {
            color = SensedColor.NOTHING;
        }
        else if (alpha > 2000 && blue > 2000 && red > 2000 && green > 2000 && alpha > 2000) {
            color = SensedColor.WHITE;
        } else if (blue > 1000 && green < 3000) {
            color = SensedColor.PURPLE;
        } else if (blue < 1000 && red < 1000 && green > 1000) {
            color = SensedColor.GREEN;
        } else if (blue < 1000 && red > 1000 && green > 1000 && alpha < 2900) {
            color = SensedColor.YELLOW;
        }

    }
    /**
     * Get current color detected
     */
    public SensedColor getColor() {
        return color;
    }


    @Override
    public Object returnDevice() {
        return sensor;
    }

    @Override
    public void telemetry() {
        // Display telemetry for upper sensor
        telemetry.addData(displayName + "- Red", red);
        telemetry.addData(displayName + " Blue", blue);
        telemetry.addData(displayName + "Green", green);
        telemetry.addData(displayName + "Alpha", alpha);
        telemetry.addData(displayName + "Distance (mm)", distance);
        telemetry.addData("Current color", color);

    }

}