package org.firstinspires.ftc.teamcode.Tests.Utilities;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    RevColorSensorV3 lowerSensor;
    RevColorSensorV3 upperSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        lowerSensor = hardwareMap.get(RevColorSensorV3.class, "lsens");
        upperSensor = hardwareMap.get(RevColorSensorV3.class, "usens");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read color and distance values for lower sensor
            int lowerRed = lowerSensor.red();
            int lowerBlue = lowerSensor.blue();
            int lowerGreen = lowerSensor.green();
            double lowerDist = lowerSensor.getDistance(DistanceUnit.MM);

            // Read color and distance values for upper sensor
            int upperRed = upperSensor.red();
            int upperBlue = upperSensor.blue();
            int upperGreen = upperSensor.green();
            double upperDist = upperSensor.getDistance(DistanceUnit.MM);

            // Display telemetry for lower sensor
            telemetry.addData("Lower Sensor - Red", lowerRed);
            telemetry.addData("Lower Sensor - Blue", lowerBlue);
            telemetry.addData("Lower Sensor - Green", lowerGreen);
            telemetry.addData("Lower Sensor - Distance (mm)", lowerDist);

            // Display telemetry for upper sensor
            telemetry.addData("Upper Sensor - Red", upperRed);
            telemetry.addData("Upper Sensor - Blue", upperBlue);
            telemetry.addData("Upper Sensor - Green", upperGreen);
            telemetry.addData("Upper Sensor - Distance (mm)", upperDist);

            telemetry.update();

            // Your color detection logic goes here
            // Example:
            if (lowerRed == 0 && lowerBlue == 0 && lowerGreen == 0) {
                telemetry.addData("Lower Sensor isPurple", "Yes");
            }

            if (upperRed == 1 && upperBlue == 1 && upperGreen == 1) {
                telemetry.addData("Upper Sensor isGreen", "Yes");
            }

            if (lowerRed == 2 && lowerBlue == 2 && lowerGreen == 2) {
                telemetry.addData("Lower Sensor isYellow", "Yes");
            }
        }
    }
}
