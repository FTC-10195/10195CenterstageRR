package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

@Autonomous
public class IMURotate extends LinearOpMode {
    IMU imu;

    MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");

        double xRotation = 90;
        double yRotation = 18;
        double zRotation = 90;

        Orientation hubRotation = xyzOrientation(xRotation,yRotation,zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    }

    public void rotate(int degrees, double power) {
        double leftPower, rightPower;
        if (degrees < 0) { //turn right
            leftPower = power; //named for front wheels
            rightPower = -power;
        }
        else if (degrees > 0){ //turn left
            leftPower = -power;
            rightPower = power;
        }
        else {
            return;
        }


    }
}

