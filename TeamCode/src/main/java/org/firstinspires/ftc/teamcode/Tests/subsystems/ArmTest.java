package org.firstinspires.ftc.teamcode.Tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSys.Arm;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);
        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            arm.rotate(pos);


        }
    }
}
