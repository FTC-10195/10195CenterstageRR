package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;

import java.util.function.BooleanSupplier;

@TeleOp(group = "Match")
@Config
@Photon
public class MIT extends LinearOpMode {
    private MecanumDrive  drive;
    private  Slides slides;

    private  GamepadEx controller1;
    private  GamepadEx controller2;
    private  ElapsedTime time;

    Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setDefaultCommand( new ManualDriveCommand(drive,
                 controller1::getLeftY,
                 controller1::getRightX,
                 controller1::getLeftX,
                () -> controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ));
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            time.reset();

            double loop = 1000/time.milliseconds();

            CommandScheduler.getInstance().run();

            telemetry.addData("Loop (hz)", loop);
            telemetry.update();
        }
    }
}
