package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class ManualDriveCommand extends CommandBase {

    private final MecanumDrive drive;
    private DoubleSupplier forward;
    private  DoubleSupplier rotate;
    private  DoubleSupplier strafe;


    private BooleanSupplier slow;


    public ManualDriveCommand(MecanumDrive drive, DoubleSupplier forward, DoubleSupplier rotate, DoubleSupplier strafe, BooleanSupplier slow) {
        this.drive = drive;
        this.forward = forward;
        this.rotate = rotate;
        this.strafe = strafe;
        this.slow = slow;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.robotDrive(
              -  forward.getAsDouble(),
              -  strafe.getAsDouble(),
             -   rotate.getAsDouble(),
                slow.getAsBoolean()
        );
    }
}
