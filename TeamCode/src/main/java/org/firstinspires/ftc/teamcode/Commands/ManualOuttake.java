package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Outtake;

import java.util.function.BooleanSupplier;

public class ManualOuttake extends CommandBase {

    private Outtake outtake;

    public ManualOuttake(Outtake outtake, BooleanSupplier intake, BooleanSupplier scoreOut ) {

    }
}
