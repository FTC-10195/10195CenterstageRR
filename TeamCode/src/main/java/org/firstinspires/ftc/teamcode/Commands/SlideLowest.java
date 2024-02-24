package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SlideLowest extends CommandBase {
    private final Slides slide;
    public SlideLowest(Slides slide) {
        this.slide = slide;
     //   this.low = low;
        addRequirements(slide);
    }
    @Override
    public void execute() {
        slide.bottomOut();
    }
}


