package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSys.CommandSubsys.Slides;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualSlides extends CommandBase {

    private final Slides slide;


   // private  rightTrigger;
   // private double leftTrigger;

    private BooleanSupplier up;
    private  BooleanSupplier down;

    public ManualSlides( Slides slide, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger) {
        this.slide = slide;
        this.up = rightTrigger;
        this.down = leftTrigger;
        addRequirements(slide);
    }

    @Override
    public void execute() {
        slide.manualControl(
                down.getAsBoolean(),
                up.getAsBoolean()
        );
    }





}
