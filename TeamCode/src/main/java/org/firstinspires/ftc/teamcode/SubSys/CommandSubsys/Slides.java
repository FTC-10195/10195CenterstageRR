package org.firstinspires.ftc.teamcode.SubSys.CommandSubsys;

import android.transition.Slide;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CustomHardware.OwlMotor;

@Config
public class Slides extends SubsystemBase {

    /*
    * Left might be right or left idr but this works so
     */
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    private double Kp;
    private double Ki;
    private double Kd;
    private double target;
    private double Kf;

    //use one controller bc using two leads to jank behavior on the right (left?) motor
    private PIDFController controller;

    Telemetry telemetry;
    private final double MINIMUM_POS = -10;
    private final double INTAKEPOS = 30;
    private final double LINE1 = 1000;
    private final double LINE2 = 1500;
   // private final double LINE3 = 6000;
    private final double MAXIMUM = 2000;
    public static double currentLeft;
  //  public static double currentRight;

    public enum SlideStates {
        MINIMUM,
        INTAKEPOS,
        LINE1,
        LINE2,
        LINES,
        MAXIMUM,
        MANUAL,
        IDLE
    }

    SlideStates state=  SlideStates.MINIMUM;

    public Slides(HardwareMap hardwareMap, Telemetry telemetry) {
        controller = new PIDFController(Kp, Ki, Kd, Kf);
        this.telemetry = telemetry;
        leftSlide = hardwareMap.get(DcMotorEx.class, "ls");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rs");
    leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

   @Override
    public void periodic() {
        currentLeft = leftSlide.getCurrentPosition();
        double pidPower = controller.calculate(currentLeft, target);
        leftSlide.setPower(pidPower);
        rightSlide.setPower(pidPower);
    }



    /***
     *
     * @param state
     * sets the target of the system, inputting a state
     */
    public void setTarget(SlideStates state) {
        switch (state) {
            case MINIMUM:
                state = SlideStates.MANUAL;
                target = MINIMUM_POS;
                break;
            case INTAKEPOS:
                state = SlideStates.INTAKEPOS;

                target = INTAKEPOS;
                break;
            case LINE1:
                target = LINE1;
                break;
            case LINE2:
                target = LINE2;
                break;
            case MAXIMUM:
                target= MAXIMUM;
                break;


        }
    }


    /*
    public double setManualTarget(boolean up, boolean down) {
        return target - 20;
    }

     */
 /*  public void goDown() {
        leftSlide.setPower(-1);
        rightSlide.setPower(-1);
   }

   public void goUp() {
        leftSlide.setPower(1);
        rightSlide.setPower(1);
   }

   public void stopMoving() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
   }


  */

   public void manualControl(boolean down, boolean up) {
       if (up) {
           target += 20;
       }
       else if (down) {
           target -= 20;
       }
       telemetry.addData("Left", leftSlide.getCurrentPosition());
       telemetry.addData("Right", rightSlide.getCurrentPosition());
   }

/*

    public boolean isJammed() {
        return rightSlide.currentOverThreshold() || leftSlide.currentOverThreshold();
    }
*/
}

