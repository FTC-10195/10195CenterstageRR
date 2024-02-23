package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class StackTest implements VisionProcessor {
    public Rect rect1 = new Rect(100, 190, 80, 90); //ALL COORDINATES PLACEHOLDERS!
    public Rect rect2 = new Rect(250, 170, 200, 40);
    public Rect rect3 = new Rect(500, 190, 90, 80);
    public Rect rect4 = new Rect(0,0,0,0);
    public Rect rect5 = new Rect(0,0,0,0);
    StackTest.Selected selection = StackTest.Selected.ZERO;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double valRect1 = getAvgValue(hsvMat, rect1);
        double valRect2 = getAvgValue(hsvMat, rect2);
        double valRect3 = getAvgValue(hsvMat, rect3);
        double valRect4 = getAvgValue(hsvMat, rect4);
        double valRect5 = getAvgValue(hsvMat, rect5);

        double threshold = 90.;

        boolean R1 = valRect1 > threshold;
        boolean R2 = valRect2 > threshold;
        boolean R3 = valRect3 > threshold;
        boolean R4 = valRect4 > threshold;
        boolean R5 = valRect5 > threshold;

        boolean[] myArray = new boolean[5];

        myArray[0] = R1;
        myArray[1] = R2;
        myArray[2] = R3;
        myArray[3] = R4;
        myArray[4] = R5;

        int numInStack = countStack(myArray);

        return(intToEnum(numInStack));

    }

    public int countStack(boolean[] booleanArray) {
        int numInStack = 0;
        for (int i = 0; i < booleanArray.length; i++) {
            if (booleanArray[i]) {
                numInStack++;
            }
        }
        return(numInStack);
    }

    public Enum intToEnum(int numInStack) {
        if (numInStack == 1) {
            return(Selected.ONE);
        }
        else if (numInStack == 2) {
            return(Selected.TWO);
        }
        else if (numInStack == 3) {
            return(Selected.THREE);
        }
        else if (numInStack == 4) {
            return(Selected.FOUR);
        }
        else if (numInStack == 5) {
            return(Selected.FIVE);
        }
        else {
            return(Selected.ZERO);
        }
    }

    protected double getAvgValue(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[2];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangle1 = makeGraphicsRect(rect1, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangle2 = makeGraphicsRect(rect2, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangle3 = makeGraphicsRect(rect3, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangle4 = makeGraphicsRect(rect4, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangle5 = makeGraphicsRect(rect5, scaleBmpPxToCanvasPx);

        canvas.drawRect(drawRectangle1, nonSelectedPaint);
        canvas.drawRect(drawRectangle2, nonSelectedPaint);
        canvas.drawRect(drawRectangle3, nonSelectedPaint);
        canvas.drawRect(drawRectangle4, nonSelectedPaint);
        canvas.drawRect(drawRectangle5, nonSelectedPaint);
        /*selection = (LocationID.Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }*/
    }

    public StackTest.Selected getSelection() {
        return selection;
    }

    public enum Selected {
        ZERO,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE
    }
}
