package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class PropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double Threshold = 0.015;

    public static String OutStr = "none"; //Set a default value in case vision does not work
    public PropPosEnum PropPos = PropPosEnum.NONE;

    public PropColorEnum PropColor = PropColorEnum.RED;
    public Rect activeLeftRect;
    public Rect activeMiddleRect;
    public Rect activeRightRect;
    public double leftBox;
    public double middleBox;

    public double rightBox;

//    public double blueLeftBoxFar;
//    public double blueMiddleBoxFar;
//    public double blueRightBoxFar;
    public double averagedLeftBox;
    public double averagedMiddleBox;

    public double averagedRightBox;

//    public double averagedBlueLeftBoxFar;
//    public double averagedBlueMiddleBoxFar;
//
//    public double averagedBlueRightBoxFar;
    static final Rect LEFT_RECTANGLE_CLOSE = new Rect(
            new Point(0, 225),
            new Point(240, 479)
    );

    static final Rect MIDDLE_RECTANGLE_CLOSE = new Rect(
            new Point(241, 225),
            new Point(440 , 479)
    );
    static final Rect RIGHT_RECTANGLE_CLOSE = new Rect(
            new Point(441, 225),
            new Point(639, 479)
    );
    static final Rect LEFT_RECTANGLE_FAR = new Rect(
            new Point(0, 225),
            new Point(320, 479)
    );
    static final Rect MIDDLE_RECTANGLE_FAR = new Rect(
            new Point(340, 225),
            new Point(525, 479)
    );
    static final Rect RIGHT_RECTANGLE_FAR = new Rect(
            new Point(526, 225),
            new Point(639, 479)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void initProp() {

    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVBlueLower = new Scalar(85, 89, 20);  //Beginning of Color Wheel
        Scalar lowHSVBlueUpper = new Scalar(140, 255, 255);

//        Scalar blueHSVBlueLower = new Scalar(85, 89, 20); //Wraps around Color Wheel
//        Scalar highHSVBlueUpper = new Scalar(140, 255, 255);

        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        if (PropColor == PropColorEnum.BLUE) {
            Core.inRange(testMat, lowHSVBlueLower, lowHSVBlueUpper, finalMat);
        } else {
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);
            Core.bitwise_or(lowMat, highMat, finalMat);
        }
//        Core.inRange(testMat, lowHSVBlueLower, lowHSVBlueUpper,lowMat);

//        testMat.release();

        lowMat.release();
        highMat.release();

        leftBox = Core.sumElems(finalMat.submat(activeLeftRect)).val[0];
        middleBox = Core.sumElems(finalMat.submat(activeMiddleRect)).val[0];
        rightBox = Core.sumElems(finalMat.submat(activeRightRect)).val[0];

//        blueLeftBoxFar = Core.sumElems(finalMat.submat(LEFT_RECTANGLE_FAR)).val[0];
//        blueMiddleBoxFar = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE_FAR)).val[0];
//        blueRightBoxFar = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE_FAR)).val[0];


        averagedLeftBox = leftBox / activeLeftRect.area() / 255;
        averagedMiddleBox = middleBox / activeMiddleRect.area() / 255; //Makes value [0,1]
        averagedRightBox = rightBox / activeRightRect.area() / 255;


//        averagedBlueLeftBoxFar = blueLeftBoxFar / LEFT_RECTANGLE_FAR.area() / 255;
//        averagedBlueMiddleBoxFar = blueMiddleBoxFar / MIDDLE_RECTANGLE_FAR.area() / 255; //Makes value [0,1]
//        averagedBlueRightBoxFar = blueRightBoxFar / RIGHT_RECTANGLE_FAR.area() / 255;


        if (averagedLeftBox > Threshold && averagedLeftBox > averagedMiddleBox) {        //Must Tune Red Threshold
            OutStr = "Left";
            PropPos = PropPosEnum.LEFT;
        } else if (averagedMiddleBox > Threshold && averagedRightBox < averagedMiddleBox) {
            OutStr = "Center";
            PropPos = PropPosEnum.CENTER;
        } else if (averagedRightBox > Threshold) {
            OutStr = "Right";
            PropPos = PropPosEnum.RIGHT;
        }
//        if(averagedBlueLeftBoxFar > Threshold){        //Must Tune Red Threshold
//            OutStr = "blueLeftFar";
//            PropPos = PropPosEnum.LEFT;
//        }else if(averagedBlueMiddleBoxFar > Threshold){
//            OutStr = "blueCenter";
//            PropPos = PropPosEnum.CENTER;
//        }else if(averagedBlueRightBoxFar > Threshold){
//            OutStr = "blueRight";
//            PropPos = PropPosEnum.RIGHT;
//        }


//        Imgproc.rectangle(
//                frame,
//                new Point(340, 230),
//                new Point(520, 479),
//                new Scalar(255, 0, 0), 10);

        Imgproc.rectangle(
                frame,
                activeMiddleRect.tl(),
                activeMiddleRect.br(),
                new Scalar(0, 255, 0), 10);


        Imgproc.rectangle(
                frame,
                rectHitL.tl(),
                rectHitL.br(),
                new Scalar(255, 0, 0), 10);

        Imgproc.rectangle(
                frame,
                rectHitR.tl(),
                rectHitR.br(),
                new Scalar(255, 0, 80), 10);

        Imgproc.rectangle(
                frame,
                rectMissL.tl(),
                rectMissL.br(),
                new Scalar(0, 200, 0), 10);

        Imgproc.rectangle(
                frame,
                rectMissR.tl(),
                rectMissR.br(),
                new Scalar(0, 255, 0), 10);

        Imgproc.rectangle(
                frame,
                activeRect.tl(),
                activeRect.br(),
                new Scalar(0, 255, 255), 10);



//     lowMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
//                                  on the driver station stream, do not use this permanently in your code as
//                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return frame;            //You do not return the original mat anymore, instead return null


    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String GetPropPosition() {
        return OutStr;
    }

    public PropPosEnum EnumGetPropPos() {
        return PropPos;
    }


//    public boolean lastLeft = false;
//    public boolean lastRight = false;
//    public boolean lastRT = false;
//    public boolean lastLT = false;

//  estimated yellow pixel detection boxes @ 46cm camera-board distance
    public Rect rectHitL = new Rect(260, 240, 60, 240);
    public Rect rectHitR = new Rect(340, 240, 60, 240);
    public Rect rectMissL = new Rect(160, 240, 60, 240);
    public Rect rectMissR = new Rect(440, 240, 60, 240);

    public Rect activeRect = rectHitL;
    public int activeRectIndx = 0;
    public String activeRectStr = "rectHitL";
    public int rectStep = 20;
    public Gamepad lastGamepad = new Gamepad();

    public void test(Gamepad gamepad, Telemetry telemetry) {

        if (activeRect == null) activeRect = activeLeftRect;
        if (gamepad.share && !lastGamepad.share) {
            activeRectIndx++;
            if (activeRectIndx > 3)
                activeRectIndx = 0;

            switch (activeRectIndx) {
                case 0:
                    activeRect = rectHitL;
                    activeRectStr = "rectHitL";
                    break;
                case 1:
                    activeRect = rectHitR;
                    activeRectStr = "rectHitR";
                    break;
                case 2:
                    activeRect = rectMissL;
                    activeRectStr = "rectMissL";
                    break;
                case 3:
                    activeRect = rectMissR;
                    activeRectStr = "rectMissR";
                    break;
            }
        }


        if (gamepad.dpad_left && !lastGamepad.dpad_left) {
            activeRect.x -= rectStep;
            if (activeRect.x < 0)
                activeRect.x = 0;
        } else if (gamepad.dpad_right && !lastGamepad.dpad_right) {
            activeRect.x += rectStep;
            if (activeRect.x > testMat.cols() - 1)
                activeRect.x = testMat.cols() - 1;
        }
        if (gamepad.dpad_up && !lastGamepad.dpad_up) {
            activeRect.y -= rectStep;
            if (activeRect.y < 0)
                activeRect.y = 0;
        } else if (gamepad.dpad_down && !lastGamepad.dpad_down) {
            activeRect.y += rectStep;
            if (activeRect.y > testMat.rows() - 1)
                activeRect.y = testMat.rows() - 1;
        }

        if (gamepad.x && !lastGamepad.x ) {
            activeRect.width -= rectStep;
            if (activeRect.width < 1)
                activeRect.width = 1;
        } else if (gamepad.b && !lastGamepad.b) {
            activeRect.width += rectStep;
            if (activeRect.br().x > testMat.cols() - 1)
                activeRect.width = testMat.cols() - activeRect.x;
        }
        if (gamepad.y && !lastGamepad.y) {
            activeRect.height -= rectStep;
            if (activeRect.height < 0)
                activeRect.height = 0;
        } else if (gamepad.a && !lastGamepad.a) {
            activeRect.height += rectStep;
            if (activeRect.br().y > testMat.rows() - 1)
                activeRect.height = testMat.rows() - activeRect.y;
        }


        if (gamepad.left_bumper && !lastGamepad.left_bumper) {
            rectStep -= 1;
        } else if (gamepad.right_bumper && !lastGamepad.right_bumper) {
            rectStep += 1;
        }

        lastGamepad.copy(gamepad);

        telemetry.addLine(String.format("Set Rectangle:  %s   - Indx: %d",
                activeRectStr, activeRectIndx));
        telemetry.addData("x, y, width, height:  ", activeRect.toString());
        telemetry.addData("TL = ", activeRect.tl());
        telemetry.addData("BR = ", activeRect.br());
        telemetry.addLine("");
        telemetry.addData("rectStep = ", rectStep);
        telemetry.addLine("gamepad = " + gamepad.toString());
        telemetry.addLine("lastGamepad = " + lastGamepad.toString());

//        lastLeft = gamepad.left_bumper;
//        lastRight = gamepad.right_bumper;
//        lastLT = gamepad.dpad_left;
//        lastRT = gamepad.dpad_right;
    }

}

