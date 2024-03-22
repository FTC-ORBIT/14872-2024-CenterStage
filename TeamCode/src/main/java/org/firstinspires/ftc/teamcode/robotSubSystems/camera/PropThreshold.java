package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import static org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum.HITLEFT;
import static org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum.HITRIGHT;
import static org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum.MISSLEFT;
import static org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum.MISSRIGHT;
import static org.firstinspires.ftc.teamcode.robotSubSystems.camera.YellowPixelPosEnum.NOPIXEL;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.HashMap;
import java.util.HashSet;


public abstract class PropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double Threshold = 0.015;

    Scalar HSVBlueLower = new Scalar(85, 89, 20);
    Scalar HSVBlueUpper = new Scalar(140, 255, 255);

    Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    Scalar highHSVRedUpper = new Scalar(180, 255, 255);

    Scalar HSVYellowLower = new Scalar(10, 49, 0);
    Scalar HSVYellowUpper = new Scalar(40, 255, 255);

    public PropPosEnum PropPos = PropPosEnum.NONE;
    public PropPosEnum sampledPropPos = PropPos;
    public PropColorEnum PropColor = PropColorEnum.RED;
    public PropColorEnum AllianceColor = PropColorEnum.RED;
    public Rect activeLeftRect;
    public Rect activeMiddleRect;
    public Rect activeRightRect;
    public double leftBox;
    public double middleBox;

    public double rightBox;

    public double averagedLeftBox;
    public double averagedMiddleBox;
    public double averagedRightBox;
    public boolean completedPropPos = false;

    public YellowPixelPosEnum yellowPixelPos = YellowPixelPosEnum.NOPIXEL;
    public YellowPixelPosEnum sampledYellowPixelPos = yellowPixelPos;

    public double yellowThreshold = 0.01;
    HashSet<ElementDetectBox> yellowBoxesHash;
    //HashMap<Double, YellowPixelPosEnum> yellowBoxesHash = new HashMap();
    public ElementDetectBox biggest;






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
    public Rect rectHitL  = new Rect(410, 0, 60, 180);
    public Rect rectHitR  = new Rect(480, 0, 60, 180);
    public Rect rectMissL = new Rect(340, 0, 60, 180);
    public Rect rectMissR = new Rect(550, 0, 60, 180);

    Rect leftRectHitL ,
         leftRectHitR ,
         leftRectMissL,
         leftRectMissR,
         centerRectHitL ,
         centerRectHitR ,
         centerRectMissL,
         centerRectMissR,
         rightRectHitL ,
         rightRectHitR ,
         rightRectMissL,
         rightRectMissR;



    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void initProp() {

    }

    public void initYellowPixelBoxes(){}

    public void initYellowPixel() {
        PropColor = PropColorEnum.YELLOW;

        switch (EnumGetPropPos()) {
            case LEFT:
                rectHitL  = leftRectHitL ;
                rectHitR  = leftRectHitR ;
                rectMissL = leftRectMissL;
                rectMissR = leftRectMissR;
                break;
            case CENTER:
            case NONE:
                rectHitL  = centerRectHitL ;
                rectHitR  = centerRectHitR ;
                rectMissL = centerRectMissL;
                rectMissR = centerRectMissR;
                break;
            case RIGHT:
                rectHitL  = rightRectHitL ;
                rectHitR  = rightRectHitR ;
                rectMissL = rightRectMissL;
                rectMissR = rightRectMissR;
                break;
        }
        yellowBoxesHash = new HashSet<ElementDetectBox>() {{
            add(new ElementDetectBox(HITLEFT, rectHitL));
            add(new ElementDetectBox(HITRIGHT, rectHitR));
            add(new ElementDetectBox(MISSLEFT, rectMissL));
            add(new ElementDetectBox(MISSRIGHT, rectMissR));
        }};
//        yellowBoxesHash.put(finalMat.submat(rectHitL)).val[0] / rectHitL.area() / 255, HITLEFT);
//        yellowBoxesHash.put( (finalMat.submat(rectHitR)).val[0]) / rectHitR.area() / 255 , HITRIGHT);
//        yellowBoxesHash.put(finalMat.submat(rectMissL)).val[0] / rectMissL.area() / 255, MISSLEFT);
//        yellowBoxesHash.put(finalMat.submat(rectMissR)).val[0] / rectMissR.area() / 255, MISSRIGHT);
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        if (PropColor == PropColorEnum.BLUE) {
            Core.inRange(testMat, HSVBlueLower, HSVBlueUpper, finalMat);
        } else if (PropColor == PropColorEnum.RED) {
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);
            Core.bitwise_or(lowMat, highMat, finalMat);
        } else if (PropColor == PropColorEnum.YELLOW) {
            Core.inRange(testMat, HSVYellowLower, HSVYellowUpper, finalMat);
        }

//        testMat.release();

        lowMat.release();
        highMat.release();
        if (PropColor == PropColorEnum.YELLOW) {
//            yellowBoxesHash.put(HITLEFT,new ElementDetectBox(HITLEFT, rectHitL, finalMat));
//            yellowBoxesHash.put(HITRIGHT,new ElementDetectBox(HITRIGHT, rectHitR, finalMat));
//            yellowBoxesHash.put(MISSLEFT,new ElementDetectBox(MISSLEFT, rectMissL, finalMat));
//            yellowBoxesHash.put(MISSRIGHT,new ElementDetectBox(MISSRIGHT, rectMissR, finalMat));

            //double biggest = MathFuncs.max(yellowBoxesHash.keySet());
            //if (biggest < yellowThreshold) {
            //  yellowPixelPos = NOPIXEL;
            // }
            //else {
            //    yellowPixelPos = yellowBoxesHash.get(biggest);
            //}
            for (ElementDetectBox eBox: yellowBoxesHash) {
                eBox.boxAverageUpdate(finalMat);
            }
            biggest = ElementDetectBox.max(yellowBoxesHash);
            if (biggest.averagedBox < yellowThreshold) {
                yellowPixelPos = NOPIXEL;
            } else {
                yellowPixelPos = biggest.place;
            }

        } else {
            leftBox = Core.sumElems(finalMat.submat(activeLeftRect)).val[0];
            middleBox = Core.sumElems(finalMat.submat(activeMiddleRect)).val[0];
            rightBox = Core.sumElems(finalMat.submat(activeRightRect)).val[0];


            averagedLeftBox = leftBox / activeLeftRect.area() / 255;
            averagedMiddleBox = middleBox / activeMiddleRect.area() / 255; //Makes value [0,1]
            averagedRightBox = rightBox / activeRightRect.area() / 255;


            if (averagedLeftBox > Threshold && averagedLeftBox > averagedMiddleBox) {        //Must Tune Red Threshold
                PropPos = PropPosEnum.LEFT;
            } else if (averagedMiddleBox > Threshold && averagedRightBox < averagedMiddleBox) {
                PropPos = PropPosEnum.CENTER;
            } else if (averagedRightBox > Threshold) {
                PropPos = PropPosEnum.RIGHT;
            }
            completedPropPos = true;
        }


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

        if (test_mode) {
            if (showFinalMat) {
                finalMat.copyTo(frame, finalMat);
            }

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
        }


//     lowMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
//                                  on the driver station stream, do not use this permanently in your code as
//                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return frame;            //You do not return the original mat anymore, instead return null


    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public PropPosEnum EnumGetPropPos() {
        sampledPropPos = PropPos;
        return sampledPropPos;
    }
    public YellowPixelPosEnum getYellowPixelPos() {
        sampledYellowPixelPos = yellowPixelPos;
        return sampledYellowPixelPos;
    }



//  estimated yellow pixel detection boxes @ 46cm camera-board distance


    public Rect activeRect = rectHitL;
    public int activeRectIndx = 0;
    public String activeRectStr = "rectHitL";
    public int rectStep = 20;
    public Gamepad lastGamepad = new Gamepad();
    boolean test_mode = false;
    boolean showFinalMat = false;

    public void test(Gamepad gamepad, Telemetry telemetry) {

        test_mode = true;

        if (activeRect == null) activeRect = activeLeftRect;

        if (gamepad.start && !lastGamepad.start) {
            showFinalMat = !showFinalMat;
        }

        if (gamepad.share && !lastGamepad.share) {
            activeRectIndx++;
            if (activeRectIndx > 3)
                activeRectIndx = 0;

            switch (activeRectIndx) {
                case 0:
                    activeRect = rectHitL;
                    activeRectStr = "rectHitL";
//                    PropPos = PropPosEnum.LEFT;
                    break;
                case 1:
                    activeRect = rectHitR;
                    activeRectStr = "rectHitR";
//                    PropPos = PropPosEnum.CENTER;
                    break;
                case 2:
                    activeRect = rectMissL;
                    activeRectStr = "rectMissL";
//                    PropPos = PropPosEnum.RIGHT;
                    break;
                case 3:
                    activeRect = rectMissR;
                    activeRectStr = "rectMissR";
//                    PropPos = PropPosEnum.NONE;
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

        telemetry.addLine(String.format("Set Rectangle:  %s   - Indx: %d", activeRectStr, activeRectIndx));
        telemetry.addData("x, y, width, height:  ", activeRect.toString());
        telemetry.addData("TL = ", activeRect.tl());
        telemetry.addData("BR = ", activeRect.br());
        telemetry.addLine("");
        telemetry.addData("rectStep = ", rectStep);
        telemetry.addLine("gamepad = " + gamepad.toString());
        telemetry.addLine("lastGamepad = " + lastGamepad.toString());

    }

}

