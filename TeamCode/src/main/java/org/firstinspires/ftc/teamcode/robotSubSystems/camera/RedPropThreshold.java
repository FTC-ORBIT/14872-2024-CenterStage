package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedPropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.5;
    public double leftBox = 5;
    public double middleBox;

    public static String outStr = "left"; //Set a default value in case vision does not work

    static final org.opencv.core.Rect LEFT_RECTANGLE = new org.opencv.core.Rect(
            new Point(0, 0),
            new Point(200, 479)
    );

    static final org.opencv.core.Rect MIDDLE_RECTANGLE = new Rect(
            new Point(220, 0),
            new Point(420, 479)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

         leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
         middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedMiddleBox = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]




        if(averagedLeftBox > redThreshold){        //Must Tune Red Threshold
            outStr = "left";
        }else if(averagedMiddleBox> redThreshold){
            outStr = "center";
        }else{
            outStr = "right";
        }

         /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return null;            //You do not return the original mat anymore, instead return null





    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


        public static void place(AutonomousPropPlace placement) {
            switch (placement) {
                case LEFTLINE:
                default:
                    outStr = "left";
                    //autonomous code for putting the purple pixel on the left line
                    break;
                case CENTERLINE:
                    outStr = "center";
                    //autonomous code for putting the purple pixel on the middle line
                    break;
                case RIGHTLINE:
                    outStr = "right";
                    //autonomous code for putting the purple pixel on the right line
                    break;
            }
        }
    public String getPropPosition(){  //Returns postion of the prop in a String
        return outStr;
    }
}