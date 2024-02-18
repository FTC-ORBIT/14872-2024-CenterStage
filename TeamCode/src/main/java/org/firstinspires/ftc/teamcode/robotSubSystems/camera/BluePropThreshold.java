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

public class BluePropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
        Mat highMat = new Mat();
    Mat lowMat = new Mat();
        Mat finalMat = new Mat();
    double blueThreshold = 0.015;
    String blueOutStr = "none"; //Set a default value in case vision does not work
    public double blueLeftBox;
    public double blueMiddleBox;
    public double averagedBlueLeftBox;
    public double averagedRedMiddleBox;
    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(232, 479)
    );

    static final Rect MIDDLE_RECTANGLE = new Rect(
            new Point(232, 0),
            new Point(510 , 479)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVBlueLower = new Scalar(85, 89, 20);  //Beginning of Color Wheel
        Scalar lowHSVBlueUpper = new Scalar(140, 255, 255);

        Scalar blueHSVBlueLower = new Scalar(85, 89, 20); //Wraps around Color Wheel
        Scalar highHSVBlueUpper = new Scalar(140, 255, 255);

//        Scalar lowHSVBlueLower = new Scalar(85, 89 , 20);  //Beginning of Color Wheel
//        Scalar lowHSVBlueUpper = new Scalar(140, 255, 255);

        //   Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        //    Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        //  Scalar lowHSVBlueLower = new Scalar(0, 80, 0);  //Beginning of Color Wheel
        //  Scalar lowHSVRedUpper = new Scalar(30, 255, 255);

        //  Scalar redHSVRedLower = new Scalar(180, 120, 40); //Wraps around Color Wheel
        //  Scalar highHSVRedUpper = new Scalar(200, 255, 255);

        Core.inRange(testMat, lowHSVBlueLower,lowHSVBlueUpper , lowMat);
            Core.inRange(testMat, blueHSVBlueLower, highHSVBlueUpper, highMat);
//        Core.inRange(testMat, lowHSVBlueLower, lowHSVBlueUpper,lowMat);

        testMat.release();

        Core.bitwise_or(lowMat,highMat, finalMat);

         lowMat.release();
         highMat.release();

        blueLeftBox = Core.sumElems(lowMat.submat(LEFT_RECTANGLE)).val[0];
         blueMiddleBox = Core.sumElems(lowMat.submat(MIDDLE_RECTANGLE)).val[0];


        averagedBlueLeftBox = blueLeftBox / LEFT_RECTANGLE.area() / 255;
        averagedRedMiddleBox = blueMiddleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]




        if(averagedBlueLeftBox > blueThreshold){        //Must Tune Red Threshold
            blueOutStr = "blueLeft";
        }else if(averagedRedMiddleBox > blueThreshold){
            blueOutStr = "blueCenter";
        }else{
            blueOutStr = "blueRight";
        }

        Imgproc.rectangle(
                frame,
                new Point(232, 0),
                new Point(510, 479),
                new Scalar(255, 0, 0), 10);
   //     lowMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
//                                  on the driver station stream, do not use this permanently in your code as
//                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return frame;            //You do not return the original mat anymore, instead return null




    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){
        return blueOutStr;
    }
}