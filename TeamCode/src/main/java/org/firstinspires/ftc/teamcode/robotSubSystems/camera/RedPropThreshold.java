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
//    Mat qrtrMat = new Mat();
    double redThreshold = 0.015;
    public double redLeftBox;
    public double redMiddleBox;
    public double averagedRedLeftBox;
    public double averagedRedMiddleBox;

    public static String redOutStr = "none"; //Set a default value in case vision does not work
    public PropPosEnum redPropPos = PropPosEnum.NONE;
    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 230),
            new Point(240, 479)
//            new Point(200, 359)
    );

    static final Rect MIDDLE_RECTANGLE = new Rect(
            new Point(241, 230),
            new Point(440 , 479)
//            new Point(420, 359)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
//        Imgproc.resize(frame, qrtrMat, Size() , 0.5, 0.5, );
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

         redLeftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
         redMiddleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

        averagedRedLeftBox = redLeftBox / LEFT_RECTANGLE.area() / 255;
        averagedRedMiddleBox = redMiddleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]




        if(averagedRedLeftBox > redThreshold && averagedRedLeftBox > averagedRedMiddleBox){        //Must Tune Red Threshold
            redOutStr = "redLeft";
            redPropPos = PropPosEnum.LEFT;
        }else if(averagedRedMiddleBox > redThreshold && averagedRedMiddleBox > averagedRedLeftBox){
            redOutStr = "redMiddle";
            redPropPos = PropPosEnum.CENTER;
        }else{
            redOutStr = "redRight";
            redPropPos = PropPosEnum.RIGHT;
        }

        Imgproc.rectangle(
                frame,
                new Point(232, 0),
                new Point(510, 479),
                new Scalar(255, 0, 0), 10);

//        new Point(220, 0),
//                new Point(420, 479)

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return frame;
    }
         /*This line should only be added in when you want to see your custom pipeline
                                          on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/     //You do not return the original mat anymore, instead return null








    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }



    public String redGetPropPosition(){  //Returns postion of the prop in a String
        return redOutStr;
    }
    public PropPosEnum redEnumGetPropPos(){
        return redPropPos;
    }
}
