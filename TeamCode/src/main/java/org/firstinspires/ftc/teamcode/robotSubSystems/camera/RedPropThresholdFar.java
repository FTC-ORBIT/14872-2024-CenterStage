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


public class RedPropThresholdFar extends PropThreshold {

    @Override
    public void initProp() {
        PropColor = PropColorEnum.RED;
        activeLeftRect = LEFT_RECTANGLE_CLOSE;
        activeMiddleRect = MIDDLE_RECTANGLE_CLOSE;
        activeRightRect = RIGHT_RECTANGLE_CLOSE;
    }
}