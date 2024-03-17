package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.HashMap;

public class ElementDetectBox {
    public YellowPixelPosEnum place;
    public PropColorEnum elementColor = PropColorEnum.YELLOW;
    public Rect elementBox;
    public double box;
    public double avergedBox;

    public ElementDetectBox(YellowPixelPosEnum place, Rect rect, Mat mat) {
        this.place = place;
        this.elementBox = rect;
        this.box = Core.sumElems(mat.submat(rect)).val[0];

        this.avergedBox = this.box / rect.area() / 255;

    }
}
