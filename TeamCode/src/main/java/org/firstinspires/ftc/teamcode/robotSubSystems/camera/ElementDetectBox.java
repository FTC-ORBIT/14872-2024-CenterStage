package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

//import java.util.HashMap;
import java.util.HashSet;

public class ElementDetectBox {
    public YellowPixelPosEnum place;
    public PropColorEnum elementColor = PropColorEnum.YELLOW;
    public Rect elementBox;
    public double box;
    public double averagedBox = 0;

    public ElementDetectBox(YellowPixelPosEnum place, Rect rect) {
        this.place = place;
        this.elementBox = rect;
    }
    public void boxAverageUpdate(Mat mat) {
        this.box = Core.sumElems(mat.submat(this.elementBox)).val[0];

        this.averagedBox = this.box / this.elementBox.area() / 255;
    }

    public static ElementDetectBox max(HashSet<ElementDetectBox> boxList) {
        ElementDetectBox highestScoreBox = null;

        for (ElementDetectBox eBox: boxList) {
            if (highestScoreBox == null) {
                highestScoreBox = eBox;
            }
            else {
                if (eBox.averagedBox > highestScoreBox.averagedBox ||
                        (eBox.averagedBox == highestScoreBox.averagedBox) && (eBox.box > highestScoreBox.box)) {
                    highestScoreBox = eBox;
                }
            }
        }

        return highestScoreBox;
    }

}
