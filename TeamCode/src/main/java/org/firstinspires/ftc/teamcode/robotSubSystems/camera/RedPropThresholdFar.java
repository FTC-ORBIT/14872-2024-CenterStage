package org.firstinspires.ftc.teamcode.robotSubSystems.camera;


import org.opencv.core.Rect;

public class RedPropThresholdFar extends PropThreshold {



    @Override
    public void initProp() {
        AllianceColor = PropColorEnum.RED;
        PropColor = AllianceColor;
        activeLeftRect = LEFT_RECTANGLE_FAR;
        activeMiddleRect = MIDDLE_RECTANGLE_FAR;
        activeRightRect = RIGHT_RECTANGLE_FAR;

        initYellowPixelBoxes();
    }

    @Override
    public void initYellowPixelBoxes() {
        leftRectHitL  = new Rect(220, 0, 110, 180);
        leftRectHitR  = new Rect(325, 0, 90, 180);
        leftRectMissL = new Rect(200, 80, 1, 0);
        leftRectMissR = new Rect(410, 0, 70, 180);

//        centerRectHitL  = new Rect(295, 0, 100, 180);
//        centerRectHitR  = new Rect(405, 0, 65, 180);
//        centerRectMissL = new Rect(120, 0, 165, 180);
//        centerRectMissR = new Rect(480, 0, 120, 180);
        centerRectHitL  = new Rect(360, 0, 70, 180);
        centerRectHitR  = new Rect(415, 0, 80, 180);
        centerRectMissL = new Rect(260, 0, 90, 180);
        centerRectMissR = new Rect(519, 0, 80, 180);

        rightRectHitL  = new Rect(380, 0, 95, 185);
        rightRectHitR  = new Rect(445, 0, 100, 185);
        rightRectMissL = new Rect(270, 0, 100, 185);
        rightRectMissR = new Rect(555, 190, 1, 0);
    }
}