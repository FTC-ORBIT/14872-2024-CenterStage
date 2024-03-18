package org.firstinspires.ftc.teamcode.robotSubSystems.camera;


import org.opencv.core.Rect;

public class BluePropThresholdFar extends PropThreshold {

    @Override
    public void initProp() {
        AllianceColor = PropColorEnum.BLUE;
        PropColor = AllianceColor;
        activeLeftRect = LEFT_RECTANGLE_CLOSE;
        activeMiddleRect = MIDDLE_RECTANGLE_CLOSE;
        activeRightRect = RIGHT_RECTANGLE_CLOSE;

        initYellowPixelBoxes();
    }


    @Override
    public void initYellowPixelBoxes() {
        leftRectHitL  = new Rect(390, 0, 70, 180);
        leftRectHitR  = new Rect(470, 0, 55, 180);
        leftRectMissL = new Rect(320, 0, 60, 180);
        leftRectMissR = new Rect(530, 0, 55, 180);

        centerRectHitL  = new Rect(355, 0, 115, 190);
        centerRectHitR  = new Rect(465, 0, 95, 190);
        centerRectMissL = new Rect(230, 0, 125, 190);
        centerRectMissR = new Rect(560, 0, 70, 190);

        rightRectHitL  = new Rect(380, 0, 90, 185);
        rightRectHitR  = new Rect(470, 0, 85 , 185);
        rightRectMissL = new Rect(285, 0, 95, 185);
        rightRectMissR = new Rect(555, 0, 70, 185);

    }

}