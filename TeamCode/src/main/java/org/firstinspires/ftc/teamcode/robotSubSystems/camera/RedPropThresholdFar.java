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
        leftRectHitL  = new Rect(410, 0, 60, 180);
        leftRectHitR  = new Rect(480, 0, 60, 180);
        leftRectMissL = new Rect(340, 0, 60, 180);
        leftRectMissR = new Rect(550, 0, 60, 180);

        centerRectHitL  = new Rect(430, 0, 60, 180);
        centerRectHitR  = new Rect(490, 0, 50, 180);
        centerRectMissL = new Rect(365, 0, 60, 180);
        centerRectMissR = new Rect(550, 0, 60, 180);

        rightRectHitL  = new Rect(350, 0, 70, 180);
        rightRectHitR  = new Rect(425, 0, 70, 180);
        rightRectMissL = new Rect(270, 0, 75, 180);
        rightRectMissR = new Rect(500, 0, 60, 180);
    }
}