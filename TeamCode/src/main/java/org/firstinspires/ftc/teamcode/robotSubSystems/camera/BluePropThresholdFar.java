package org.firstinspires.ftc.teamcode.robotSubSystems.camera;


public class BluePropThresholdFar extends PropThreshold {

    @Override
    public void initProp() {
        PropColor = PropColorEnum.BLUE;
        activeLeftRect = LEFT_RECTANGLE_CLOSE;
        activeMiddleRect = MIDDLE_RECTANGLE_CLOSE;
        activeRightRect = RIGHT_RECTANGLE_CLOSE;
    }
}