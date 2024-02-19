package org.firstinspires.ftc.teamcode.robotSubSystems.camera;


public class RedPropThresholdClose extends PropThreshold {

    @Override
    public void initProp() {
        PropColor = PropColorEnum.RED;
        activeLeftRect = LEFT_RECTANGLE_FAR;
        activeMiddleRect = MIDDLE_RECTANGLE_FAR;
        activeRightRect = RIGHT_RECTANGLE_FAR;
    }
}