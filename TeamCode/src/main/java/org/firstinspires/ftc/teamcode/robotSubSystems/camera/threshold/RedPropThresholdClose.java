package org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold;


import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.enums.PropColorEnum;

public class RedPropThresholdClose extends PropThreshold {

    @Override
    public void initProp() {
        PropColor = PropColorEnum.RED;
        activeLeftRect = LEFT_RECTANGLE_CLOSE;
        activeMiddleRect = MIDDLE_RECTANGLE_CLOSE;
        activeRightRect = RIGHT_RECTANGLE_CLOSE;
    }
}