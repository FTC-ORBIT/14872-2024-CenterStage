package org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold;


import org.firstinspires.ftc.teamcode.robotSubSystems.camera.threshold.enums.PropColorEnum;

public class BluePropThresholdClose extends PropThreshold {

    @Override
    public void initProp() {
        PropColor = PropColorEnum.BLUE;
        activeLeftRect = LEFT_RECTANGLE_FAR;
        activeMiddleRect = MIDDLE_RECTANGLE_FAR;
        activeRightRect = RIGHT_RECTANGLE_FAR;
    }
}