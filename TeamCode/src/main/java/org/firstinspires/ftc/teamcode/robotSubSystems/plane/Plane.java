package org.firstinspires.ftc.teamcode.robotSubSystems.plane;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    public static Servo planeServo;
    private static int pos = 0;

    public static void init(HardwareMap hardwareMap) {
        planeServo = hardwareMap.get(Servo.class, "planeServo");

    }

    public static void operate(PlaneState state) {
        switch (state) {
            case STOP:
            default:
                pos = PlaneConstants.stopPos;
                break;
            case THROW:
                pos = PlaneConstants.throwPos;
                break;
        }
        planeServo.setPosition(pos);
    }
}
