package org.firstinspires.ftc.teamcode.robotSubSystems.plane;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    public static Servo planeServo;
    private static float pos = 0;

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
    public static void test(Gamepad gamepad1 , Gamepad gamepad2 ){
        if (gamepad1.left_bumper) planeServo.setPosition(0.05);
        if (gamepad1.right_bumper) planeServo.setPosition(0);

    }
}
