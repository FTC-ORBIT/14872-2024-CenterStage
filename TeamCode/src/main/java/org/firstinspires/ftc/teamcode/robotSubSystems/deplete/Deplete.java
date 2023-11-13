package org.firstinspires.ftc.teamcode.robotSubSystems.deplete;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deplete {
    public static Servo rightServo;
    public static Servo leftServo;
    public static float leftPos;
    public static float rightPos;

    public static void init(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public static void operate(DepleteState state) {
        switch (state) {
            case CLOSED:
            default:
                leftPos = DepleteConstants.leftClosedPos;
                rightPos = DepleteConstants.rightClosedPos;
                break;
            case OPEN:
                leftPos = DepleteConstants.leftOpenPos;
                rightPos = DepleteConstants.rightOpenPos;
                break;
            case LEFT_OPEN:
                leftPos = DepleteConstants.leftOpenPos;
                rightPos = DepleteConstants.rightClosedPos;
                break;
            case RIGHT_OPEN:
                leftPos = DepleteConstants.leftClosedPos;
                rightPos = DepleteConstants.rightOpenPos;
                break;
        }
        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);
    }
}
