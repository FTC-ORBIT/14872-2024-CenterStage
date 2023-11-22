package org.firstinspires.ftc.teamcode.robotSubSystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public static Servo servo;
    public static float pos;


    public static void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "outtakeServo");

    }

    public static void operate(OuttakeState state) {
        switch (state) {
            case CLOSED:
            default:
                pos = OuttakeConstants.closedPos;

                break;
            case OPEN:
                pos = OuttakeConstants.openPos;

                break;
        }
        servo.setPosition(pos);
    }
}
