package org.firstinspires.ftc.teamcode.robotSubSystems.rake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rake {
    public static Servo servo;
    public static Servo servo2;
    public static float pos;
    public static float pos2;

    public static void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "rakeServo");
        servo.setPosition(RakeConstants.closedPos);

        servo2 = hardwareMap.get(Servo.class, "rakeServo2");
        servo2.setPosition(RakeConstants.closedPos);
    }

    public static void operate(RakeState state) {
        switch (state) {
            case CLOSED:
            default:
                pos = RakeConstants.closedPos;
                pos2 = RakeConstants.closedPos2;
                break;
            case OPEN:
                pos = RakeConstants.openPos;
                pos2 = RakeConstants.openPos2;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos2);
    }


}


//dani yalechan!
// yoel yalechan!