package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;

public class Claw {
    private  static  ClawState lastState = ClawState.OPEN;
    private static float pos;
    private static Servo clawServo;

    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
//        clawServo.setDirection(Servo.Direction.REVERSE);
        //clawColorSensor = new OrbitColorSensor(hardwareMap, "clawColorSensor");
    }

    public static void operate(ClawState state) {
        switch (state) {
            case OPEN:
                pos = ClawConstants.open;
                break;
            case CLOSE:
                pos = ClawConstants.closed;
                break;
        }

        clawServo.setPosition(pos);

    }


}
