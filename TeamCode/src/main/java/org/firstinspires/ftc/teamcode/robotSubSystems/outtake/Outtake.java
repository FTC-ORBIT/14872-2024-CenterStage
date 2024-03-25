package org.firstinspires.ftc.teamcode.robotSubSystems.outtake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    public static Servo servo;
     public static Servo servo2;
    public static float pos;
    public static float pos2;
    public static boolean lastLeft = false;
    public static boolean lastRight = false;
    public static boolean lastRT = false;
    public static boolean lastlT = false;

    public static void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "outtakeServo");
        servo.setPosition(OuttakeConstants.closedPos);

        servo2 = hardwareMap.get(Servo.class, "outtakeServo2");
        servo2.setPosition(OuttakeConstants.closedPos);
    }

    public static void operate(OuttakeState state) {
        switch (state) {
            case CLOSED:
            default:
                pos = OuttakeConstants.closedPos;
                pos2 = OuttakeConstants.closedPos2;
                break;
            case OPEN:
                pos = OuttakeConstants.openPos;
                pos2 = OuttakeConstants.openPos2;
                break;
            case OUT:
                pos = OuttakeConstants.outPos;
                pos2 = OuttakeConstants.closedPos2;
                break;
            case TOWOUT:
                pos = OuttakeConstants.outPos;
                pos2 = OuttakeConstants.openPos2;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos2);
    }

    public static void test(Gamepad gamepad, Telemetry telemetry) {

//        if (gamepad.left_bumper) {
//            pos = OuttakeConstants.closedPos;
//            pos2 = OuttakeConstants.closedPos2;
//        } else if (gamepad.right_bumper) {
//            pos = OuttakeConstants.openPos;
//            pos2 = OuttakeConstants.openPos2;
//        }
        if (gamepad.dpad_left &&  !lastLeft){
            pos2 += 0.05;
            if (pos2 > 1){
                pos2 = 1;
            }
        }else if (gamepad.dpad_right && !lastRight){
            pos2 -= 0.05;
            if (pos2 < 0){
                pos2 = 0;
            }
        }
        if (gamepad.left_bumper && !lastlT){
            pos2 += 0.001;
            if (pos2 > 1){
                pos2 = 1;
            }
        }else if (gamepad.right_bumper && !lastRT){
            pos2 -= 0.001;
            if (pos2 < 0){
                pos2 = 0;
            }
        }
        servo2.setPosition(pos2);
        lastLeft = gamepad.left_bumper;
        lastRight = gamepad.right_bumper;
        lastlT = gamepad.dpad_left;
        lastRT = gamepad.dpad_right;
        telemetry.addData("Outtake pos" , pos2);
        telemetry.update();
        }

    }


//dani yalechan!
// yoel yalechan!