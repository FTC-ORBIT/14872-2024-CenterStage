package org.firstinspires.ftc.teamcode.robotSubSystems.plane;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Plane {
    public static Servo planeServo;
    private static float pos = 1f;
    public static boolean lastLeft = false;
    public static boolean lastRight = false;
    public static boolean lastRT = false;
    public static boolean lastLT = false;
    public static void init(HardwareMap hardwareMap) {
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(PlaneConstants.stopPos);
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
    public static void test(Gamepad gamepad , Telemetry telemetry){
        if (gamepad.left_bumper &&  !lastLeft){
            pos += 0.05;
            if (pos > 1){
                pos = 1;

            }
        }else if (gamepad.right_bumper && !lastRight){
            pos -= 0.05;
            if (pos < 0){
                pos = 0;
            }
        }
        if (gamepad.dpad_left && !lastLT){
            pos += 0.001;
            if (pos > 1){
                pos = 1;
            }
        }else if (gamepad.dpad_right && !lastRT){
            pos -= 0.001;
            if (pos < 0){
                pos = 0;
            }
        }
        planeServo.setPosition(pos);
        lastLeft = gamepad.left_bumper;
        lastRight = gamepad.right_bumper;
        lastLT = gamepad.dpad_left;
        lastRT = gamepad.dpad_right;
        telemetry.addData("pos" , Plane.planeServo.getPosition());
        telemetry.update();
    }

    }

//dani yalechan!