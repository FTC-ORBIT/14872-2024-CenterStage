package org.firstinspires.ftc.teamcode.robotSubSystems.fourbar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Fourbar {
    public static Servo servo;
    public static float pos;
    public static boolean lastLeft = false;
    public static boolean lastRight = false;
    public static boolean lastRT = false;
    public static boolean lastLT = false;
    public static void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "fourbarServo");
    }
    public static void operate(FourbarState state, Gamepad gamepad, Telemetry telemetry) {
        switch (state){
            case MOVE:
                pos = FourbarConstants.move;
                break;
            case REVERSE:
               pos = FourbarConstants.reverse;
                break;
            case MOVETOMID:
                pos = FourbarConstants.midMove;

        }

          servo.setPosition(pos);
    }
    public static void test(Gamepad gamepad, Telemetry telemetry){

//        if (gamepad.right_bumper) pos = FourbarConstants.move;
//        if (gamepad.left_bumper) pos = FourbarConstants.reverse;
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
        servo.setPosition(pos);
        lastLeft = gamepad.left_bumper;
        lastRight = gamepad.right_bumper;
        lastLT = gamepad.dpad_left;
        lastRT = gamepad.dpad_right;
    }
}
