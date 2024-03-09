package org.firstinspires.ftc.teamcode.robotSubSystems.fourbar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Fourbar {
    public static Servo servo;
    public static float pos;
    public static boolean lastLB = false;
    public static boolean lastRB = false;
    public static boolean lastRdpad = false;
    public static boolean lastLdpad = false;

    public static ElapsedTime time = new ElapsedTime();
    public static void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "fourbarServo");
        time.reset();
    }
    public static void operateTeleop(FourbarState state) {
        switch (state){
            case MOVE:
                pos = FourbarConstants.move;
                break;
            case REVERSE:
               pos = FourbarConstants.reverse;
                break;
            case MID:
                pos = FourbarConstants.midMove;
                break;
            case COLLECT:
                pos = FourbarConstants.collect;
                break;
        }

          servo.setPosition(pos);
    }

    public static void operateAutonomous(FourbarState state) {
            switch (state) {
                case MOVE:
                    pos = FourbarConstants.move;
                    break;
                case REVERSE:
                    pos = FourbarConstants.reverse;
                    break;
                case MID:
                    pos = FourbarConstants.midMove;
                    break;
            }
            servo.setPosition(pos);
        }

    public static void test(Gamepad gamepad, Telemetry telemetry){

//        if (gamepad.right_bumper) pos = FourbarConstants.midMove;
//        if (gamepad.left_bumper) pos = FourbarConstants.reverse;
        if (gamepad.left_bumper &&  !lastLB){
            pos += 0.05;
            if (pos > 1){
                pos = 1;

            }
        }else if (gamepad.right_bumper && !lastRB){
            pos -= 0.05;
            if (pos < 0){
                pos = 0;
            }
        }
        if (gamepad.dpad_left && !lastLdpad){
            pos += 0.001;
            if (pos > 1){
                pos = 1;
            }
        }else if (gamepad.dpad_right && !lastRdpad){
            pos -= 0.001;
            if (pos < 0){
                pos = 0;
            }
        }
        servo.setPosition(pos);
        lastLB = gamepad.left_bumper;
        lastRB = gamepad.right_bumper;
        lastLdpad = gamepad.dpad_left;
        lastRdpad = gamepad.dpad_right;
        telemetry.addData("pos" , servo.getPosition());
        telemetry.update();
    }
}
//dani yalechan!
// yoel yalechan!