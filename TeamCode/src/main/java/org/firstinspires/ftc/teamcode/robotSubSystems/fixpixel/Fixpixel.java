package org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Fixpixel {
    public static Servo servo;
    public static Servo servo2;
    public static float pos;
    public static boolean lastLeft = false;
    public static boolean lastRight = false;
    public static boolean lastRT = false;
    public static boolean lastlT = false;
    public static void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "fixPixelServo");
        pos = FixpixelConstants.close;
    }
    public static void operate(FixpixelState state , Gamepad gamepad , Telemetry telemetry){
        switch (state){
            case CLOSE:
                pos = FixpixelConstants.close;
                break;
            case OPEN:
                pos = FixpixelConstants.open;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos);
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
        if (gamepad.dpad_left && !lastlT){
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
        lastlT = gamepad.dpad_left;
        lastRT = gamepad.dpad_right;
        telemetry.addData("pos" , pos);
        telemetry.update();
    }

}
