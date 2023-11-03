package org.firstinspires.ftc.teamcode.robotSubSystems.center;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoCenter {
    static Servo center;
    static boolean lastRight;
    static boolean lastLeft;
    static double power;

    public static void init (HardwareMap hardwareMap){
        center = hardwareMap.get(Servo.class, "centerServo");
    }

    public static void operate (CenterState state){
        switch (state){
            case UP:
                center.setPosition(CenterConstants.upPose);
                break;
            case DOWN:
                center.setPosition(CenterConstants.downPose);
                break;
        }
    }

    public static void testArm (Gamepad gamepad, Telemetry telemetry) {
        if (!lastRight && gamepad.right_bumper){
            power = power + 0.005;
        } else if (!lastLeft && gamepad.left_bumper){
            power = power - 0.005;
        }
        center.setPosition(power);
        telemetry.addData("position", power);
        lastRight = gamepad.right_bumper;
        lastLeft = gamepad.left_bumper;

    }
}
