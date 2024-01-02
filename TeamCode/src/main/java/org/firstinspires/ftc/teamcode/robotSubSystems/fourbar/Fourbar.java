package org.firstinspires.ftc.teamcode.robotSubSystems.fourbar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Fourbar {
    public static Servo servo;
    public static float pos;

    public static void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "fourbarServo");
    }
    public static void operate(FourbarState state, Gamepad gamepad, Telemetry telemetry) {
        switch (state){
            case MOVE:
                //pos = FourbarConstants.move;
                break;
            case REVERSE:
                //pos = FourbarConstants.reverse;
                break;

        }

          servo.setPosition(pos);
    }
}
