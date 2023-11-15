package org.firstinspires.ftc.teamcode.robotSubSystems.fourbar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Fourbar {
    public static DcMotor motor;
    public static float pos;
    public static PID fourbarPID = new PID(FourbarConstants.kp, FourbarConstants.ki, FourbarConstants.kd, FourbarConstants.kf, FourbarConstants.izone);


    public static void init(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotor.class, "FourbarMotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void operate(FourbarState state) {
        switch (state){
            case MOVE:
                pos = FourbarConstants.move;
                break;
            case REVERSE:
                pos = FourbarConstants.reverse;
                break;
        }
        fourbarPID.setWanted(pos);
    }
}
