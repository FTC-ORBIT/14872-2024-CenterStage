package org.firstinspires.ftc.teamcode.robotSubSystems.climb;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Climb {
    public static DcMotor climbMotor;
    public static float pos;
    private static final PID climbPID = new PID(ClimbConstants.kp, ClimbConstants.ki, ClimbConstants.kd, ClimbConstants.kf, ClimbConstants.izone);

    public static void init(HardwareMap hardwareMap){
        climbMotor = hardwareMap.get(DcMotor.class, "climbMotor");

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void operate(ClimbState state, Gamepad gamepad1) {
        switch (state) {
            case DOWN:
                default:
                    pos = ClimbConstants.DOWN;
                    break;
            case UP:
                pos = ClimbConstants.UP;
                break;
            case OVERRIDE:
                climbMotor.setPower(-gamepad1.right_stick_y + ClimbConstants.kf);
                break;
        }

    climbPID.setWanted(pos);
        if (!state.equals(ClimbState.OVERRIDE)) {
            climbMotor.setPower(climbPID.update(climbMotor.getCurrentPosition()));
        }
    }
}
