package org.firstinspires.ftc.teamcode.robotSubSystems.conveyor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class Conveyor {
    public static DcMotor conveyorMotor;
    public static float pos;
    public static float power;

    public static void init(HardwareMap hardwareMap){
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void operate(ConveyorState state) {
        switch (state){
            case STOP:
            default:
                power = 0;
                break;
            case MOVE:
                power = ConveyorConstants.move;
                break;
            case REVERSE:
                power = ConveyorConstants.reverse;
                break;
        }
        conveyorMotor.setPower(power);
    }
}
