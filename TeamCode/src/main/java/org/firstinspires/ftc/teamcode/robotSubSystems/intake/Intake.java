package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public static DcMotor motor;
    private static float power;

    public static void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void operate(IntakeState state) {
        switch (state){
            case STOP:
            default:
                power = 0;
                break;
            case COLLECT:
                power = IntakeConstants.intakePower;
                break;
            case DEPLETE:
                power = IntakeConstants.depletePower;
                break;
        }
        motor.setPower(power);
    }

    public static void test(Telemetry telemetry){
        telemetry.addData("intakeMotor", motor.getCurrentPosition());

    }
}
