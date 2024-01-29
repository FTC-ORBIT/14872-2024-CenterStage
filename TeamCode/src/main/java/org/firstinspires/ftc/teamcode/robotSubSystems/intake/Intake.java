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

    public static void test(Gamepad gamepad, Telemetry telemetry){
        if (gamepad.a){
            motor.setPower(-1);
        }else if (gamepad.b){
            motor.setPower(0);
        }else if (gamepad.y){
            motor.setPower(1);
        }
                telemetry.addData("pos", motor.getCurrentPosition());

    }
}
