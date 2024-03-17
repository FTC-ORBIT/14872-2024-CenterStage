package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public static DcMotor motor;
    //public static DcMotor motor2;
    private static float power;

    public static void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");

        //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        //motor2.setPower(power);
    }

    public static void test(Gamepad gamepad1 , Telemetry telemetry){
        if (gamepad1.a){
            motor.setPower(-1);
            //motor2.setPower(-1);
        }else if (gamepad1.b){
            motor.setPower(1);
            //motor2.setPower(1);
        }else {
            motor.setPower(0);
            //motor2.setPower(0);
        }
        telemetry.addData("intakeMotor", motor.getCurrentPosition());
        //telemetry.addData("intakeMotor2", motor2.getCurrentPosition());
    }
}
//dani yalechan!
// yoel yalechan!