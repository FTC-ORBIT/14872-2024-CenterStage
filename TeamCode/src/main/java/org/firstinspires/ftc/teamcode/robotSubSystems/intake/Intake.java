package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public static final DcMotor motors[] = new DcMotor[2];
    private static float power;

    public static void init(HardwareMap hardwareMap) {

        motors[0] = hardwareMap.get(DcMotor.class, "IntakeR");
        motors[1] = hardwareMap.get(DcMotor.class, "IntakeL");

        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public static void operate(IntakeState state) {
        switch (state) {
            case COLLECT:
                power = IntakeConstants.intakePower;
                break;
            case STOP:
                power = 0;
                break;
            case DEPLETE:
                power = IntakeConstants.depletePower;
                break;
        }

        for (final DcMotor motor : motors)
            motor.setPower(power);

    }

    public static void testMotors(Telemetry telemetry){
        int num = 0;
            for (final DcMotor motor : motors){
                motor.setPower(0.5);
                telemetry.addData(String.valueOf(num), motors[num].getCurrentPosition());
                num++;
            }
    }
}
