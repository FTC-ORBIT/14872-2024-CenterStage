package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeConstants;

public class Intake {

    public static DcMotor motor;
    public static DcMotor motor2;
    public static Servo servo;
    private static float power;
    private static float pos;
    public static boolean lastLeft = false;
    public static boolean lastRight = false;
    public static boolean lastRT = false;
    public static boolean lastlT = false;
    public static boolean firstTime = true;


    public static void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class,"rakeServo");

    }

    public static void operate(IntakeState state) {
        switch (state){
            case STOP:
            default:
                power = 0;
                pos = IntakeConstants.stopPos;
                break;
            case COLLECT:
                if (SubSystemManager.rakeDelay.isDelayPassed()) {
                    power = IntakeConstants.intakePower;
                }
                pos = IntakeConstants.groundPos;
                break;
            case DEPLETE:
                if (SubSystemManager.rakeDelay.isDelayPassed()) {
                    power = IntakeConstants.depletePower;
                }
                pos = IntakeConstants.stackPos;
                break;
            case RAKE:
                if (SubSystemManager.rakeDelay.isDelayPassed()) {
                    power = IntakeConstants.intakePower;
                }
                pos = IntakeConstants.stackPos;
                break;
        }
        motor.setPower(power);
        motor2.setPower(power);
        servo.setPosition(pos);
    }

    public static void test(Gamepad gamepad, Telemetry telemetry) {

//        if (firstTime){
//            pos = 1;
//            firstTime = false;
//        }
        if (gamepad.dpad_left &&  !lastLeft){
            pos += 0.05;
            if (pos > 1){
                pos = 1;
            }
        }else if (gamepad.dpad_right && !lastRight){
            pos -= 0.05;
            if (pos < 0){
                pos = 0;
            }
        }
        if (gamepad.left_bumper && !lastlT){
            pos += 0.01;
            if (pos > 1){
                pos = 1;
            }
        }else if (gamepad.right_bumper && !lastRT){
            pos -= 0.01;
            if (pos < 0){
                pos = 0;
            }
        }

        servo.setPosition(pos);
        lastLeft = gamepad.dpad_left;
        lastRight = gamepad.dpad_right;
        lastlT = gamepad.left_bumper;
        lastRT = gamepad.right_bumper;
        telemetry.addData("rake pos" , servo.getPosition());
        telemetry.update();
    }
}
//dani yalechan!
// yoel yalechan!