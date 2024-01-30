package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Elevator {
    public static DcMotor elevatorMotor;
    public static DcMotor elevatorMotor2;
    public static float pos;
    private static float currentPos = 0;
    private static float currentPos2 = 0;
    private static final PID elevatorPID = new PID(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd, ElevatorConstants.kf, ElevatorConstants.izone);
    private static final float power = 0.7f;
    public static void init(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        elevatorMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // the y axis in the gamepad joysticks are inverted, so we use -gamepad to use a positive y
    // the y axis varies between 1 and -1.
    public static void operate(ElevatorStates state, Gamepad gamepad1, Telemetry telemetry) {
        switch (state) {
            case OVERRIDE:
//                elevatorMotor.setPower(-gamepad1.right_stick_y * ElevatorConstants.overrideFactor);
//                elevatorMotor2.setPower(-gamepad1.right_stick_y * ElevatorConstants.overrideFactor);
//           //     pos += -gamepad1.right_stick_y * ElevatorConstants.overrideFactor;
                break;
            case INTAKE:
            default:
                pos = ElevatorConstants.intakeHeight;
                break;
            case LOW:
                pos = ElevatorConstants.lowHeight;
                break;
            case MID:
                pos = ElevatorConstants.midHeight;
                break;
            case HIGH:
                pos = ElevatorConstants.highHeight;
                break;
        }
        currentPos = elevatorMotor.getCurrentPosition();
        currentPos2 = elevatorMotor2.getCurrentPosition();
        elevatorPID.setWanted(pos);
        if (power > elevatorPID.update(currentPos)){
            elevatorMotor.setPower(power);
            elevatorMotor2.setPower(-power);
        }else {
            elevatorMotor.setPower(elevatorPID.update(currentPos));
            elevatorMotor2.setPower(elevatorPID.update(-currentPos2));
        }
//        elevatorMotor.setPower(-gamepad1.right_stick_y);
//        elevatorMotor2.setPower(-gamepad1.right_stick_y);

    telemetry.addData("pos", currentPos);
        telemetry.addData("pos2", currentPos2);
    }
    public static double getPos(){
        return currentPos;
    }

    public static void test(Gamepad gamepad , Telemetry telemetry){
        elevatorMotor.setPower(-gamepad.right_stick_y * 10 );
        elevatorMotor2.setPower(-gamepad.right_stick_y * 10);
    }
}
