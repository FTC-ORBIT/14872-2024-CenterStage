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
    public static float lastPos;
    public static float pos;
    public static float currentPos = 0;
    public static float currentPos2 = 0;
    public static final PID elevatorPID = new PID(ElevatorConstants.elevatorKp, ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, ElevatorConstants.elevatorKf, ElevatorConstants.elevatorIzone);
    public static final PID encoderPID = new PID(ElevatorConstants.encoderKp, ElevatorConstants.encoderKi, ElevatorConstants.encoderKd, ElevatorConstants.encoderKf, ElevatorConstants.encoderIzone);

    public static void init(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        elevatorMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // the y axis in the gamepad joysticks are inverted, so we use -gamepad to use a positive y
    // the y axis varies between 1 and -1.
    public static void operateTeleop(ElevatorStates state, Gamepad gamepad1, Telemetry telemetry) {
        switch (state) {
            case OVERRIDE:
                pos += -gamepad1.right_stick_y * ElevatorConstants.overrideFactor;
                break;
            case LOW:
                pos = ElevatorConstants.lowHeight;
                break;
            case MID:
                pos = ElevatorConstants.midHeight;
                break;
            case INTAKE:
            default:
                pos = ElevatorConstants.intakeHeight;
                break;
            case CLIMB:
                pos = ElevatorConstants.climbHeight;
                break;
            case MIN:
                pos = ElevatorConstants.minHeight;
                break;
            case FIX:
                pos = lastPos;
                break;
        }
        //TODO - elevator max = 3254 , 3244
        if (currentPos > 3240 || currentPos2 > 3240) pos -= 100;
        //TODO - elevator min = 0!
        if (0 > currentPos || 0 > currentPos2){
            pos += 10;
            telemetry.addLine("elevator below 0");
            telemetry.update();
        }
        lastPos = pos;
        currentPos = elevatorMotor.getCurrentPosition();
        currentPos2 = elevatorMotor2.getCurrentPosition();
        elevatorPID.setWanted(pos);
        encoderPID.setWanted(0);
            elevatorMotor.setPower(elevatorPID.update(currentPos, telemetry) + encoderPID.update(currentPos - currentPos2 , telemetry));
            elevatorMotor2.setPower(elevatorPID.update(currentPos2, telemetry) + encoderPID.update(currentPos2 - currentPos , telemetry));



    telemetry.addData("pos", currentPos);
        telemetry.addData("pos2", currentPos2);
    }

    public static void operateAutonomous (ElevatorStates state, Telemetry telemetry) {
            switch (state) {
                case LOW:
                    pos = ElevatorConstants.lowHeight;
                    break;
                case MID:
                    pos = ElevatorConstants.midHeight;
                    break;
                case INTAKE:
                default:
                    pos = ElevatorConstants.intakeHeight;
                    break;
                case CLIMB:
                    pos = ElevatorConstants.climbHeight;
                    break;
                case MIN:
                    pos = ElevatorConstants.autoHeight;
                    break;
                case AUTO:
                    pos = ElevatorConstants.autoHeightFar;
                    break;
            }
            if (pos == ElevatorConstants.autoHeight) {
                while (currentPos < pos || currentPos2 < pos) {
                    currentPos = elevatorMotor.getCurrentPosition();
                    currentPos2 = elevatorMotor2.getCurrentPosition();
                    elevatorPID.setWanted(pos);
                    encoderPID.setWanted(0);
                    elevatorMotor.setPower(elevatorPID.update(currentPos, telemetry) + encoderPID.update(currentPos - currentPos2, telemetry));
                    elevatorMotor2.setPower(elevatorPID.update(currentPos2, telemetry) + encoderPID.update(currentPos2 - currentPos, telemetry));
                    telemetry.addData("pos", currentPos);
                    telemetry.addData("pos2", currentPos2);
                }
            } else {
                while (currentPos > pos && currentPos2 > pos) {
                    currentPos = elevatorMotor.getCurrentPosition();
                    currentPos2 = elevatorMotor2.getCurrentPosition();
                    elevatorPID.setWanted(pos);
                    encoderPID.setWanted(0);
                    elevatorMotor.setPower(elevatorPID.update(currentPos, telemetry) + encoderPID.update(currentPos - currentPos2, telemetry));
                    elevatorMotor2.setPower(elevatorPID.update(currentPos2, telemetry) + encoderPID.update(currentPos2 - currentPos, telemetry));

                    telemetry.addData("pos", currentPos);
                    telemetry.addData("pos2", currentPos2);
                    telemetry.update();
                }
            }
        elevatorMotor.setPower(0);
        elevatorMotor2.setPower(0);
    }

    public static double getPos(){
        return currentPos;
    }

    public static void test(Gamepad gamepad, Telemetry telemetry){


//            elevatorMotor.setPower(elevatorPID.update(currentPos, telemetry) + encoderPID.update(currentPos - currentPos2, telemetry));
//            elevatorMotor2.setPower(elevatorPID.update(currentPos2, telemetry) + encoderPID.update(currentPos2 - currentPos, telemetry));
        elevatorMotor.setPower(-gamepad.right_stick_y * 10 );
        elevatorMotor2.setPower(-gamepad.right_stick_y * 10);

    }
}
//dani yalechan!
// yoel yalechan!