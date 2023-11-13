package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Elevator {
    public static DcMotor elevatorMotor;
    public static float pos;
    private static final PID elevatorPID = new PID(ElevatorConstants.kp,ElevatorConstants.ki,ElevatorConstants.kd,ElevatorConstants.kf,ElevatorConstants.izone);
    public static void init(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotor.class,"elevatorMotor" );

        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void operate(ElevatorStates state, float overrideHeight){
        switch (state){
            case OVERRIDE:
                default:
                    pos = overrideHeight;
                break;
            case INTAKE:
                pos = ElevatorConstants.intakeHeight;
                break;
            case LOW:
                pos = ElevatorConstants.lowHeight;
                break;
            case MID:
                pos = ElevatorConstants.midHeight;
                break;
            }
            elevatorPID.setWanted(pos);
            elevatorMotor.setPower(elevatorPID.update(elevatorMotor.getCurrentPosition()));
        }
    }
