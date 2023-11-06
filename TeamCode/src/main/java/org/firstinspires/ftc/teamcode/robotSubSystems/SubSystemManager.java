package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    private  static IntakeState intakeState = IntakeState.STOP;
    private  static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLIMB : lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver){
        switch (stateFromDriver){
            case INTAKE:
                if(GlobalData.hasGamePiece) return RobotState.TRAVEL;
                break;
            case DEPLETE:
                break;
            case TRAVEL:

                break;
            case CLIMB:
                break;
        }
        return stateFromDriver;
    }

    public static void setState(Gamepad gamepad, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad));
        if (wanted != null) {
            GlobalData.robotState = wanted;
        }
        setSubsystemToState(gamepad, gamepad2, telemetry);
    }

    public static ElevatorStates getElevatorStateFromDriver (Gamepad gamepad1) {
        return gamepad1.a ? ElevatorStates.LOW
                : gamepad1.b ? ElevatorStates.MID
                        : gamepad1.x ? ElevatorStates.HIGH : null;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {


            switch (GlobalData.robotState) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    break;
                case INTAKE:
                        intakeState = IntakeState.COLLECT;
                        elevatorState = ElevatorStates.INTAKE;
                    break;

                case DEPLETE:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.OVERRIDE;

                    break;
                case CLIMB:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.OVERRIDE;
                    break;
            }

        Intake.operate(intakeState);
            Elevator.opereate(elevatorState, gamepad1,telemetry);
        lastState = GlobalData.robotState;
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
    }
}
