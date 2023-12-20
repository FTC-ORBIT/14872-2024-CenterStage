package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    private  static IntakeState intakeState = IntakeState.STOP;
    private  static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static OuttakeState outtakeState = OuttakeState.CLOSED;
    private static FourbarState fourbarState = FourbarState.REVERSE;

    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLIMB : lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver){

        switch (stateFromDriver){
            case INTAKE:
                break;
            case LOW:
                break;
            case MID:
                break;
            case HIGH:
                break;
            case TRAVEL:
                break;
        }
        return stateFromDriver;
    }


    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));


            switch (wanted) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    outtakeState = OuttakeState.CLOSED;
                    fourbarState = FourbarState.REVERSE;
                    break;
                case INTAKE:
                        intakeState = IntakeState.COLLECT;
                        elevatorState = ElevatorStates.INTAKE;
                        outtakeState = OuttakeState.OPEN;
                        fourbarState = FourbarState.REVERSE;
                    break;
                case LOW:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.LOW;
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    break;
                case MID:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.MID;
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                case HIGH:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.HIGH;
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    break;
                case CLIMB:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.CLIMB;
                    outtakeState = OuttakeState.CLOSED;
                    fourbarState = FourbarState.REVERSE;
                    break;

            }

            Intake.operate(intakeState);
            Outtake.operate(outtakeState);
            Elevator.operate(elevatorState, gamepad1);
            Fourbar.operate(fourbarState);
            lastState = wanted;
            if (gamepad1.dpad_down) OrbitGyro.resetGyro();
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
    }
}
