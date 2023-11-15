package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.deplete.Deplete;
import org.firstinspires.ftc.teamcode.robotSubSystems.deplete.DepleteState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    private  static IntakeState intakeState = IntakeState.STOP;
    private  static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static DepleteState depleteState = DepleteState.CLOSED;

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
            case TRAVEL:
                break;
            case CLIMB:
                break;
        }
        return stateFromDriver;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));


            switch (wanted) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    depleteState = DepleteState.CLOSED;
                    break;
                case INTAKE:
                        intakeState = IntakeState.COLLECT;
                        elevatorState = ElevatorStates.INTAKE;
                        depleteState = DepleteState.CLOSED;
                    break;
                case LOW:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.LOW;
                    if (gamepad1.right_bumper){
                        depleteState = DepleteState.RIGHT_OPEN;
                    }else if (gamepad1.left_bumper){
                        depleteState = DepleteState.LEFT_OPEN;
                    }else if (gamepad1.left_stick_button){
                        depleteState = DepleteState.OPEN;
                    }else {
                        depleteState = DepleteState.CLOSED;
                    }
                    break;
                case MID:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.MID;
                    if (gamepad1.right_bumper){
                        depleteState = DepleteState.RIGHT_OPEN;
                    }else if (gamepad1.left_bumper){
                        depleteState = DepleteState.LEFT_OPEN;
                    }else if (gamepad1.left_stick_button){
                        depleteState = DepleteState.OPEN;
                    }else {
                        depleteState = DepleteState.CLOSED;
                    }
                    break;
                case CLIMB:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    depleteState = DepleteState.CLOSED;
                    break;
            }

        Intake.operate(intakeState);
            Deplete.operate(depleteState);

        lastState = wanted;
        if (gamepad1.dpad_down) OrbitGyro.resetGyro();
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
    }
}
