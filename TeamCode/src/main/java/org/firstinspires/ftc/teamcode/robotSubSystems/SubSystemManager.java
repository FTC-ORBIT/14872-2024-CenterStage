package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;
import static org.firstinspires.ftc.teamcode.robotData.Constants.robotRadius;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.climb.Climb;
import org.firstinspires.ftc.teamcode.robotSubSystems.climb.ClimbState;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.PlaneState;
public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    private  static IntakeState intakeState = IntakeState.STOP;
    private  static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static OuttakeState outtakeState = OuttakeState.CLOSED;
    private static FourbarState fourbarState = FourbarState.REVERSE;
    private static ClimbState climbState = ClimbState.DOWN;
    private static Delay delayElevator = new Delay(1f);
    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLIMB :gamepad.x ? RobotState.LOW:gamepad.y ? RobotState.MID: gamepad.right_bumper ? RobotState.HIGH: lastState;
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
            case CLIMB:
                break;

        }
        return stateFromDriver;
    }


    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));

            if ((wanted.equals(RobotState.TRAVEL)|| wanted.equals(RobotState.INTAKE)) && (lastState.equals(RobotState.LOW) || lastState.equals(RobotState.MID) || lastState.equals(RobotState.HIGH))){
                delayElevator.startAction(GlobalData.currentTime);
            }
            switch (wanted) {
                case TRAVEL:
                    outtakeState = OuttakeState.CLOSED;
                    intakeState = IntakeState.STOP;
                    if (delayElevator.isDelayPassed()) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
                    fourbarState = FourbarState.REVERSE;
                    climbState = ClimbState.DOWN;
                    break;
                case INTAKE:
                    intakeState = IntakeState.COLLECT;
                    if (delayElevator.isDelayPassed()) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
                    outtakeState = OuttakeState.OPEN;
                    fourbarState = FourbarState.REVERSE;
                    climbState = ClimbState.DOWN;
                    break;
                case LOW:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.LOW;
                    if (gamepad1.left_bumper) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    climbState = ClimbState.DOWN;
                    break;
                case MID:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.MID;
                    if (gamepad1.dpad_down) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    climbState = ClimbState.DOWN;
                case HIGH:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.HIGH;
                    if (gamepad1.dpad_down) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    climbState = ClimbState.DOWN;
                    break;
                case CLIMB:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    outtakeState = OuttakeState.CLOSED;
                    fourbarState = FourbarState.REVERSE;
                    climbState = ClimbState.UP;
                    break;

            }
            Intake.operate(intakeState);
            Outtake.operate(outtakeState);
            Elevator.operate(elevatorState, gamepad1, telemetry );
            Fourbar.operate(fourbarState,gamepad1,telemetry);
//            Climb.operate(climbState, gamepad1);
            lastState = wanted;
            if (gamepad1.dpad_down) OrbitGyro.resetGyro();
        if (gamepad1.left_bumper) Plane.operate(PlaneState.THROW);
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delay" , delayElevator.isDelayPassed());
    }
}
