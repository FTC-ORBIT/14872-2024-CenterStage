package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
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
    private static Delay delayElevator = new Delay(1.5f);
    private static Delay intakeDelay = new Delay(1f);
    private static boolean toggleButton = true;
    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLIMB :gamepad.x ? RobotState.LOW:gamepad.y ? RobotState.MID:  gamepad.right_bumper  ? RobotState.DEPLETE: lastState;
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
            case DEPLETE:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));

            if ((wanted.equals(RobotState.TRAVEL)|| wanted.equals(RobotState.INTAKE)) && (lastState.equals(RobotState.LOW) || lastState.equals(RobotState.MID) || lastState.equals(RobotState.HIGH))){
                delayElevator.startAction(GlobalData.currentTime);
            }
            if (wanted.equals(RobotState.INTAKE) && lastState.equals(RobotState.TRAVEL)){
                intakeDelay.startAction(GlobalData.currentTime);
            }
            switch (wanted) {
                case TRAVEL:
                    outtakeState = OuttakeState.CLOSED;
                    if (intakeDelay.isDelayPassed()) {
                        intakeState = IntakeState.STOP;
                    }
                    fourbarState = FourbarState.REVERSE;
                    if (delayElevator.isDelayPassed()) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
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
                    if (gamepad1.left_bumper){
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    climbState = ClimbState.DOWN;
                    break;
//                case HIGH:
//                    intakeState = IntakeState.STOP;
//                    elevatorState = ElevatorStates.HIGH;
//                    if (gamepad1.left_bumper) {
//                        outtakeState = OuttakeState.OPEN;
//                    }
//                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
//                        fourbarState = FourbarState.MOVE;
//                    }else {
//                        fourbarState = FourbarState.REVERSE;
//                    }
//                    climbState = ClimbState.DOWN;
//                    break;
                case CLIMB:
                    intakeState = IntakeState.STOP;
                    elevatorState = ElevatorStates.INTAKE;
                    outtakeState = OuttakeState.CLOSED;
                    fourbarState = FourbarState.REVERSE;
                    climbState = ClimbState.UP;
                    break;
                case DEPLETE:
                    fourbarState = FourbarState.REVERSE;
                    intakeState = IntakeState.DEPLETE;
                    elevatorState = ElevatorStates.INTAKE;
                    outtakeState = OuttakeState.CLOSED;
                    climbState = ClimbState.DOWN;
            }
        if (gamepad1.dpad_right) Intake.operate(IntakeState.STOP);
        if (gamepad1.dpad_up){
            if  (toggleButton == true){
             outtakeState.equals(OuttakeState.OPEN);
            }else {
                outtakeState.equals(OuttakeState.CLOSED);
            }
            Outtake.operate(outtakeState);
            toggleButton = !toggleButton;
        }

            Intake.operate(intakeState);
            Outtake.operate(outtakeState);
            Elevator.operate(elevatorState, gamepad1, telemetry );
            Fourbar.operate(fourbarState,gamepad1,telemetry);
//            Climb.operate(climbState, gamepad1);
            lastState = wanted;
            if (gamepad1.dpad_down) OrbitGyro.resetGyro();
        if (gamepad1.options) Plane.operate(PlaneState.THROW);
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delay" , delayElevator.isDelayPassed());
    }
}