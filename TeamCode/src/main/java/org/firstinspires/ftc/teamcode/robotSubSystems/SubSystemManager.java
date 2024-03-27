package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.MaxHeightForFourbarDelay;
import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel.Fixpixel;
import org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel.FixpixelState;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.Fourbar;
import org.firstinspires.ftc.teamcode.robotSubSystems.fourbar.FourbarState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robotSubSystems.outtake.OuttakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.PlaneState;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;

    private static IntakeState intakeState = IntakeState.STOP;
    public static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static OuttakeState outtakeState = OuttakeState.CLOSED;
    private static FourbarState fourbarState = FourbarState.REVERSE;
    public static FixpixelState fixpixelState = FixpixelState.CLOSE;
    private static PlaneState planeState = PlaneState.STOP;
    private static Delay delayElevator = new Delay(0.75f);
    private static Delay intakeDelay = new Delay(1f);
    private static boolean toggleButton = true;
    private static boolean FixPixelToggleButton = false;
    private static ElapsedTime elapsedTime = new ElapsedTime();
    private static boolean ElevatorToggleButton = false;
    public static RobotState wanted = RobotState.TRAVEL;
    private static boolean lastLeftBumper = false;
    private static int outtakeCounter = 0;


    private static RobotState getState(Gamepad gamepad) {
        if (gamepad.b || gamepad.a || gamepad.x || gamepad.y || gamepad.right_bumper || gamepad.back || gamepad.dpad_up) {
            ElevatorToggleButton = false;
        }
        if (gamepad.x || gamepad.y ||  gamepad.right_bumper){
            outtakeCounter =0;
        }
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                : gamepad.x ? RobotState.MIN : gamepad.y ? RobotState.LOW :gamepad.start ? RobotState.DEPLETE: gamepad.dpad_up ? RobotState.FIXPIXEL : gamepad.right_bumper ? RobotState.MID: lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver) {

        switch (stateFromDriver) {
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
            case DEPLETE:
                break;
            case FIXPIXEL:
                break;
            case MIN:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
//        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));
        wanted = getState(gamepad1);



        if (wanted.equals(RobotState.TRAVEL) || wanted.equals(RobotState.INTAKE) || wanted.equals(RobotState.DEPLETE)){
            if (!wanted.equals(lastState)) {
                if (Elevator.getPos() < 1700) {
                    delayElevator.startAction(GlobalData.currentTime);
                } else {
                    delayElevator.startAction(GlobalData.currentTime - 1);
                }
            }
        }
        if (wanted.equals(RobotState.INTAKE) && lastState.equals(RobotState.TRAVEL)) {
            intakeDelay.startAction(GlobalData.currentTime);
        }
        if (!wanted.equals(RobotState.TRAVEL)) {
            elapsedTime.reset();
        }
        switch (wanted) {
            case TRAVEL:
                outtakeState = OuttakeState.CLOSED;
                if (intakeDelay.isDelayPassed()) {
                    intakeState = IntakeState.STOP;
                }
                fourbarState = FourbarState.REVERSE;
                if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                    elevatorState = ElevatorStates.INTAKE;
                }
                break;
            case INTAKE:
                intakeState = IntakeState.COLLECT;
                if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                    elevatorState = ElevatorStates.INTAKE;
                }
                outtakeState = OuttakeState.OPEN;
                fourbarState = FourbarState.COLLECT;
                fixpixelState = FixpixelState.CLOSE;
                break;
            case LOW:
                intakeState = IntakeState.STOP;
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    outtakeCounter++;
                }
                if (!ElevatorToggleButton) elevatorState = ElevatorStates.LOW;
                if (outtakeCounter % 3 == 0) { // i wrote the 3 on this line\
                    outtakeState = OuttakeState.CLOSED;
                } else if (outtakeCounter % 3 == 1) {
                    outtakeState = OuttakeState.OUT;
                } else if ((outtakeCounter % 3) == 2) {
                    outtakeState = OuttakeState.TOWOUT;
                }
                if (minHeightToOpenFourbar <= Elevator.getPos()) {
                    fourbarState = FourbarState.MID;
                }
                fixpixelState = FixpixelState.CLOSE;
                lastLeftBumper = gamepad1.left_bumper;
                break;
            case HIGH:
                break;
            case DEPLETE:
                fourbarState = FourbarState.REVERSE;
                intakeState = IntakeState.DEPLETE;
                if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                    elevatorState = ElevatorStates.INTAKE;
                }
                outtakeState = OuttakeState.CLOSED;
                fixpixelState = FixpixelState.CLOSE;
                break;
            case FIXPIXEL:
                outtakeState = OuttakeState.CLOSED;
                intakeState = IntakeState.STOP;
                fourbarState = FourbarState.REVERSE;
                elevatorState = ElevatorStates.FIX;
                fixpixelState = FixpixelState.CLOSE;
                break;
            case MIN:
                intakeState = IntakeState.STOP;
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    outtakeCounter++;
                }
                if (!ElevatorToggleButton) elevatorState = ElevatorStates.MIN;
                if (outtakeCounter % 3 == 0) { // i wrote the 3 on this line\
                    outtakeState = OuttakeState.CLOSED;
                } else if (outtakeCounter % 3 == 1) {
                    outtakeState = OuttakeState.OUT;
                } else if ((outtakeCounter % 3) == 2) {
                    outtakeState = OuttakeState.TOWOUT;
                }
                if (minHeightToOpenFourbar <= Elevator.getPos()) {
                    fourbarState = FourbarState.MID;
                }
                fixpixelState = FixpixelState.CLOSE;
                lastLeftBumper = gamepad1.left_bumper;
                break;
            case MID:
                intakeState = IntakeState.STOP;
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    outtakeCounter++;
                }
                if (!ElevatorToggleButton) elevatorState = ElevatorStates.MID;
                if (outtakeCounter % 3 == 0) { // i wrote the 3 on this line\
                    outtakeState = OuttakeState.CLOSED;
                } else if (outtakeCounter % 3 == 1) {
                    outtakeState = OuttakeState.OUT;
                } else if ((outtakeCounter % 3) == 2) {
                    outtakeState = OuttakeState.TOWOUT;
                }
                if (minHeightToOpenFourbar <= Elevator.getPos()) {
                    fourbarState = FourbarState.MID;
                }
                fixpixelState = FixpixelState.CLOSE;
                lastLeftBumper = gamepad1.left_bumper;
                break;
        }
        if (gamepad1.back) {
            fourbarState = FourbarState.REVERSE;
            elevatorState = ElevatorStates.CLIMB;
        }
        if (gamepad2.x || gamepad2.y || gamepad2.dpad_down) FixPixelToggleButton =false;
        if (gamepad2.x && !FixPixelToggleButton){
            fixpixelState =FixpixelState.MIN;
        }else if (gamepad2.y && !FixPixelToggleButton){
            fixpixelState = FixpixelState.LOW;
        } else if (gamepad2.dpad_down && !FixPixelToggleButton) {
            fixpixelState = FixpixelState.CLOSE;
        } else  if (gamepad2.right_bumper){
            fixpixelState = FixpixelState.THIRD;
        }else if (gamepad2.left_stick_y != 0) {
            FixPixelToggleButton = true;
            fixpixelState = FixpixelState.OVERRIDE;
        }

        if (gamepad1.right_stick_y != 0) {
            elevatorState = ElevatorStates.OVERRIDE;
            ElevatorToggleButton = true;
        }
        if (gamepad1.dpad_up) fourbarState = FourbarState.REVERSE;
        Intake.operate(intakeState);
        Outtake.operate(outtakeState);
        Elevator.operateTeleop(elevatorState, gamepad1, telemetry, gamepad2);
        Fourbar.operateTeleop(fourbarState);
        Fixpixel.operate(fixpixelState , gamepad2 , telemetry);

        lastState = wanted;
        if (gamepad1.dpad_down) OrbitGyro.resetGyro();
        if (gamepad2.back) Plane.operate(PlaneState.THROW);
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", wanted);
        telemetry.addData("last state", lastState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delayElevator", delayElevator.isDelayPassed());
        telemetry.addData("intakeDelay", intakeDelay.isDelayPassed());
        telemetry.addData("elevator", Elevator.elevatorMotor.getCurrentPosition());
        telemetry.addData("elevator2", Elevator.elevatorMotor2.getCurrentPosition());
        telemetry.addData("fourBar", Fourbar.servo.getPosition());
        telemetry.addData("outtake", Outtake.servo.getPosition());
        telemetry.addData("plane", Plane.planeServo.getPosition());
        telemetry.addData("time for deplete in travel", elapsedTime);
//        telemetry.addData("color" , OrbitColorSensor.color);
    }
}

//dani yalechan!
// yoel yalechan!