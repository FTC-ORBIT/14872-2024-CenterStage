package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel.Fixpixel;
import org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel.FixpixelState;
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
    private static FixpixelState fixpixelState = FixpixelState.CLOSE;
    private static Delay delayElevator = new Delay(1.3f);
    private static Delay intakeDelay = new Delay(1f);
    private static boolean toggleButton = true;
    private static boolean ElevatorToggleButton = false;
    private static RobotState getState(Gamepad gamepad) {
        if (gamepad.b || gamepad.a || gamepad.dpad_left || gamepad.x || gamepad.y || gamepad.right_bumper || gamepad.back){
            ElevatorToggleButton = false;
        }
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        :gamepad.x ? RobotState.LOW:gamepad.y ? RobotState.MID: gamepad.back ? RobotState.FIXPIXEL: gamepad.right_bumper  ? RobotState.DEPLETE: lastState;
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
            case DEPLETE:
                break;
            case FIXPIXEL:
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
                    if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
                    fixpixelState = FixpixelState.CLOSE;
                    break;
                case INTAKE:
                    intakeState = IntakeState.COLLECT;
                    if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
                    outtakeState = OuttakeState.OPEN;
                    fourbarState = FourbarState.REVERSE;
                    fixpixelState = FixpixelState.CLOSE;
                    break;
                case LOW:
                    intakeState = IntakeState.STOP;
                    if (!ElevatorToggleButton) elevatorState = ElevatorStates.LOW;
                    if (gamepad1.left_bumper) {
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    fixpixelState = FixpixelState.CLOSE;
                    break;
                case MID:
                    intakeState = IntakeState.STOP;
                    if (!ElevatorToggleButton) elevatorState = ElevatorStates.MID;
                    if (gamepad1.left_bumper){
                        outtakeState = OuttakeState.OPEN;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MOVE;
                    }else {
                        fourbarState = FourbarState.REVERSE;
                    }
                    fixpixelState = FixpixelState.CLOSE;
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
                case DEPLETE:
                    fourbarState = FourbarState.REVERSE;
                    intakeState = IntakeState.DEPLETE;
                    if (!ElevatorToggleButton)  elevatorState = ElevatorStates.INTAKE;
                    outtakeState = OuttakeState.CLOSED;
                    fixpixelState = FixpixelState.CLOSE;
                    break;
                case FIXPIXEL:
                    outtakeState = OuttakeState.CLOSED;
                    if (intakeDelay.isDelayPassed()) {
                        intakeState = IntakeState.STOP;
                    }
                    fourbarState = FourbarState.REVERSE;
                    if (delayElevator.isDelayPassed() && !ElevatorToggleButton) {
                        elevatorState = ElevatorStates.INTAKE;
                    }
                    fixpixelState = FixpixelState.OPEN;
                    break;
            }
            if (gamepad1.dpad_right)intakeState = IntakeState.STOP;
        if (gamepad1.dpad_up){
            if  (toggleButton){
             outtakeState.equals(OuttakeState.OPEN);
            }else {
                outtakeState.equals(OuttakeState.CLOSED);
            }
            Outtake.operate(outtakeState);
            toggleButton = !toggleButton;
        }
        if (gamepad1.right_stick_y != 0 ){
            elevatorState = ElevatorStates.OVERRIDE;
            ElevatorToggleButton = true;
        }


            Intake.operate(intakeState);
            Outtake.operate(outtakeState);
            Elevator.operate(elevatorState, gamepad1, telemetry );
            Fourbar.operate(fourbarState,gamepad1,telemetry);
            Fixpixel.operate(fixpixelState , gamepad1 , telemetry);
            lastState = wanted;
            if (gamepad1.dpad_down) OrbitGyro.resetGyro();
        if (gamepad1.options) Plane.operate(PlaneState.THROW);
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delay" , delayElevator.isDelayPassed());
        telemetry.addData("intakeDelay" , intakeDelay.isDelayPassed());
        telemetry.addData("elevator" , Elevator.elevatorMotor.getCurrentPosition());
        telemetry.addData("fourbar" , Fourbar.servo.getPosition());
        telemetry.addData("fixPixle-Servo_1" , Fixpixel.servo.getPosition());
        telemetry.addData("fixPixle-Servo_2" , Fixpixel.servo2.getPosition());
        telemetry.addData("outtake" , Outtake.servo.getPosition());
        telemetry.addData("plane" , Plane.planeServo.getPosition());
    }
}