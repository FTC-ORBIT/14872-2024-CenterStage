package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.Constants.minHeightToOpenFourbar;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
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
    private  static IntakeState intakeState = IntakeState.STOP;
    public   static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static OuttakeState outtakeState = OuttakeState.CLOSED;
    private static FourbarState fourbarState = FourbarState.REVERSE;
    private static FixpixelState fixpixelState = FixpixelState.CLOSE;
    private static PlaneState planeState = PlaneState.STOP;
    private static Delay delayElevator = new Delay(0.6f);
    private static Delay intakeDelay = new Delay(1f);
    private static boolean toggleButton = true;
    private static boolean ElevatorToggleButton = false;
    public static RobotState wanted = RobotState.TRAVEL;
    private static RobotState getState(Gamepad gamepad) {
        if (gamepad.b || gamepad.a || gamepad.x || gamepad.y || gamepad.right_bumper || gamepad.back || gamepad.dpad_up){
            ElevatorToggleButton = false;
        }
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        :gamepad.x ? RobotState.MIN:gamepad.y ? RobotState.LOW: gamepad.back ? RobotState.FIXPIXEL:gamepad.dpad_up ? RobotState.MID: gamepad.right_bumper  ? RobotState.DEPLETE: lastState;
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
            case MIN:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad1));

            if ((wanted.equals(RobotState.TRAVEL)|| wanted.equals(RobotState.INTAKE)) && (lastState.equals(RobotState.LOW) || lastState.equals(RobotState.MID) || lastState.equals(RobotState.MIN))){
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
                        outtakeState = OuttakeState.OUT;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MID;
                    }
                    fixpixelState = FixpixelState.CLOSE;
                    break;
                case MID:
                    intakeState = IntakeState.STOP;
                    if (!ElevatorToggleButton) elevatorState = ElevatorStates.MID;
                    if (gamepad1.left_bumper){
                        outtakeState = OuttakeState.OUT;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MID;
                    }
                    fixpixelState = FixpixelState.CLOSE;
                    break;
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
                case MIN:
                    intakeState = IntakeState.STOP;
                    if (!ElevatorToggleButton) elevatorState = ElevatorStates.MIN;
                    if (gamepad1.left_bumper){
                        outtakeState = OuttakeState.OUT;
                    }
                    if (minHeightToOpenFourbar <=Elevator.getPos()) {
                        fourbarState = FourbarState.MID;
                    }
                    fixpixelState = FixpixelState.CLOSE;
                    break;
            }
            if (gamepad1.back) {
                fourbarState = FourbarState.REVERSE;
                elevatorState = ElevatorStates.CLIMB;
            }
            if (gamepad1.dpad_left){
                fourbarState = FourbarState.REVERSE;
                elevatorState = ElevatorStates.INTAKE;
            }

        if (gamepad1.right_stick_y != 0 ){
            elevatorState = ElevatorStates.OVERRIDE;
            ElevatorToggleButton = true;
        }


         Intake.operate(intakeState);
         Outtake.operate(outtakeState);
            Elevator.operateTeleop(elevatorState, gamepad1, telemetry );
            Fourbar.operateTeleop(fourbarState);
     //       Fixpixel.operate(fixpixelState , gamepad1 , telemetry);
            lastState = wanted;
            if (gamepad1.dpad_down) OrbitGyro.resetGyro();
        if (gamepad1.options && gamepad1.dpad_left || gamepad1.options && gamepad1.dpad_right) Plane.operate(PlaneState.THROW);
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delay" , delayElevator.isDelayPassed());
        telemetry.addData("intakeDelay" , intakeDelay.isDelayPassed());
        telemetry.addData("elevator" , Elevator.elevatorMotor.getCurrentPosition());
        telemetry.addData("elevator2" , Elevator.elevatorMotor2.getCurrentPosition());
        telemetry.addData("fourBar" , Fourbar.servo.getPosition());
//        telemetry.addData("fixPixel-Servo_1" , Fixpixel.servo.getPosition());
//        telemetry.addData("fixPixel-Servo_2" , Fixpixel.servo2.getPosition());
        telemetry.addData("outtake" , Outtake.servo.getPosition());
        telemetry.addData("plane" , Plane.planeServo.getPosition());
    }
}