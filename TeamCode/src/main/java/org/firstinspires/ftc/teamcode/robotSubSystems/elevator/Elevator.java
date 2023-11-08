//package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;
//
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.groundHeight;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.highHeight;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.intakeHeight;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.kD;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.kP;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.lowHeight;
//import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.midHeight;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
//import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
//
//@Config
//public class Elevator {
//    private static DcMotorEx elevatorMotor;
//    private static final PID elevatorPID = new PID(kP, 0, kD, 0, 0);
//    private static float lastStateHeight;
//    private static boolean ableToChangeConeStacksFloor = false;
//    private static float elevatorPower = 0f;
//    public static  float wanted = 0;
//    public static float height = 0;
//    public static float lastHeight = 0;
//    public static int coneStacksFloor = 5;
//    public static int floor = 5;
//    private static final ElapsedTime time = new ElapsedTime();
//    public static final double depleteTime = 400;
//
//    public static void initAutonomous(HardwareMap hardwareMap) {
//        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
//        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        height = 0;
//        floor = 5;
//    }
//
//    public static void operateAutonomous(ElevatorStates elevatorState, Telemetry telemetry) {
//        height = Intake.motors[0].getCurrentPosition() - lastHeight;
//
//        switch (elevatorState) {
//            case INTAKE:
//                wanted = intakeHeight;
//                break;
//            case LOW:
//                wanted = lowHeight;
//                break;
//            case MID:
//                wanted = midHeight;
//                break;
//            case HIGH:
//                wanted = highHeight;
//                break;
//        }
//        elevatorPID.setWanted(wanted);
//        if (!elevatorState.equals(ElevatorStates.OVERRIDE)) {
//          elevatorPower = (float) elevatorPID.update(height);
//        }
//
//        elevatorMotor.setPower(elevatorPower + ElevatorConstants.constantPower);
//        telemetry.addData("height", height);
//        telemetry.addData("reachedHeight", reachedHeight());
//
//    }
//
//    public static void opereate(ElevatorStates elevatorState,Gamepad gamepad1, Telemetry telemetry) {
//        height = Intake.motors[0].getCurrentPosition() - lastHeight;
//
//        switch (elevatorState) {
//            case INTAKE:
//                wanted = intakeHeight;
//                break;
//            case LOW:
//                wanted = lowHeight;
//                break;
//            case MID:
//                wanted = midHeight;
//                break;
//            case HIGH:
//                wanted = highHeight;
//                break;
//            case OVERRIDE:
//                elevatorPower = -gamepad1.right_stick_y + ElevatorConstants.constantPower;
//                break;
//        }
//        elevatorPID.setWanted(wanted);
//        if (!elevatorState.equals(ElevatorStates.OVERRIDE)) {
//            elevatorPower = (float) elevatorPID.update(height);
//        }
//
//        elevatorMotor.setPower(elevatorPower + ElevatorConstants.constantPower);
//        telemetry.addData("height", height);
//        telemetry.addData("reachedHeight", reachedHeight());
//    }
//
//    public static float getMotorCurrent(Telemetry telemetry){
//        telemetry.addData("elevatorMotorCurrent", elevatorMotor.getCurrent(CurrentUnit.AMPS));
//        return (float) elevatorMotor.getCurrent(CurrentUnit.AMPS);
//    }
//
//    public static boolean reachedHeight (){
//        return height >= wanted - 400 && height <= wanted + 400;
//    }
//
//    public static void setWanted (float wantedNum){
//        wanted = wantedNum;
//    }
//
//    public static boolean reachedHeightVal (float wantedHeight){
//        return height >= wantedHeight - 200 && height <= wantedHeight + 200;
//    }
//    public static float getError(){
//        return wanted - height;
//    }
//
//    public static void setPower (float power) { elevatorMotor.setPower(-power);}
//    public static float getAndUpdateHeight(){
//        height = Intake.motors[0].getCurrentPosition();
//        return height;
//    }
//
//    public static void resetElevator (float height){
//        lastHeight = height;
//    }
//
//    public static void testMotors(Telemetry telemetry, Gamepad gamepad){
//        if (gamepad.a) elevatorMotor.setPower(0.5);
//        telemetry.addData("value", Drivetrain.motors[1].getCurrentPosition());
//        telemetry.addData("power", elevatorPower);
//    }
//
//
//
//}
