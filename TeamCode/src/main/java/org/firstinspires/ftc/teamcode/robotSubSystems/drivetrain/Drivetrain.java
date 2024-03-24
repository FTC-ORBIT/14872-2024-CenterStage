package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.fixpixel.FixpixelState;

public class Drivetrain {

    public static final DcMotor[] motors = new DcMotor[4];
    public static float driveFactor = DrivetrainConstants.power;
    private static Pose2d pose;
    public static Vector lastPosition;
    // equal to the last Autonomous position?
    public static Vector lastVelocity = GlobalData.inAutonomous ? getVelocity_FieldCS() : null;

    public static void init(HardwareMap hardwareMap) {
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO if your initial robot position is not 0,0,0 make sure to fix the
        // position (look for the function in the documentry). might be setPoseEstimate

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public static void operate(final Vector velocity_W, float omega , Telemetry telemetry, Gamepad gamepad1) {
        final float robotAngle = (float) Math.toRadians(OrbitGyro.getAngle());
        final Vector velocity_RobotCS_W = velocity_W.rotate(-robotAngle);
        if (SubSystemManager.elevatorState == ElevatorStates.MIN || SubSystemManager.elevatorState == ElevatorStates.LOW  || SubSystemManager.fixpixelState != FixpixelState.CLOSE){
            if (omega > 0) omega = DrivetrainConstants.MaxOmegaSlow;
            else if (omega <0)omega =  -DrivetrainConstants.MaxOmegaSlow;
            else omega = 0;
            driveFactor = DrivetrainConstants.slowPower;
        }else if (SubSystemManager.elevatorState == ElevatorStates.MID){
            driveFactor = DrivetrainConstants.superSlowPower;
            if (omega > 0) omega = DrivetrainConstants.MaxOmegaSlowMid;
            else if (omega < 0) omega = -DrivetrainConstants.MaxOmegaSlowMid;
            else omega = 0;
        }else {
            driveFactor = DrivetrainConstants.power;
        }

        if(velocity_RobotCS_W.norm() <= Math.sqrt(0.005) && Math.abs(omega) == 0) stop();
        else drive(velocity_RobotCS_W, omega);



        telemetry.addData("lf power" , motors[0].getPower());
        telemetry.addData("rf power" , motors[1].getPower());
        telemetry.addData("lb power" , motors[2].getPower());
        telemetry.addData("rb power" , motors[3].getPower());
    }
    // did field centric

    public static Pose2d getPose_FieldCS() {
        return pose;
    }

    public static Vector getVelocity_FieldCS() {
        Vector position = new Vector((float) pose.getX(), (float) pose.getY());
        Vector deltaPosition = position.subtract(lastPosition);

        final Vector velocity = deltaPosition.scale(1 / GlobalData.deltaTime);

        lastPosition = position;
        return velocity;
    }

    public static Vector getAcceleration() {
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        Vector acceleration = deltaVelocity.scale(1 / GlobalData.deltaTime);

        lastVelocity = currentVelocity;
        return acceleration;
    }

    public static void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

    }

    public static void drive(Vector drive, double r) {
        final double lfPower = drive.y + drive.x + r;
        final double rfPower = drive.y - drive.x - r;
        final double lbPower = drive.y - drive.x + r;
        final double rbPower = drive.y + drive.x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));

        if (max > 1)  highestPower = max;
        motors[0].setPower(driveFactor * (lfPower / highestPower));
        motors[1].setPower(driveFactor * (rfPower / highestPower));
        motors[2].setPower(driveFactor * (lbPower / highestPower));
        motors[3].setPower(driveFactor * (rbPower / highestPower));

    }

    public static void testEncoder(Telemetry telemetry){
        telemetry.addData("lb", motors[2].getCurrentPosition());
        telemetry.addData("lf", motors[0].getCurrentPosition());
        telemetry.addData("rb-2", motors[3].getCurrentPosition());
        telemetry.addData("rf-3", motors[1].getCurrentPosition());
    }

    public static void testMotors(Gamepad gamepad, Telemetry telemetry){
        if (gamepad.dpad_down){motors[0].setPower(0.2);}
        else if (gamepad.dpad_left){motors[1].setPower(0.2);}
        else if (gamepad.dpad_up){motors[2].setPower(0.2);}
        else if (gamepad.dpad_right){motors[3].setPower(0.2);}
        telemetry.addData("lf", motors[0].getCurrentPosition());
        telemetry.addData("rf", motors[1].getCurrentPosition());
        telemetry.addData("lb", motors[2].getCurrentPosition());
        telemetry.addData("rb", motors[3].getCurrentPosition());
    }

}
//dani yalechan!
// yoel yalechan!